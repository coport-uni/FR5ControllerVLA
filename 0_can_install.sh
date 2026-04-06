#!/bin/bash
# can_install.sh -- Set up CAN bus inside a Docker container.
#
# Prerequisites:
#   - Container must run with --privileged (or equivalent caps)
#   - USB CAN adapter (e.g. candleLight) plugged into host
#
# Usage:
#   bash can_install.sh            # default 1 Mbps
#   bash can_install.sh 500000     # 500 kbps

set -e

BITRATE="${1:-1000000}"
CAN_IF="can0"

echo "=== [1/5] Installing packages ==="
apt-get update -qq
apt-get install -y -qq kmod ethtool can-utils iproute2

echo ""
echo "=== [2/5] Installing kernel modules ==="
KVER=$(uname -r)
apt-get install -y -qq \
    "linux-modules-${KVER}" \
    "linux-modules-extra-${KVER}"

echo ""
echo "=== [3/5] Loading CAN kernel modules ==="
modprobe can
modprobe can_raw
modprobe can_dev
modprobe gs_usb
echo "Loaded: can, can_raw, can_dev, gs_usb"

echo ""
echo "=== [4/5] Moving CAN interface into container ==="

# Wait for gs_usb to bind and create the interface.
sleep 1

# The interface lives in the host network namespace.
# Move it into this container's namespace.
if ip link show "${CAN_IF}" >/dev/null 2>&1; then
    echo "${CAN_IF} already visible in container."
else
    HOST_IF=$(nsenter --net=/proc/1/ns/net ip -br link show type can 2>/dev/null | awk '{print $1}' | head -1)
    if [ -z "${HOST_IF}" ]; then
        echo "ERROR: No CAN interface found in host namespace."
        echo "       Check that the USB adapter is plugged in."
        exit 1
    fi
    nsenter --net=/proc/1/ns/net ip link set "${HOST_IF}" netns $$
    echo "Moved ${HOST_IF} from host into container."
fi

echo ""
echo "=== [5/5] Configuring ${CAN_IF} (bitrate ${BITRATE}) ==="
ip link set "${CAN_IF}" type can bitrate "${BITRATE}"
ip link set "${CAN_IF}" up

echo ""
echo "=== Done ==="
ip -d link show "${CAN_IF}"
echo ""
ethtool -i "${CAN_IF}" | grep bus-info
echo ""
echo "CAN bus ready.  Test with:  candump ${CAN_IF}"
