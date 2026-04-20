#!/usr/bin/env bash
# Map three HikVision PTZ cameras (192.168.1.218/.219/.220) to fixed
# V4L2 loopback devices /dev/video18, /dev/video19, /dev/video20 so
# downstream consumers can address each camera deterministically by
# device number.
#
# Camera <-> device mapping mirrors the last octet of each IP:
#   192.168.1.218 -> /dev/video18
#   192.168.1.219 -> /dev/video19
#   192.168.1.220 -> /dev/video20
#
# Based on rtsp_to_v4l2_.sh; see CLAUDE.md "Purpose" for why the
# credentials and ports are hardcoded.
#
# Usage:
#   ./rtsp_to_v4l2_multi.sh setup    # install deps + load module
#   ./rtsp_to_v4l2_multi.sh stream   # spawn 3 ffmpeg writers
#   ./rtsp_to_v4l2_multi.sh          # setup && stream
#
# Use ./rtsp_to_v4l2_teardown.sh to stop and release the devices.

set -euo pipefail

source /home/inno-controller/anaconda3/etc/profile.d/conda.sh
conda activate lerobot

python3 hkvision_related/goto_preset_all_ipcamera.py

camera_ips=(
    "192.168.1.218"
    "192.168.1.219"
    "192.168.1.220"
)
video_nrs=(18 19 20)
rtsp_user="admin"
rtsp_pass="peal2024"
rtsp_path="ISAPI/streaming/channels/02"
log_dir="/tmp/rtsp_to_v4l2"

setup_loopback() {
    sudo apt-get update
    sudo apt-get install -y \
        v4l2loopback-dkms \
        v4l2loopback-utils \
        ffmpeg

    # Build comma-separated lists for the three devices in one modprobe
    # call so they share a single module instance.
    local nrs_csv
    local labels_csv
    nrs_csv=$(IFS=, ; echo "${video_nrs[*]}")
    labels_csv="HikVision .218,HikVision .219,HikVision .220"

    # Reload only if any of the target devices are missing, so we don't
    # disrupt a session that's already up.
    local need_reload=0
    for nr in "${video_nrs[@]}"; do
        [ -e "/dev/video${nr}" ] || need_reload=1
    done

    if [ "${need_reload}" -eq 1 ]; then
        sudo modprobe -r v4l2loopback || true
        sudo modprobe v4l2loopback \
            devices=3 \
            video_nr="${nrs_csv}" \
            card_label="${labels_csv}" \
            exclusive_caps=1,1,1
    fi

    for nr in "${video_nrs[@]}"; do
        ls -l "/dev/video${nr}"
    done
}

stream_rtsp() {
    mkdir -p "${log_dir}"

    for i in "${!camera_ips[@]}"; do
        local ip="${camera_ips[$i]}"
        local nr="${video_nrs[$i]}"
        local device="/dev/video${nr}"
        local url="rtsp://${rtsp_user}:${rtsp_pass}@${ip}:554/${rtsp_path}"
        local log="${log_dir}/cam_${nr}.log"

        echo "Starting ${ip} -> ${device} (log: ${log})"
        # nohup + & : leave one ffmpeg per camera running in the
        # background so this script can return. Teardown script kills
        # them by matching '/dev/video' in the command line.
        nohup ffmpeg -rtsp_transport tcp \
                     -i "${url}" \
                     -f v4l2 \
                     -vcodec rawvideo \
                     -pix_fmt yuv420p \
                     "${device}" \
                     >"${log}" 2>&1 &
    done

    echo "All 3 ffmpeg writers started. Use rtsp_to_v4l2_teardown.sh"
    echo "to stop them."
}

case "${1:-all}" in
    setup)  setup_loopback ;;
    stream) stream_rtsp ;;
    all)    setup_loopback; stream_rtsp ;;
    *)
        echo "Usage: $0 {setup|stream|all}" >&2
        exit 1
        ;;
esac
