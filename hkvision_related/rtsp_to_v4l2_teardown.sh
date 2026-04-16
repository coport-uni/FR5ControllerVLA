#!/usr/bin/env bash
# Release the virtual V4L2 devices created by rtsp_to_v4l2_.sh /
# rtsp_to_v4l2_multi.sh. Kills any ffmpeg processes that are writing
# into /dev/videoN, then unloads the v4l2loopback kernel module so the
# devices disappear.
#
# Usage:
#   ./rtsp_to_v4l2_teardown.sh            # kill ffmpeg + rmmod
#   ./rtsp_to_v4l2_teardown.sh kill-only  # only stop the ffmpeg pipes

set -euo pipefail

kill_writers() {
    # Match ffmpeg invocations writing to /dev/videoN — the scripts in
    # this repo always pass the device path as the last argument.
    if pgrep -fa 'ffmpeg.*/dev/video' >/dev/null; then
        echo "Stopping ffmpeg writers..."
        pkill -TERM -f 'ffmpeg.*/dev/video' || true
        # Give them a moment to release the device, then SIGKILL any
        # stragglers.
        sleep 2
        pkill -KILL -f 'ffmpeg.*/dev/video' || true
    else
        echo "No ffmpeg writers running."
    fi
}

unload_module() {
    if lsmod | grep -q '^v4l2loopback'; then
        echo "Unloading v4l2loopback..."
        modprobe -r v4l2loopback
    else
        echo "v4l2loopback is not loaded."
    fi
}

case "${1:-all}" in
    kill-only) kill_writers ;;
    all)       kill_writers; unload_module ;;
    *)
        echo "Usage: $0 {all|kill-only}" >&2
        exit 1
        ;;
esac

echo "Remaining /dev/video* devices:"
ls /dev/video* 2>/dev/null || echo "  (none)"
