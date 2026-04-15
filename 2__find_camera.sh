source /opt/conda/etc/profile.d/conda.sh
conda activate lerobot

# Usage: ./2__find_camera.sh [realsense|opencv]
# No arg searches both. RealSense exposes depth/metadata nodes
# (e.g. /dev/video2, /dev/video4) that OpenCV cannot decode, which
# emits harmless but noisy "read failed" warnings. Pass "realsense"
# or "opencv" to restrict the search and avoid the warnings.
CAMERA_TYPE="${1:-}"

python ./src/lerobot/find_cameras.py ${CAMERA_TYPE}
nautilus ./outputs/captured_images
