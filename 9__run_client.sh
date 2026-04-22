#!/bin/bash
# FR5 async-inference client.
# Sends observations (joints + cameras) to the policy server and streams
# back action chunks, which are executed on the follower via ServoJ.
# Paired with 8__run_server.sh.
#
# NOTE: Set PRETRAINED to a real FR5 policy checkpoint (HuggingFace repo id
# or local path) before running.
source /home/inno-controller/anaconda3/etc/profile.d/conda.sh
conda activate lerobot

PRETRAINED="<FR5_POLICY_REPO_OR_PATH>"

python src/lerobot/async_inference/robot_client.py \
    --server_address=127.0.0.1:8088 \
    --robot.type=fairino_follower \
    --robot.ip_address=192.168.58.2 \
    --robot.gripper_enabled=true \
    --robot.id=fr5_follower \
    --robot.cameras="{ \
        top_left: {type: opencv, index_or_path: '/dev/video18', width: 640, height: 480, fps: 20}, \
        top_right: {type: opencv, index_or_path: '/dev/video19', width: 640, height: 480, fps: 20}, \
        hand: {type: intelrealsense, serial_number_or_name: '333422300435', width: 640, height: 480, fps: 30}}" \
    --pretrained_name_or_path="${PRETRAINED}" \
    --policy_type=smolvla \
    --policy_device=cuda \
    --actions_per_chunk=50 \
    --chunk_size_threshold=0.8 \
    --aggregate_fn_name=average \
    --debug_visualize_queue_size=false \
    --task="pick_red_colored_marker_to_box" \
    --fps=20
