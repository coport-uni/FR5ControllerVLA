#!/bin/bash
# FR5-to-FR5 teleoperated dataset recording with cameras.
# Leader (192.168.59.2): drag-teach mode -- move by hand.
# Follower (192.168.58.2): mirrors leader via ServoJ.
# Observations: two HikVision OpenCV cameras (top_left, top_right) +
# one Intel RealSense (hand).
source /home/inno-controller/anaconda3/etc/profile.d/conda.sh
conda activate lerobot

HF_USER=$(hf auth whoami 2>/dev/null | head -n 1)
echo "$HF_USER"

export HF_XET_HIGH_PERFORMANCE=1

# Alternative hand camera: OpenCV instead of RealSense.
# hand: {type: opencv, index_or_path: '/dev/video4', width: 640, height: 480, fps: 30}

# Right Arrow (→): End episode and move to next
# Left Arrow (←): Cancel episode and re-record
# Escape (ESC): Stop session and upload dataset

JOB_NAME="FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle_into_another_brown_bottle"

# Remove any stale local copy so hf download starts from a clean state.
# rm -rf /home/inno-controller/workspace/SungwooVLA/FR5ControllerVLA/outputs/datasets/${JOB_NAME}

# hf download \
#     --repo-type dataset \
#     coport-uni/${JOB_NAME} \
#     --local-dir /home/inno-controller/workspace/SungwooVLA/FR5ControllerVLA/outputs/datasets/${JOB_NAME}

lerobot-record \
    --robot.type=fairino_follower \
    --robot.ip_address=192.168.58.2 \
    --robot.gripper_enabled=true \
    --robot.id=fr5_follower \
    --robot.cameras="{ \
        top_left: {type: opencv, index_or_path: '/dev/video18', width: 640, height: 480, fps: 20}, \
        top_right: {type: opencv, index_or_path: '/dev/video19', width: 640, height: 480, fps: 20}, \
        hand: {type: intelrealsense, serial_number_or_name: '333422300435', width: 640, height: 480, fps: 30}}" \
    --teleop.type=fairino_leader \
    --teleop.ip_address=192.168.59.2 \
    --teleop.gripper_enabled=true \
    --robot.gripper_force=25 \
    --teleop.gripper_force=1 \
    --teleop.id=fr5_leader \
    --display_data=true \
    --dataset.root="/home/inno-controller/workspace/SungwooVLA/FR5ControllerVLA/outputs/datasets/${JOB_NAME}" \
    --dataset.repo_id=coport-uni/${JOB_NAME} \
    --dataset.episode_time_s=120 \
    --dataset.reset_time_s=30 \
    --dataset.num_episodes=9 \
    --dataset.fps=20 \
    --dataset.single_task="transfer the gray tablets from the brown bottle into another brown bottle" \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=8 \
    --dataset.upload_large_folder=true \
    --resume=true \