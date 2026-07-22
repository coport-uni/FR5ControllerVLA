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

# Remove any stale local copy so hf download starts from a clean state.
rm -rf /home/inno-controller/workspace/SungwooVLA/FR5ControllerVLA/outputs/datasets/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location

hf download \
    --repo-type dataset \
    coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location \
    --local-dir /home/inno-controller/workspace/SungwooVLA/FR5ControllerVLA/outputs/datasets/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location

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
    --teleop.gripper_force=1 \
    --teleop.id=fr5_leader \
    --display_data=true \
    --dataset.root="/home/inno-controller/workspace/SungwooVLA/FR5ControllerVLA/outputs/datasets/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location" \
    --dataset.repo_id=coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=30 \
    --dataset.num_episodes=10 \
    --dataset.fps=20 \
    --dataset.single_task="move the brown colored glass bottle to the designated location" \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=8 \
    --dataset.upload_large_folder=true \
    --resume=true \