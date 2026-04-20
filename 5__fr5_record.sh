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

# hand: {type: intelrealsense, serial_number_or_name: '333422300435', width: 640, height: 480, fps: 30}
# hand: {type: opencv, index_or_path: '/dev/video4', width: 640, height: 480, fps: 30}
# top_right: {type: opencv, index_or_path: '/dev/video19', width: 640, height: 480, fps: 30}, \

# Right Arrow (→): End episode and move to next
# Left Arrow (←): Cancel episode and re-record
# Escape (ESC): Stop session and upload dataset

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
    --teleop.gripper_force=0 \
    --teleop.id=fr5_leader \
    --display_data=true \
    --dataset.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=30 \
    --dataset.num_episodes=1 \
    --dataset.fps=20 \
    --dataset.single_task="pick_red_colored_marker_to_box" \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=8 \
    --resume=true
