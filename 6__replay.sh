#!/bin/bash
# Replay a recorded episode on the FR5 follower arm.
# Loads an episode from the local dataset and streams its actions to the
# fairino_follower via ServoJ at the dataset's recorded fps.
source /home/inno-controller/anaconda3/etc/profile.d/conda.sh
conda activate lerobot

lerobot-replay \
    --robot.type=fairino_follower \
    --robot.ip_address=192.168.58.2 \
    --robot.gripper_enabled=true \
    --robot.id=fr5_follower \
    --dataset.root="/home/inno-controller/workspace/SungwooVLA/FR5ControllerVLA/outputs/datasets/FR5_pick_red_colored_marker_to_box" \
    --dataset.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box \
    --dataset.episode=0 \
    --dataset.fps=20
