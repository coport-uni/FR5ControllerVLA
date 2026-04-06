#!/bin/bash
# pip install pynput
source /opt/conda/etc/profile.d/conda.sh
conda activate lerobot

lerobot-teleoperate \
    --robot.type=fairino_follower \
    --robot.ip_address=192.168.58.2 \
    --teleop.type=keyboard_fairino
