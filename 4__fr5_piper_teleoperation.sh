#!/bin/bash
# pip install pynput
source /opt/conda/etc/profile.d/conda.sh
conda activate lerobot

lerobot-teleoperate \
    --teleop.type=piper_leader \
    --teleop.port=piper_leader \
    --robot.type=fairino_follower \
    --robot.ip_address=192.168.58.2 \
    --fps=20