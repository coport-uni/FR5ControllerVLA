#!/bin/bash
# FR5-to-FR5 leader-follower teleoperation.
# Leader (192.168.59.2): drag-teach mode -- move by hand.
# Follower (192.168.58.2): mirrors leader via ServoJ.
source /opt/conda/etc/profile.d/conda.sh
conda activate lerobot

lerobot-teleoperate \
    --robot.type=fairino_follower \
    --robot.ip_address=192.168.58.2 \
    --teleop.type=fairino_leader \
    --teleop.ip_address=192.168.59.2 \
    --display_data=false
