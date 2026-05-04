#!/bin/bash
# FR5 async-inference robot client.
#
# Connects to the PolicyServer (8__run_server.sh, default 127.0.0.1:8080)
# via gRPC, performs the handshake that uploads the policy/robot config
# to the server, then streams observations (joints + cameras) and
# executes the returned action chunks on the FR5 follower via ServoJ.
#
# Prerequisite: pip install -e ".[async]"   (grpcio + matplotlib extras)
# Docs:        https://huggingface.co/docs/lerobot/async
# NOTE:        Set PRETRAINED to a real FR5 policy checkpoint
#              (HuggingFace repo id or local path) before running.
# Paired with: 8__run_server.sh

_conda_sh=""

for _root in /home/inno-controller/anaconda3 /opt/conda \
             "$HOME/anaconda3" "$HOME/miniconda3"; do
    if [ -f "$_root/etc/profile.d/conda.sh" ]; then
        _conda_sh="$_root/etc/profile.d/conda.sh"
        break
    fi
done

if [ -z "$_conda_sh" ]; then
    echo "ERROR: no conda install found (tried inno-controller, /opt/conda, \$HOME)" >&2
    exit 1
fi

source "$_conda_sh"
conda activate lerobot

python3 -m lerobot.async_inference.robot_client \
    --server_address=10.0.12.139:17044 \
    --robot.type=fairino_follower \
    --robot.ip_address=192.168.58.2 \
    --robot.gripper_enabled=true \
    --robot.id=fr5_follower \
    --robot.cameras="{ \
        top_left: {type: opencv, index_or_path: '/dev/video18', width: 640, height: 480, fps: 20}, \
        top_right: {type: opencv, index_or_path: '/dev/video19', width: 640, height: 480, fps: 20}, \
        hand: {type: intelrealsense, serial_number_or_name: '333422300435', width: 640, height: 480, fps: 30}}" \
    --pretrained_name_or_path=coport-uni/FR5_pick_red_colored_marker_to_box_smolvla_model \
    --policy_type=smolvla \
    --policy_device=cuda \
    --actions_per_chunk=50 \
    --chunk_size_threshold=0.8 \
    --aggregate_fn_name=average \
    --debug_visualize_queue_size=false \
    --task="pick_red_colored_marker_to_box" \
    --fps=20
