#!/bin/bash
# FR5 async-inference robot client (Pi0 policy variant).
#
# Connects to the PolicyServer (8__run_server.sh, default 127.0.0.1:8080)
# via gRPC, performs the handshake that uploads the Pi0 policy/robot
# config to the server, then streams observations (joints + cameras)
# and executes the returned action chunks on the FR5 follower via
# ServoJ.
#
# Pi0-specific notes (vs. 9__run_client_act.sh / 9__run_client_smovla.sh):
#   - actions_per_chunk=50 matches the Pi0 default chunk_size=50 /
#     n_action_steps=50 (src/lerobot/policies/pi0/configuration_pi0.py),
#     which 7__train_pi0.sh does not override.
#   - The --task string is derived from the repo name by
#     derive_task_from_repo(). Pi0 is language-conditioned (like
#     SmolVLA, unlike ACT), so the string matters at inference.
#   - Camera keys (top_left / top_right / hand) follow the dataset
#     feature names used at training time, matching 9__run_client_act.sh.
#
# Prerequisite: pip install -e ".[pi]"        (Pi0 dependencies)
#               pip install -e ".[async]"     (grpcio + matplotlib extras)
# Docs:        https://huggingface.co/docs/lerobot/pi0
#              https://huggingface.co/docs/lerobot/async
# NOTE:        Set PRETRAINED to a real FR5 Pi0 checkpoint
#              (HuggingFace repo id or local path) before running.
# Paired with: 8__run_server.sh, 7__train_pi0.sh

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

# ---------------------------------------------------------------
# Checkpoint selection. TASK is derived from the repo name below;
# switching checkpoints is a one-line change here.
# ---------------------------------------------------------------

# Every FR5 Pi0 checkpoint on the Hub under coport-uni as of
# 2026-07-30 (hf models list --author coport-uni). Uncomment one.
#
# task1 -- move the brown glass bottle to the designated location:
# PRETRAINED="coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_50_pi0_b200_model"
# PRETRAINED="coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_100_pi0_b200_model"
# PRETRAINED="coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_200_pi0_b200_model"
#
# task2 -- transfer the gray tablets between brown bottles:
# PRETRAINED="coport-uni/FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle_into_another_brown_bottle_50_pi0_b200"
# PRETRAINED="coport-uni/FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle_into_another_brown_bottle_100_pi0_b200"
# PRETRAINED="coport-uni/FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle_into_another_brown_bottle_200_pi0_b200"
#
# task3 -- turn the silver air valve 90 degrees counterclockwise:
# PRETRAINED="coport-uni/FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise_50_pi0_b200_model"
# PRETRAINED="coport-uni/FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise_100_pi0_b200_model"
# PRETRAINED="coport-uni/FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise_200_pi0_b200_model"
#
# Earlier red-marker checkpoints (no taskN prefix, no episode
# count):
# PRETRAINED="coport-uni/FR5_pick_red_colored_marker_to_box_pi0_model_paper"
PRETRAINED="coport-uni/FR5_pick_red_colored_marker_to_box_pi0_adv_model"

derive_task_from_repo() {
    # Turn a checkpoint repo id into the dataset's natural-language
    # task string: drop the namespace, the FR5_/taskN_ prefix and
    # the _<episodes>_<policy>_<hardware>/_model suffix, then map
    # underscores to spaces. Handles all naming generations:
    #   FR5_task2_transfer_..._brown_bottle_50_pi0_b200
    #   FR5_task1_move_..._designated_location_50_model
    #   FR5_pick_red_colored_marker_to_box_pi0_adv_model
    local name="${1##*/}"
    name="${name#FR5_}"
    name="$(sed -E -e 's/^task[0-9]+_//' \
                   -e 's/_(act|pi05|pi5|pi0|smovla22|smovla)_.*$//' \
                   -e 's/_model$//' \
                   -e 's/_[0-9]+$//' <<< "$name")"
    printf '%s\n' "${name//_/ }"
}

TASK="$(derive_task_from_repo "$PRETRAINED")"

# TASK="manual override"      # uncomment to bypass auto-derivation

echo "PRETRAINED=${PRETRAINED}"
echo "TASK=${TASK}"

# The policy server runs in the NHN GPU-hub container, reachable only
# through an SSH tunnel opened beforehand from this machine:
#
#   ssh -N -C -i ~/.ssh/appeal_test_key -p 45406 \
#       -o ServerAliveInterval=30 -o ServerAliveCountMax=3 \
#       -L 17040:127.0.0.1:17040 appeal@59.150.32.1
#
# The hub's HTTPS entry point (https://cl1.gpuhub.nhncloud.com:50030/
# FiF42P8A3y/) cannot serve as --server_address: the value goes straight
# into grpc.insecure_channel(), which parses "host:port" only, and gRPC
# carries the service/method in the HTTP/2 :path, leaving no room for the
# proxy's path prefix.

python3 -m lerobot.async_inference.robot_client \
    --server_address=127.0.0.1:17044 \
    --robot.type=fairino_follower \
    --robot.ip_address=192.168.58.2 \
    --robot.gripper_enabled=true \
    --robot.id=fr5_follower \
    --robot.cameras="{ \
        top_left: {type: opencv, index_or_path: '/dev/video18', width: 640, height: 480, fps: 20}, \
        top_right: {type: opencv, index_or_path: '/dev/video19', width: 640, height: 480, fps: 20}, \
        hand: {type: intelrealsense, serial_number_or_name: '333422300435', width: 640, height: 480, fps: 30}}" \
    --pretrained_name_or_path="${PRETRAINED}" \
    --policy_type=pi0 \
    --policy_device=cuda \
    --actions_per_chunk=50 \
    --chunk_size_threshold=0.7 \
    --aggregate_fn_name=weighted_average \
    --debug_visualize_queue_size=false \
    --task="${TASK}" \
    --fps=20
