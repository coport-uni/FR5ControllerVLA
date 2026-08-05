#!/bin/bash
# FR5 async-inference robot client (Pi0.5 policy variant).
#
# Connects to the PolicyServer (8__run_server.sh) via gRPC, performs the
# handshake that uploads the Pi0.5 policy/robot config to the server,
# then streams observations (joints + cameras) and executes the returned
# action chunks on the FR5 follower via ServoJ.
#
# Settings below are derived from 7__train_pi05_task2_b200.sh and were
# cross-checked against the trained checkpoint's config.json:
#   - input_features: observation.state (7) + observation.images.
#     {top_left, top_right, hand}, each 3x480x640. The camera keys here
#     must match those names exactly -- the server builds its
#     rename_map from the client's robot features and the checkpoint
#     preprocessor ships an empty rename_map, so a mismatched key is a
#     silently missing input.
#   - chunk_size = n_action_steps = 50, hence actions_per_chunk=50
#     (configuration_pi05.py defaults; the training script does not
#     override them).
#   - Pi0.5 is language-conditioned: the tokenizer step reads
#     observation["task"], so --task must repeat the dataset's
#     single_task string verbatim (5__fr5_record_task2.sh). It is
#     derived from the checkpoint repo name below, which encodes
#     that same string, so switching checkpoints switches the
#     instruction with it.
#   - Dataset fps is 20, so client fps = server fps = 20.
#
# Server-side prerequisites (the server loads the checkpoint, not this
# client):
#   - The path/repo in PRETRAINED must resolve ON THE SERVER BOX.
#   - Pi0.5 fetches the gated google/paligemma-3b-pt-224 tokenizer at
#     load time, so the server needs an authenticated HF cache.
#
# Prerequisite: pip install -e ".[pi]"        (Pi0.5 dependencies)
#               pip install -e ".[async]"     (grpcio + matplotlib)
# Docs:        https://huggingface.co/docs/lerobot/pi05
#              https://huggingface.co/docs/lerobot/async
# Paired with: 8__run_server.sh, 7__train_pi05_task2_b200.sh

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

# The policy server runs in the NHN GPU-hub container, which exposes no
# routable address of its own -- open an SSH tunnel from this machine
# first, then dial the forwarded local port:
#
#   ssh -N -C -i ~/.ssh/appeal_test_key -p 45406 \
#       -o ServerAliveInterval=30 -o ServerAliveCountMax=3 \
#       -L 17040:127.0.0.1:17040 appeal@59.150.32.1
#
# The hub's HTTPS entry point (https://cl1.gpuhub.nhncloud.com:50030/
# FiF42P8A3y/) cannot be used here: this value goes straight into
# grpc.insecure_channel(), which parses "host:port" only, and gRPC puts
# the service/method in the HTTP/2 :path, so the proxy's path prefix has
# nowhere to live.
#
# The port must also match the --port that 8__run_server.sh binds.
SERVER_ADDRESS="127.0.0.1:17040"

# ---------------------------------------------------------------
# Checkpoint selection. TASK is derived from the repo name below;
# switching checkpoints is a one-line change here.
# ---------------------------------------------------------------
ACTIONCHUNK=50

# Every FR5 Pi0.5 checkpoint on the Hub under coport-uni as of
# 2026-08-05 (hf models list --author coport-uni --limit 500 -- the
# default page size is 30 and truncates this listing). Uncomment one.
#
# task1 -- move the brown glass bottle to the designated location:
# PRETRAINED="coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_50_pi05_b200_model"
# PRETRAINED="coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_100_pi05_b200_model"
# PRETRAINED="coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_200_pi05_b200_model"
#
# task2 -- transfer the gray tablets between brown bottles (no
# 100-episode Pi0.5 variant on the Hub). The 200-episode repo ends in
# "_pi5_b200", NOT "_pi05_b200" as in the training script:
# "<JOB_NAME>_pi05_b200" is 97 chars and the Hub caps repo names at 96
# (LearnedPatterns Q18), so the run's own --policy.push_to_hub failed
# silently and the checkpoint was uploaded by hand under the
# one-char-shorter name, which lands on exactly 96:
# PRETRAINED="coport-uni/FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle_into_another_brown_bottle_50_pi05_b200"
PRETRAINED="coport-uni/FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle_into_another_brown_bottle_200_pi5_b200"
#
# task3 -- turn the silver air valve 90 degrees counterclockwise:
# PRETRAINED="coport-uni/FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise_50_pi05_b200_model"
# PRETRAINED="coport-uni/FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise_100_pi05_b200_model"
# PRETRAINED="coport-uni/FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise_200_pi05_b200_model"
#
# Legacy red-marker checkpoints (single-camera era, kept for
# reference -- they predate the top_left/top_right/hand layout below):
# PRETRAINED="coport-uni/FR5_pick_red_colored_marker_to_box_pi05_model_model"
# PRETRAINED="coport-uni/FR5_pick_red_colored_marker_to_box_pi05_adv_model"
#
# Local fallback. This path must exist on the SERVER box, which is
# where the policy is actually loaded -- not on the machine running
# this client. TASK cannot be derived from a path like this one, so
# use the manual override below with it.
# PRETRAINED="outputs/train/FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle_into_another_brown_bottle_200_pi05_b200/checkpoints/last/pretrained_model"

derive_task_from_repo() {
    # Turn a checkpoint repo id into the dataset's natural-language
    # task string: drop the namespace, the FR5_/taskN_ prefix and
    # the _<episodes>_<policy>_<hardware>/_model suffix, then map
    # underscores to spaces. Handles all naming generations:
    #   FR5_task2_transfer_..._brown_bottle_200_pi5_b200
    #   FR5_task1_move_..._designated_location_50_pi05_b200_model
    #   FR5_pick_red_colored_marker_to_box_pi05_adv_model
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

python3 -m lerobot.async_inference.robot_client \
    --server_address="${SERVER_ADDRESS}" \
    --robot.type=fairino_follower \
    --robot.ip_address=192.168.58.2 \
    --robot.gripper_enabled=true \
    --robot.id=fr5_follower \
    --robot.cameras="{ \
        top_left: {type: opencv, index_or_path: '/dev/video18', width: 640, height: 480, fps: 20}, \
        top_right: {type: opencv, index_or_path: '/dev/video19', width: 640, height: 480, fps: 20}, \
        hand: {type: intelrealsense, serial_number_or_name: '333422300435', width: 640, height: 480, fps: 30}}" \
    --pretrained_name_or_path="${PRETRAINED}" \
    --policy_type=pi05 \
    --policy_device=cuda \
    --actions_per_chunk="${ACTIONCHUNK}" \
    --chunk_size_threshold=0.6 \
    --aggregate_fn_name=weighted_average \
    --debug_visualize_queue_size=false \
    --task="${TASK}" \
    --fps=20

# Tuning notes (docs/source/async.mdx):
#   - chunk_size_threshold 0.5-0.6 is the documented sweet spot. At
#     0.6 the client re-requests once 30 of the 50 actions remain,
#     i.e. it gives the server ~1.5 s to answer at 20 fps. If the
#     queue still drains (watch it with
#     --debug_visualize_queue_size=true), lower fps before lowering
#     the threshold: dropping to 0.4 would shrink the inference
#     budget to ~1.0 s rather than growing it. 9__run_client_pi0.sh
#     goes the other way, to 0.7, for Pi0's slower forward pass.
#   - aggregate_fn_name=weighted_average (0.3*old + 0.7*new) is the
#     LeRobot default for overlapping chunk regions.
