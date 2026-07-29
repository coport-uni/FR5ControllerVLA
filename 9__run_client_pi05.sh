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
#     single_task string verbatim (5__fr5_record_task2.sh).
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

# Must match the --port that 8__run_server.sh binds. The two are easy to
# desync: the server currently defaults its active block to 17040.
SERVER_ADDRESS="10.0.12.139:17044"

# Note the "_pi5_b200" suffix, NOT "_pi05_b200" as in the training
# script: "<JOB_NAME>_pi05_b200" is 97 chars and the Hub caps repo names
# at 96 (LearnedPatterns Q18), so the run's own --policy.push_to_hub
# failed silently. The checkpoint was uploaded by hand under the
# one-char-shorter name, which lands on exactly 96.
PRETRAINED="coport-uni/FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle_into_another_brown_bottle_200_pi5_b200"

# Local fallback. This path must exist on the SERVER box, which is where
# the policy is actually loaded -- not on the machine running this
# client.
# PRETRAINED="outputs/train/FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle_into_another_brown_bottle_200_pi05_b200/checkpoints/last/pretrained_model"

# Shorter-dataset variants of the same task (both fit the 96-char cap):
# PRETRAINED="coport-uni/FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle_into_another_brown_bottle_50_pi05_b200"

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
    --actions_per_chunk=50 \
    --chunk_size_threshold=0.6 \
    --aggregate_fn_name=weighted_average \
    --debug_visualize_queue_size=false \
    --task="transfer the gray tablets from the brown bottle into another brown bottle" \
    --fps=20

# Tuning notes (docs/source/async.mdx):
#   - chunk_size_threshold 0.5-0.6 is the documented sweet spot. At
#     0.6 the client re-requests once 30 of the 50 actions remain,
#     i.e. it gives the server ~1.5 s to answer at 20 fps. If the
#     queue still drains (watch it with
#     --debug_visualize_queue_size=true), lower fps before lowering
#     the threshold -- 0.4, as in 9__run_client_pi0.sh, shrinks the
#     inference budget to ~1.0 s rather than growing it.
#   - aggregate_fn_name=weighted_average (0.3*old + 0.7*new) is the
#     LeRobot default for overlapping chunk regions.
