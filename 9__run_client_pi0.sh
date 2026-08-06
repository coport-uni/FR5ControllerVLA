#!/bin/bash
# FR5 async-inference robot client (Pi0 policy variant).
#
# Connects to the PolicyServer via gRPC, performs the handshake that
# uploads the Pi0 policy/robot config to the server, then streams
# observations (joints + cameras) and executes the returned action
# chunks on the FR5 follower via ServoJ.
#
# What this script automates before launching the client (ported
# from 9__run_client_act.sh, gh #121):
#   1. Restarts the remote policy server on the B200 "appeal" box:
#      kills any running policy_server over SSH, then starts a fresh
#      one with `bash 8__run_server.sh` (binds 0.0.0.0:17044 there).
#   2. Streams the remote outputs/policy_server.log into this
#      terminal with a "[server]" prefix, so policy-side output
#      (model loading, per-chunk inference timings) is visible next
#      to the client output. The streamer dies with this script.
#   3. Restarts the local SSH tunnel mapping 127.0.0.1:17044 to the
#      server box, so a stale or dead tunnel cannot linger.
#   4. Waits for gRPC channel readiness before starting the client.
#
# Only one operator may drive the async stack at a time -- these
# restarts assume exclusive ownership of the server, tunnel, and
# streamer (see LP §G13).
#
# Pi0-specific notes (vs. 9__run_client_act.sh):
#   - actions_per_chunk=50 matches the Pi0 default chunk_size=50 /
#     n_action_steps=50 (src/lerobot/policies/pi0/configuration_pi0.py),
#     which 7__train_pi0.sh does not override.
#   - The --task string is derived from the repo name by
#     derive_task_from_repo(). Pi0 is language-conditioned (like
#     SmolVLA, unlike ACT), so the string matters at inference.
#   - Camera keys (top_left / top_right / hand) follow the dataset
#     feature names used at training time, matching 9__run_client_act.sh.
#   - The appeal box's copy of 8__run_server.sh has the Pi0 block
#     active, so the server boots with --inference_latency=1.0.
#     That value is only a floor on the GetActions duration -- it
#     pads faster calls and never delays slower ones -- so either
#     that or the ACT-tuned 0.01 works here; check the config dump
#     at the top of outputs/policy_server.log to see which is live.
#
# Prerequisite: pip install -e ".[pi]"        (Pi0 dependencies)
#               pip install -e ".[async]"     (grpcio + matplotlib extras)
# Docs:        https://huggingface.co/docs/lerobot/pi0
#              https://huggingface.co/docs/lerobot/async
# Paired with: 8__run_server.sh (the copy on the appeal box),
#              7__train_pi0_task1_b200.sh

_conda_sh=""

for _root in /home/inno-controller/anaconda3 /opt/conda \
             "$HOME/anaconda3" "$HOME/miniconda3"; do
    if [ -f "$_root/etc/profile.d/conda.sh" ]; then
        _conda_sh="$_root/etc/profile.d/conda.sh"
        break
    fi
done

if [ -z "$_conda_sh" ]; then
    echo "ERROR: no conda install found (tried inno-controller," \
         "/opt/conda, \$HOME)" >&2
    exit 1
fi

source "$_conda_sh"
conda activate lerobot

# ---------------------------------------------------------------
# Checkpoint selection. TASK is derived from the repo name below;
# switching checkpoints is a one-line change here.
# ---------------------------------------------------------------
ACTIONCHUNK=50

# Every FR5 Pi0 checkpoint on the Hub under coport-uni as of
# 2026-07-30 (hf models list --author coport-uni). Uncomment one.
#
# task1 -- move the brown glass bottle to the designated location:
# PRETRAINED="coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_50_pi0_b200_model"
# PRETRAINED="coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_100_pi0_b200_model"
PRETRAINED="coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_200_pi0_b200_model"
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

# ---------------------------------------------------------------
# Remote policy server -- kill any live instance and start fresh.
# ---------------------------------------------------------------
SSH_KEY="/home/inno-controller/workspace/SungwooVLA/FR5ControllerVLA/outputs/appeal_test_key"
SSH_DEST="appeal@59.150.32.1"
SSH_PORT=45406
REMOTE_REPO="/NHNHOME/WORKSPACE/26msit002_E/appeal_workspace/sungwoo/FR5ControllerVLA"
SERVER_PORT=17044
SERVER_ADDRESS="127.0.0.1:${SERVER_PORT}"

chmod 600 "$SSH_KEY"

ssh_opts=(-i "$SSH_KEY" -p "$SSH_PORT"
          -o StrictHostKeyChecking=no
          -o UserKnownHostsFile=/dev/null
          -o ConnectTimeout=10
          -o ServerAliveInterval=30
          -o ServerAliveCountMax=3)

# The [.] brackets keep pkill from matching this command's own
# shell on the remote side (the pattern text appears in its own
# command line). The backgrounded job must stay a SIMPLE command:
# backgrounding a compound like `cd ... && nohup ... &` makes bash
# fork a wrapper subshell that keeps the ssh stdout/stderr pipes
# and waits on the server forever, so ssh never returns. With a
# simple command bash fork+execs it directly and every fd is
# redirected to the log, letting the ssh call return immediately.
echo "Restarting the policy server on ${SSH_DEST} ..."
ssh "${ssh_opts[@]}" "$SSH_DEST" "
    pkill -f 'lerobot[.]async_inference[.]policy_server' && sleep 2
    cd '${REMOTE_REPO}' || exit 1
    nohup bash 8__run_server.sh \
        > outputs/policy_server.log 2>&1 < /dev/null &
    echo 'policy server launched (log: outputs/policy_server.log)'
" || { echo "ERROR: ssh to ${SSH_DEST} failed" >&2; exit 1; }

# ---------------------------------------------------------------
# Stream the remote server log into this terminal. The [server]
# prefix is added on the remote side so the stream stays a single
# local ssh process that the EXIT trap can kill. tail -n +1 replays
# the log from the top, so lines written while the tunnel and the
# readiness probe below run are not lost. The remote tail outlives
# the ssh channel until its next write hits the broken pipe, which
# is harmless. The bracketed pkill pattern cannot match this
# script or the pkill itself (compare LP §G12 on the restart
# block above).
# ---------------------------------------------------------------
pkill -f "tail -n [+]1 -F outputs/policy_server[.]log" && sleep 1
ssh "${ssh_opts[@]}" "$SSH_DEST" "
    cd '${REMOTE_REPO}' || exit 1
    tail -n +1 -F outputs/policy_server.log | sed -u 's/^/[server] /'
" &
server_log_stream_pid=$!
# INT/TERM re-enter through exit so the EXIT trap always runs and
# the streamer never outlives this script.
trap 'kill "$server_log_stream_pid" 2>/dev/null' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo "Streaming the remote server log (local pid" \
     "${server_log_stream_pid})"

# ---------------------------------------------------------------
# Local SSH tunnel: 127.0.0.1:17044 -> appeal box 127.0.0.1:17044.
# Kill any previous tunnel first so the forward is never stale.
#
# The tunnel is not optional: the policy server runs in the NHN
# GPU-hub container, which exposes no routable address of its own.
# The hub's HTTPS entry point (https://cl1.gpuhub.nhncloud.com:50030/
# FiF42P8A3y/) cannot serve as --server_address either -- the value
# goes straight into grpc.insecure_channel(), which parses
# "host:port" only, and gRPC carries the service/method in the
# HTTP/2 :path, leaving no room for the proxy's path prefix.
# ---------------------------------------------------------------
pkill -f "ssh.*-L ${SERVER_PORT}:127.0.0.1:${SERVER_PORT}" \
    && sleep 1
ssh -N -f -C "${ssh_opts[@]}" \
    -o ExitOnForwardFailure=yes \
    -L "${SERVER_PORT}:127.0.0.1:${SERVER_PORT}" "$SSH_DEST" \
    || { echo "ERROR: could not open the SSH tunnel" >&2; exit 1; }
echo "SSH tunnel up on ${SERVER_ADDRESS}"

# ---------------------------------------------------------------
# Wait until the server answers through the tunnel. The tunnel
# listener accepts TCP even while the remote server is still
# booting, so a plain port probe gives false positives; gRPC
# channel readiness is the real end-to-end check (grpc retries
# internally until the timeout).
# ---------------------------------------------------------------
echo "Waiting for the policy server to become ready ..."
python3 - <<PY || { echo "ERROR: server not ready in 120 s" >&2; exit 1; }
import grpc

channel = grpc.insecure_channel("${SERVER_ADDRESS}")
grpc.channel_ready_future(channel).result(timeout=120)
print("gRPC server ready on ${SERVER_ADDRESS}")
PY

# ---------------------------------------------------------------
# Robot client. This moves the FR5 -- everything above is setup.
# ---------------------------------------------------------------
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
    --policy_type=pi0 \
    --policy_device=cuda \
    --actions_per_chunk="${ACTIONCHUNK}" \
    --chunk_size_threshold=0.7 \
    --aggregate_fn_name=weighted_average \
    --debug_visualize_queue_size=false \
    --task="${TASK}" \
    --fps=20
