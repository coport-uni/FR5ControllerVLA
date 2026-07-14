#!/usr/bin/env bash
# Train an ACT policy on the FR5 teleoperation dataset.
# Reference: https://huggingface.co/docs/lerobot/act
# Multi-GPU: https://huggingface.co/docs/lerobot/multi_gpu_training
# ACT ships with the base LeRobot install; no extra pip group needed.
#
# Launched via `accelerate launch` on 2x H200 NVL (141 GB / GPU).
# BATCH_SIZE here is the global (effective) batch; per-GPU batch is
# computed as $((BATCH_SIZE / GPU_NUMBER)) and passed to lerobot.
# BATCH_SIZE=496 -> per-GPU=248, tuned to ~80% peak VRAM via the
# gh issue #84 probe ladder (GPU0 78.6%, GPU1 80.8% on this
# dataset). LR is sqrt-scaled from the LeRobot ACT default lr=1e-5
# (configuration_act.py:127-129) against the baseline global=24
# used in issue #56: sqrt(496/24) ~= 4.55 -> optimizer_lr=4.5e-5,
# optimizer_lr_backbone=4.5e-5. LeRobot's multi-GPU docs leave LR
# auto-scaling to the user, and ACT's scheduler preset is None
# (no warmup), so sqrt is the safer default vs linear. Mixed
# precision bf16 is chosen for Hopper (H200).

# Try known conda install roots in order (FR5 control PC first, then
# the H200 training box, then per-user fallbacks). Fail clearly if
# none is present so we don't die later with "conda: not found".
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

# Force NCCL socket transport inside this Docker image; P2P/CUMEM
# fails silently on the first allreduce/barrier here and stalls DDP
# forever. Harmless on single-GPU runs (no torch.distributed). See
# issue #30 for the full diagnosis.

export NCCL_P2P_DISABLE=1
# export NCCL_SHM_DISABLE=1

source "$_conda_sh"
conda activate lerobot

HF_USER=$(hf auth whoami | head -n 1)
echo "HF_USER=${HF_USER}"

# JOB_NAME="FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise_50"
# JOB_NAME="FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise_100"
# JOB_NAME="FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise_200"

# JOB_NAME="FR5_tas3_move_the_brown_colored_glass_bottle_to_the_designated_location_100"
# JOB_NAME="FR5_task3_move_the_brown_colored_glass_bottle_to_the_designated_location_200"

JOB_NAME="FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle_into_another_brown_bottle_50"

# LeRobot resolves the dataset revision from a git tag matching the
# codebase_version in info.json (get_safe_version in utils.py). An
# untagged repo aborts with a FileNotFoundError on meta/info.json --
# the underlying RevisionNotFoundError is masked by a huggingface_hub
# API change. Tag the dataset here if the tag is missing. See issue
# #92.
DATASET_REPO="coport-uni/${JOB_NAME}"
python - "$DATASET_REPO" <<'PY'
import json
import sys

from huggingface_hub import HfApi

repo = sys.argv[1]
api = HfApi()
info_path = api.hf_hub_download(repo, "meta/info.json", repo_type="dataset")
with open(info_path) as f:
    version = json.load(f)["codebase_version"]
tags = {t.name for t in api.list_repo_refs(repo, repo_type="dataset").tags}
if version in tags:
    print(f"dataset tag {version} already present")
else:
    api.create_tag(repo, tag=version, repo_type="dataset")
    print(f"created dataset tag {version}")
PY

BATCH_SIZE=496
GPU_NUMBER=2

STEPS_NUMBER=100000
CHECKPOINT_NUMBER=10

accelerate launch \
    --multi_gpu \
    --num_processes=${GPU_NUMBER} \
    --mixed_precision=bf16 \
    "$(which lerobot-train)" \
    --dataset.repo_id=coport-uni/${JOB_NAME} \
    --dataset.video_backend=pyav \
    --policy.type=act \
    --policy.repo_id=coport-uni/${JOB_NAME}_act_h200_model \
    --policy.push_to_hub=true \
    --policy.device=cuda \
    --output_dir=outputs/train/${JOB_NAME}_act_h200 \
    --job_name=${JOB_NAME}_act_h200 \
    --wandb.enable=true \
    --batch_size=$((BATCH_SIZE / GPU_NUMBER)) \
    --steps=${STEPS_NUMBER} \
    --save_freq=$((STEPS_NUMBER / CHECKPOINT_NUMBER)) \
    --resume=false \
    --seed=55 \
    --tolerance_s=0.1 \
    --policy.optimizer_lr=4.5e-5 \
    --policy.optimizer_lr_backbone=4.5e-5 \
    --num_workers=4