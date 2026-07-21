#!/usr/bin/env bash
# Train a Pi0 policy on the FR5 teleop dataset, tuned for 4 x B200.
# Adapted from 7__train_pi0_adv.sh (2 x H200) following the
# global-batch / GPU_NUMBER convention of 7__train_act_task1_h200.sh.
#
# openpi production fine-tune defaults (openpi
# src/openpi/training/config.py) used as the baseline:
#   - num_train_steps = 30_000
#   - batch_size      = 32   (effective)
#   - optimizer       = AdamW lr=2.5e-5 betas=(0.9, 0.95) wd~=0
#   - scheduler       = warmup-cosine, warmup=1_000, decay=30_000
#   - dtype           = bfloat16
#   - full fine-tune (vision encoder NOT frozen, expert NOT only)
#
# ---------------------------------------------------------------------
# Why these numbers on 4 x B200 -- MEASURED on this box (issue #87,
# claude_test/probe_logs/SUMMARY_b200.md)
# ---------------------------------------------------------------------
# IMPORTANT: the GPU count is NOT fixed -- verify it every run with
#   `nvidia-smi -L`  (or `nvidia-smi --query-gpu=index --format=csv`)
# before launching, then set GPU_NUMBER to match. This box was probed at
# 4 x B200, but the visible device count changes between sessions/boxes;
# the per-GPU figures below assume the per-GPU batch, NOT a fixed GPU
# count. Keep BATCH_SIZE / GPU_NUMBER at the validated per-GPU rung (176)
# rather than holding BATCH_SIZE constant when GPU_NUMBER changes.
#
# Per-GPU batch = 176 (BATCH_SIZE 704):
#   B200 VRAM ladder (4 x B200, 183359 MiB/GPU, bf16 + compile +
#   gradient_checkpointing, full fine-tune): batch=176 completes
#   50/50 steps at peak 135458 MiB = 73.9 % VRAM, 6.29 s/step;
#   batch=192 OOMs on the first forward/backward. 176 is the largest
#   validated rung, same accept-below-OOM rule as the H200 probe
#   (issue #55, batch=160 there).
#
# Learning rate = 1.2e-4:
#   Effective batch = 176 * 4 = 704 = 22x openpi's baseline 32.
#   SQRT scaling: lr = 2.5e-5 * sqrt(22) = 1.17e-4 -> 1.2e-4.
#   SQRT (not linear) per the probe docs; LeRobot does no LR
#   auto-scaling. scheduler_warmup_steps stays at the LeRobot default
#   1000 (warmup length is not proportional to batch size). If the
#   model underfits, raise lr toward the linear bound (5.5e-4) or
#   extend --steps.
#
# compile_mode=default (NOT pi0's max-autotune default):
#   pi0 compiles with mode="max-autotune" by default
#   (configuration_pi0.py), whose Triton GEMM autotune crashes with
#   CUDA illegal-memory-access on this sm_100 box at per-GPU
#   batch >= ~176 (torch 2.10.0+cu128). mode=default keeps
#   torch.compile (GEMMs via cuBLAS) and trains cleanly.
#
# Precision:
#   bf16 on Blackwell, same as Hopper. (B200 also exposes fp8/fp4, but
#   LeRobot pi0 trains in bf16; leave mixed_precision=bf16.)
#
# Checkpoint-loading notes for this env (both handled; SUMMARY_b200.md):
#   1. lerobot/pi0_base HEAD ships a processor step
#      (relative_actions_processor) unknown to this v0.5.1 fork ->
#      hard ImportError. pretrained_path below therefore points at the
#      pinned local snapshot models/pi0_base_v051compat (revision
#      26b99b9439ac; identical weights).
#   2. transformers 5.11.0 renamed PaliGemma vision-tower keys
#      (vision_tower.vision_model.* -> vision_tower.*), which used to
#      leave the vision tower randomly initialized. FIXED (gh #88) via
#      a conditional key remap in
#      PI0Policy._fix_pytorch_state_dict_keys; verified 437/437
#      vision-tower tensors load (claude_test/verify_pi0_vision_load.py).
#
# Note: openpi's EMA (ema_decay=0.99) is NOT implemented in LeRobot pi0,
#       so this script does not enable it. Optimizer weight_decay is left
#       at the LeRobot default 0.01 (openpi uses ~1e-10); add
#       --policy.optimizer_weight_decay=1e-10 if you need exact parity.
#
# Reference: https://github.com/Physical-Intelligence/openpi
# Requires: pip install -e ".[pi]"

# Try known conda install roots in order (FR5 control PC first, then
# the training box, then per-user fallbacks). Fail clearly if none is
# present so we don't die later with "conda: not found".

# source /NHNHOME/workspace/sungwoo/miniforge3/etc/profile.d/conda.sh

_conda_sh=""
for _root in /NHNHOME/workspace/sungwoo/miniforge3 \
             /home/inno-controller/anaconda3 /opt/conda \
             "$HOME/anaconda3" "$HOME/miniconda3"; do
    if [ -f "$_root/etc/profile.d/conda.sh" ]; then
        _conda_sh="$_root/etc/profile.d/conda.sh"
        _conda_root="$_root"
        break
    fi
done

if [ -z "$_conda_sh" ]; then
    echo "ERROR: no conda install found (tried inno-controller, /opt/conda, \$HOME)" >&2
    exit 1
fi

source "$_conda_sh"
conda activate lerobot
# On this box `conda activate` leaves bare `python` shadowed by
# /usr/bin/python; force the env bin to the front of PATH.
export PATH="${_conda_root}/envs/lerobot/bin:${PATH}"

# /tmp is mounted noexec on the B200 box, so torch.compile/triton cannot
# dlopen the kernels it writes to /tmp ("failed to map segment from
# shared object" -> crash at step 0). Redirect both caches to an
# exec-allowed dir next to this script. Required because
# compile_model=true below. See issue #87.
_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export TORCHINDUCTOR_CACHE_DIR="${_repo_root}/.cache/torchinductor"
export TRITON_CACHE_DIR="${_repo_root}/.cache/triton"
mkdir -p "${TORCHINDUCTOR_CACHE_DIR}" "${TRITON_CACHE_DIR}"

# The HF cache defaults to ~/.cache, which lives on the container
# overlay and is destroyed by a container swap -- the 2026-07-14
# migration wiped it and training then failed on the gated
# google/paligemma-3b-pt-224 tokenizer that pi0/pi05 fetch at startup.
# Keep the cache (and the auth token) on the persistent disk instead.
export HF_HOME=/NHNHOME/workspace/sungwoo/hf_cache
mkdir -p "${HF_HOME}"

# Force NCCL socket transport inside this Docker image; P2P/CUMEM fails
# silently on the first allreduce/barrier here and stalls DDP forever.
# See issue #30 for the full diagnosis. On a properly configured B200
# NVSwitch node (NVLink 5) this throttles allreduce bandwidth -- try
# removing it there and confirm the first barrier still completes.
# export NCCL_P2P_DISABLE=1
# export NCCL_SHM_DISABLE=1

HF_USER=$(hf auth whoami | head -n 1)
echo "HF_USER=${HF_USER}"

# JOB_NAME="FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise_50"
# JOB_NAME="FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise_100"
# JOB_NAME="FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise_200"

# JOB_NAME="FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle_into_another_brown_bottle_50"
JOB_NAME="FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle_into_another_brown_bottle_100"

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

# JOB_NAME="FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_100"
# JOB_NAME="FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_200"

# Verify against `nvidia-smi -L` before each run; see header note.
GPU_NUMBER=3

# Global (effective) batch; per-GPU = BATCH_SIZE / GPU_NUMBER = 176,
# measured at 73.9 % peak VRAM on this box (192/GPU OOMs). See header.
BATCH_SIZE=528

STEPS_NUMBER=30000
CHECKPOINT_NUMBER=10

accelerate launch \
    --multi_gpu \
    --num_processes=${GPU_NUMBER} \
    --mixed_precision=bf16 \
    "$(which lerobot-train)" \
    --dataset.repo_id=coport-uni/${JOB_NAME} \
    --policy.type=pi0 \
    --policy.pretrained_path=models/pi0_base_v051compat \
    --policy.repo_id=coport-uni/${JOB_NAME}_pi0_b200 \
    --policy.push_to_hub=true \
    --dataset.video_backend=pyav \
    --policy.device=cuda \
    --policy.compile_model=true \
    --policy.compile_mode=default \
    --policy.gradient_checkpointing=true \
    --policy.dtype=bfloat16 \
    --policy.freeze_vision_encoder=false \
    --policy.train_expert_only=false \
    --output_dir=outputs/train/${JOB_NAME}_pi0_b200 \
    --job_name=${JOB_NAME}_pi0_b200 \
    --wandb.enable=false \
    --batch_size=$((BATCH_SIZE / GPU_NUMBER)) \
    --policy.optimizer_lr=1.04e-4 \
    --steps=${STEPS_NUMBER} \
    --save_freq=$((STEPS_NUMBER / CHECKPOINT_NUMBER)) \
    --num_workers=10 \
    --seed=55 \
    --tolerance_s=0.1 \
    --resume=false