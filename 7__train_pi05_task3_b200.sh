#!/usr/bin/env bash
# Train a Pi0.5 "adv" policy on the FR5 teleop dataset, tuned for 4 x B200.
# Adapted from 7__train_pi05_adv_h200.sh (pi05 policy + MEAN_STD norm +
# weight_decay=1e-10 + red-marker dataset) using the 4 x B200 environment
# conventions of 7__train_pi0_task1_b200.sh.
# Reference: https://huggingface.co/docs/lerobot/pi05
#            https://arxiv.org/abs/2504.16054 (original Pi0.5 paper)
#            https://github.com/Physical-Intelligence/openpi
# Requires: pip install -e ".[pi]"
#
# ---------------------------------------------------------------------
# Why these numbers on 4 x B200 -- MEASURED on this box (issue #89,
# claude_test/probe_logs/SUMMARY_pi05_b200.md)
# ---------------------------------------------------------------------
# Per-GPU batch = 152 (BATCH_SIZE 608):
#   pi05 VRAM/compile ladder (4 x B200, 183359 MiB/GPU, bf16 + compile
#   mode=default + gradient_checkpointing, full fine-tune): batch=152
#   completes 50/50 steps at peak 131024 MiB = 71.5 % VRAM, 5.30 s/step;
#   batch=160 and above FAIL -- but NOT on VRAM (152 leaves ~28 %
#   headroom). pi05's gemma RMSNorm compiles to a Triton persistent-
#   reduction kernel whose per-block shared memory (294976 B at batch
#   168) exceeds the B200 per-SM limit (232448 B) once per-GPU batch
#   crosses ~160 ("No valid triton configs. OutOfMemoryError: out of
#   resource"). 152 is the largest validated rung, same accept-below-
#   failure rule as the pi0 B200 probe (issue #87, batch 176 there).
#
# Learning rate = 1.1e-4:
#   Effective batch = 152 * 4 = 608 = 19x openpi's baseline 32.
#   SQRT scaling: lr = 2.5e-5 * sqrt(19) = 1.09e-4 -> 1.1e-4. SQRT (not
#   linear) per the probe docs; LeRobot does no LR auto-scaling.
#   scheduler_warmup_steps stays at the LeRobot/openpi default 1000
#   (warmup length is not proportional to batch size). If the model
#   underfits, raise lr toward the linear bound (4.75e-4) or extend
#   --steps.
#
# --steps=1570 holds the openpi sample budget at this larger batch:
#   1570 * 608 = 954,560 samples ~= 8.00 epoch on the 119,356-frame
#   FR5_pick_red_colored_marker_to_box dataset (vs openpi reference
#   30000 * 32 = 960 k samples ~= 8.04 epoch, and the H200 adv run's
#   3500 * 272 = 952 k).
#
# Hyperparameters here track openpi's reference defaults (AdamW
# betas=(0.9,0.95) eps=1e-8 wd=1e-10 clip_grad=1.0; warmup-cosine
# warmup=1000). Two paper/openpi features LeRobot's pi05 cannot
# reproduce (carried over from the H200 adv script):
#   - EMA (ema_decay=0.99): not implemented in LeRobot pi0/pi05.
#   - Per-dataset 1%/99% quantile action normalization: replaced by the
#     MEAN_STD override below (the HF-doc-sanctioned escape hatch,
#     docs/source/pi05.mdx). The proper alternative is to pre-augment
#     the dataset with quantile stats and push to Hub; we skip that.
#
# ---------------------------------------------------------------------
# B200-specific environment fixes (all four are required here; see
# SUMMARY_pi05_b200.md / SUMMARY_b200.md)
# ---------------------------------------------------------------------
#   1. pretrained_path points at the pinned local snapshot
#      models/pi05_base_v051compat, NOT lerobot/pi05_base. The HEAD
#      checkpoint ships processor steps unknown to this v0.5.1 fork --
#      relative_actions_processor (preprocessor) and
#      absolute_actions_processor (postprocessor), both enabled:false
#      no-ops -- which raise ImportError before training starts. The
#      snapshot drops just those two disabled steps; weights are
#      byte-identical. (pi0 hit the same blocker, gh #87/#88.)
#   2. compile_mode=default. pi05 compiles with mode="max-autotune" by
#      default, whose Triton GEMM autotune crashes with CUDA illegal-
#      memory-access on this sm_100 box (torch 2.10.0+cu128). default
#      keeps torch.compile (GEMMs via cuBLAS) and trains cleanly.
#   3. NCCL P2P stays ENABLED. The inherited NCCL_P2P_DISABLE=1 (issue
#      #30, a Docker-image workaround) HANGS the first allreduce on this
#      bare-metal B200 NVLink box; we force it off.
#   4. /tmp is mounted noexec, so torch.compile/triton cannot dlopen the
#      kernels it writes there. Both caches are redirected to the repo
#      .cache/ below.

# Try known conda install roots in order (miniforge3 on this B200 box
# first, then the FR5/H200 roots, then per-user fallbacks). Fail clearly
# if none is present so we don't die later with "conda: not found".
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
    echo "ERROR: no conda install found (tried miniforge3, inno-controller, /opt/conda, \$HOME)" >&2
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

# NCCL: P2P must stay ENABLED on this bare-metal B200 NVLink/NVSwitch
# box. The inherited NCCL_P2P_DISABLE=1 (issue #30, a Docker-image
# workaround) HANGS the first allreduce here. See issue #87.
export NCCL_P2P_DISABLE=0

HF_USER=$(hf auth whoami | head -n 1)
echo "HF_USER=${HF_USER}"

# wandb logging is enabled below (--wandb.enable=true). Verify the env is
# authenticated up front so the run fails fast here instead of stalling
# on an interactive login prompt mid-training. The viewer query also
# validates the stored API key against the server. Uses the active
# lerobot env's python (PATH was forced to its bin above).
WANDB_USER=$(python -c "import wandb; print(wandb.Api().viewer.username)" 2>/dev/null)
if [ -z "${WANDB_USER}" ]; then
    echo "ERROR: wandb is not logged in (no valid API key)." >&2
    echo "       Run 'wandb login' or export WANDB_API_KEY before training." >&2
    exit 1
fi
echo "WANDB_USER=${WANDB_USER}"

# JOB_NAME="FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_50"
# JOB_NAME="FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_100"
# JOB_NAME="FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_200"
# JOB_NAME="FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_50"
JOB_NAME="FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise"

GPU_NUMBER=6

# Global (effective) batch; per-GPU = BATCH_SIZE / GPU_NUMBER = 152,
# the largest rung that compiles (160 hits the Triton shared-memory
# limit) at 71.5 % peak VRAM. See header.
BATCH_SIZE=608

# 1570 * 608 = 954,560 samples ~= 8.00 epoch on the 119,356-frame
# dataset (holds the openpi sample budget). CHECKPOINT_NUMBER=10 ->
# save every 157 steps.
STEPS_NUMBER=30000
CHECKPOINT_NUMBER=10

accelerate launch \
    --multi_gpu \
    --num_processes=${GPU_NUMBER} \
    --mixed_precision=bf16 \
    "$(which lerobot-train)" \
    --dataset.repo_id=coport-uni/${JOB_NAME} \
    --policy.type=pi05 \
    --policy.pretrained_path=models/pi05_base_v051compat \
    --policy.normalization_mapping='{"ACTION": "MEAN_STD", "STATE": "MEAN_STD", "VISUAL": "IDENTITY"}' \
    --dataset.video_backend=pyav \
    --policy.repo_id=coport-uni/${JOB_NAME}_pi05_b200_model \
    --policy.push_to_hub=true \
    --policy.device=cuda \
    --policy.compile_model=true \
    --policy.compile_mode=default \
    --policy.gradient_checkpointing=true \
    --policy.dtype=bfloat16 \
    --policy.freeze_vision_encoder=false \
    --policy.train_expert_only=false \
    --policy.optimizer_weight_decay=1e-10 \
    --output_dir=outputs/train/${JOB_NAME}_pi05_b200 \
    --job_name=${JOB_NAME}_pi05_b200 \
    --wandb.enable=true \
    --batch_size=$((BATCH_SIZE / GPU_NUMBER)) \
    --policy.optimizer_lr=1.1e-4 \
    --steps=${STEPS_NUMBER} \
    --save_freq=$((STEPS_NUMBER / CHECKPOINT_NUMBER)) \
    --num_workers=10 \
    --seed=55 \
    --tolerance_s=0.1 \
    --resume=false
