#!/usr/bin/env bash
# Ten-minute smoke test of 7__train_pi05_task3_b200.sh after the
# 2026-07-14 container migration. Verifies the rebuilt toolchain
# actually trains; it is NOT a real training run and its checkpoints
# are throwaway.
#
# Deliberate deviations from the production script (all temporary):
#   - GPU_NUMBER=3: only 3 of 4 B200s enumerate on this box right now.
#     Per-GPU batch stays at the measured 152 rung, so the compile /
#     VRAM behaviour under test matches production; only the global
#     batch (456 vs 608) differs. LR is therefore NOT the production
#     1.1e-4 -- see below.
#   - wandb disabled and push_to_hub disabled: credentials were lost in
#     the migration and a smoke test should not publish anything.
#   - steps/save_freq shrunk so the run never checkpoints: we are
#     testing the step loop, not producing a model.
#
# Expected outcome: after ~2-4 min of dataset load + torch.compile,
# the step loop advances and loss decreases. Anything else is a
# migration regression.

set -uo pipefail

_conda_root=/NHNHOME/workspace/sungwoo/miniforge3
source "${_conda_root}/etc/profile.d/conda.sh"
conda activate lerobot
export PATH="${_conda_root}/envs/lerobot/bin:${PATH}"

# The HF cache default (~/.cache) lives on the container overlay and was
# wiped by the migration. Keep it on the persistent disk instead.
export HF_HOME=/NHNHOME/workspace/sungwoo/hf_cache

# /tmp is noexec here, so torch.compile/triton cannot dlopen kernels it
# writes there. Same redirect as the production script (LP E11).
_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export TORCHINDUCTOR_CACHE_DIR="${_repo_root}/.cache/torchinductor"
export TRITON_CACHE_DIR="${_repo_root}/.cache/triton"
mkdir -p "${TORCHINDUCTOR_CACHE_DIR}" "${TRITON_CACHE_DIR}"

# P2P must stay enabled on this bare-metal NVLink box (LP E13/issue #87).
export NCCL_P2P_DISABLE=0

JOB_NAME="FR5_task3_turn_the_sliver_air_valve_90_degress_counterclockwise_200"
GPU_NUMBER=3
PER_GPU_BATCH=152

# Production uses lr=1.1e-4 for a 608 global batch (sqrt scaling off
# openpi's 2.5e-5 @ 32). This run's global batch is 456, so the matching
# sqrt-scaled lr is 2.5e-5 * sqrt(456/32) = 9.4e-5. Kept consistent so
# the observed loss curve is meaningful rather than mis-scaled.
LR=9.4e-5

cd "${_repo_root}"

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
    --policy.push_to_hub=false \
    --policy.device=cuda \
    --policy.compile_model=true \
    --policy.compile_mode=default \
    --policy.gradient_checkpointing=true \
    --policy.dtype=bfloat16 \
    --policy.freeze_vision_encoder=false \
    --policy.train_expert_only=false \
    --policy.optimizer_weight_decay=1e-10 \
    --output_dir=outputs/train/VERIFY_${JOB_NAME}_pi05_b200_10min \
    --job_name=VERIFY_${JOB_NAME}_pi05_b200_10min \
    --wandb.enable=false \
    --batch_size=${PER_GPU_BATCH} \
    --policy.optimizer_lr=${LR} \
    --steps=500 \
    --save_freq=100000 \
    --log_freq=1 \
    --num_workers=10 \
    --seed=55 \
    --tolerance_s=0.1 \
    --resume=false
