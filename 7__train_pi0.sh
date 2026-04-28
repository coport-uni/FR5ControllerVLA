#!/usr/bin/env bash
# Train a Pi0 policy on the FR5 teleop dataset using PI0 paper / openpi
# production fine-tune defaults (vs. 7__train_pi0.sh which follows the
# LeRobot docs quickstart with steps=3000).
#
# Reference settings (openpi src/openpi/training/config.py):
#   - num_train_steps = 30_000
#   - batch_size      = 32
#   - optimizer       = AdamW lr=2.5e-5 betas=(0.9, 0.95) wd~=0
#   - scheduler       = warmup-cosine, warmup=1_000, decay=30_000
#   - dtype           = bfloat16
#   - full fine-tune (vision encoder NOT frozen, expert NOT only)
#
# Note: openpi's EMA (ema_decay=0.99) is NOT implemented in LeRobot pi0,
#       so this script does not enable it. Optimizer weight_decay is left
#       at the LeRobot default 0.01 (openpi uses ~1e-10); add
#       --policy.optimizer_weight_decay=1e-10 if you need exact parity.
#
# Reference: https://github.com/Physical-Intelligence/openpi
# Requires: pip install -e ".[pi]"

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

source "$_conda_sh"
conda activate lerobot

# Force NCCL socket transport inside this Docker image; P2P/CUMEM
# fails silently on the first allreduce/barrier here and stalls DDP
# forever. Harmless on single-GPU runs (no torch.distributed). See
# issue #30 for the full diagnosis.
export NCCL_P2P_DISABLE=1
export NCCL_SHM_DISABLE=1

HF_USER=$(hf auth whoami | head -n 1)
echo "HF_USER=${HF_USER}"

JOB_NAME="fr5_pi0_red_marker_base"

accelerate launch \
    --multi_gpu \
    --num_processes=2 \
    --mixed_precision=bf16 \
    "$(which lerobot-train)" \
    --dataset.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box \
    --policy.type=pi0 \
    --policy.pretrained_path=lerobot/pi0_base \
    --policy.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box_pi0_model_paper \
    --policy.push_to_hub=true \
    --dataset.video_backend=pyav \
    --policy.device=cuda \
    --policy.compile_model=true \
    --policy.gradient_checkpointing=true \
    --policy.dtype=bfloat16 \
    --policy.freeze_vision_encoder=false \
    --policy.train_expert_only=false \
    --output_dir=outputs/train/${JOB_NAME} \
    --job_name=${JOB_NAME} \
    --wandb.enable=true \
    --batch_size=16 \
    --steps=30000 \
    --save_freq=5000 \
    --resume=false \
    --num_workers=10 \
    --seed=55 \
    --tolerance_s=0.1
