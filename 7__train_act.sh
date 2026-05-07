#!/usr/bin/env bash
# Train an ACT policy on the FR5 teleoperation dataset.
# Reference: https://huggingface.co/docs/lerobot/act
# Multi-GPU: https://huggingface.co/docs/lerobot/multi_gpu_training
# ACT ships with the base LeRobot install; no extra pip group needed.
#
# Launched via `accelerate launch` to use 2x H200 GPUs. With
# --num_processes=2, the effective (global) batch size is
# batch_size * num_processes, so --batch_size=8 here -> global 16.
# Aloha-level training volume: 500000 steps x global 16 = 8M samples
# (~200 epoch on a 40k-timestep dataset), matching the ACT paper
# scale (Zhao 2023, arXiv:2304.13705). LR is left at the LeRobot
# default 1e-5 per docs/source/multi_gpu_training.mdx (no auto-scale).
# Mixed precision bf16 is chosen for Hopper (H200).

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

JOB_NAME="fr5_act_red_marker"

accelerate launch \
    --multi_gpu \
    --num_processes=3 \
    --mixed_precision=bf16 \
    "$(which lerobot-train)" \
    --dataset.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box \
    --dataset.video_backend=pyav \
    --policy.type=act \
    --policy.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box_model \
    --policy.push_to_hub=true \
    --policy.device=cuda \
    --output_dir=outputs/train/${JOB_NAME} \
    --job_name=${JOB_NAME} \
    --wandb.enable=true \
    --num_workers=10 \
    --batch_size=8 \
    --steps=500000 \
    --save_freq=10000 \
    --resume=false \
    --seed=55 \
    --tolerance_s=0.1 \
