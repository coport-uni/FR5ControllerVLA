#!/usr/bin/env bash
# Train an ACT policy on the FR5 teleoperation dataset.
# Reference: https://huggingface.co/docs/lerobot/act
# Multi-GPU: https://huggingface.co/docs/lerobot/multi_gpu_training
# ACT ships with the base LeRobot install; no extra pip group needed.
#
# Launched via `accelerate launch` to use 2x H200 GPUs. With
# --num_processes=2, the effective (global) batch size is
# batch_size * num_processes, so --batch_size=256 here -> global 512.
# Mixed precision bf16 is chosen for Hopper (H200).

source /home/inno-controller/anaconda3/etc/profile.d/conda.sh
conda activate lerobot

HF_USER=$(hf auth whoami | head -n 1)
echo "HF_USER=${HF_USER}"

JOB_NAME="fr5_act_test1"

accelerate launch \
    --multi_gpu \
    --num_processes=2 \
    --mixed_precision=bf16 \
    "$(which lerobot-train)" \
    --dataset.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box \
    --dataset.video_backend=pyav \
    --policy.type=act \
    --policy.repo_id=${HF_USER}/${JOB_NAME}_model \
    --policy.push_to_hub=true \
    --policy.device=cuda \
    --output_dir=outputs/train/${JOB_NAME} \
    --job_name=${JOB_NAME} \
    --wandb.enable=true \
    --num_workers=10 \
    --batch_size=256 \
    --steps=200000 \
    --save_freq=5000 \
    --resume=false
