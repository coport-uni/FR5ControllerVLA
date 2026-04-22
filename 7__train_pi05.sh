#!/usr/bin/env bash
# Train a Pi0.5 policy on the FR5 teleoperation dataset.
# Reference: https://huggingface.co/docs/lerobot/pi05
# Requires: pip install -e ".[pi]"
#
# Pi0.5 needs augmented quantile stats on the dataset before training. Run
# once (and push to the Hub) before invoking this script:
#   python src/lerobot/datasets/v30/augment_dataset_quantile_stats.py \
#       --repo-id=coport-uni/FR5_pick_red_colored_marker_to_box

source /home/inno-controller/anaconda3/etc/profile.d/conda.sh
conda activate lerobot

HF_USER=$(hf auth whoami | head -n 1)
echo "HF_USER=${HF_USER}"

JOB_NAME="fr5_pi05_test1"

lerobot-train \
    --dataset.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box \
    --policy.type=pi05 \
    --policy.pretrained_path=lerobot/pi05_base \
    --policy.repo_id=${HF_USER}/${JOB_NAME}_model \
    --policy.push_to_hub=true \
    --policy.device=cuda \
    --policy.compile_model=true \
    --policy.gradient_checkpointing=true \
    --policy.dtype=bfloat16 \
    --policy.freeze_vision_encoder=false \
    --policy.train_expert_only=false \
    --output_dir=outputs/train/${JOB_NAME} \
    --job_name=${JOB_NAME} \
    --wandb.enable=true \
    --batch_size=32 \
    --steps=3000 \
    --save_freq=500 \
    --resume=false
