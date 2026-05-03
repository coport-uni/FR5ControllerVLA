#!/usr/bin/env bash
# Train a Pi0.5 policy on the FR5 teleoperation dataset.
# Reference: https://huggingface.co/docs/lerobot/pi05
#            https://arxiv.org/abs/2504.16054 (original Pi0.5 paper)
#            https://github.com/Physical-Intelligence/openpi
# Requires: pip install -e ".[pi]"
#
# VRAM probe (issue #58, claude_test/probe_logs/SUMMARY_pi05.md):
# on this 2 x H200 NVL box (143.77 GB / GPU) per-GPU batch=136 hits
# ~76 % VRAM (max(GPU0, GPU1) = 108.78 GB), batch=140 OOMs. Pi0.5 is
# heavier than pi0 here (gemma backbone), which fits 160 at 79.6 %.
# Effective batch is 16 x 2 = 32 -> 136 x 2 = 272 (8.5 x). lr is
# scaled by SQRT(8.5) ~= 2.92 from the openpi default 2.5e-5 to
# 7.3e-5; warmup_steps stays at the LeRobot/openpi default 1000.
# --steps=3000 keeps the openpi sample budget (~7 epoch on the
# 119,356-frame FR5_pick_red_colored_marker_to_box dataset).
#
# Hyperparameters here track openpi's reference defaults
# (optimizer.py: AdamW betas=(0.9, 0.95) eps=1e-8 wd=1e-10
# clip_grad=1.0; CosineDecay warmup=1000 peak=2.5e-5 decay=30000
# decay_lr=2.5e-6) and the paper's training-stage step counts
# (post-train = 80k; openpi's shortest fine-tune preset = 30k).
# PI05Config defaults already match openpi for everything except
# weight_decay (LeRobot=0.01, openpi=1e-10), which we override below.
#
# Two paper/openpi features LeRobot's pi05 cannot reproduce:
#   - EMA (ema_decay=0.99): not implemented in LeRobot pi0/pi05.
#   - Per-dataset 1%/99% quantile action normalization: replaced by
#     MEAN_STD override below. This is the HF-doc-sanctioned escape
#     hatch (docs/source/pi05.mdx §"If your dataset is not converted
#     with quantiles..."). The proper alternative is to pre-augment
#     the dataset and push to Hub; we deliberately skip that step:
#     python src/lerobot/datasets/v30/augment_dataset_quantile_stats.py \
#         --repo-id=coport-uni/FR5_pick_red_colored_marker_to_box

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

# SHM works. So no need to disable it 
# export NCCL_SHM_DISABLE=1

HF_USER=$(hf auth whoami | head -n 1)
echo "HF_USER=${HF_USER}"

JOB_NAME="fr5_pi05_red_marker_adv"

accelerate launch \
    --multi_gpu \
    --num_processes=2 \
    --mixed_precision=bf16 \
    "$(which lerobot-train)" \
    --dataset.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box \
    --policy.type=pi05 \
    --policy.pretrained_path=lerobot/pi05_base \
    --policy.normalization_mapping='{"ACTION": "MEAN_STD", "STATE": "MEAN_STD", "VISUAL": "IDENTITY"}' \
    --dataset.video_backend=pyav \
    --policy.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box_pi05_adv_model \
    --policy.push_to_hub=true \
    --policy.device=cuda \
    --policy.compile_model=true \
    --policy.gradient_checkpointing=true \
    --policy.dtype=bfloat16 \
    --policy.freeze_vision_encoder=false \
    --policy.train_expert_only=false \
    --policy.optimizer_weight_decay=1e-10 \
    --output_dir=outputs/train/${JOB_NAME} \
    --job_name=${JOB_NAME} \
    --wandb.enable=true \
    --batch_size=136 \
    --policy.optimizer_lr=7.3e-5 \
    --steps=3000 \
    --save_freq=500 \
    --resume=false \
    --seed=55 \
    --tolerance_s=0.1
