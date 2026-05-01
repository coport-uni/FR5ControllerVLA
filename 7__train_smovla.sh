#!/usr/bin/env bash
# Train a SmolVLA policy on the FR5 teleop dataset following the
# SmolVLA paper (arXiv:2506.01844) and the LeRobot docs recipe.
#
# Reference: https://huggingface.co/docs/lerobot/smolvla
# Paper:     https://arxiv.org/pdf/2506.01844
# Requires:  pip install -e ".[smolvla]"
#
# About the "2B" question
# -----------------------
# SmolVLA ships exactly one pretrained checkpoint on the Hub:
#   lerobot/smolvla_base ~= 450M params (SmolVLM2-500M backbone +
#                                        action expert).
# There is NO official 2B SmolVLA. The SmolVLM2 family does have a
# 2.2B variant, and you can swap the backbone in via
#   --policy.vlm_model_name=HuggingFaceTB/SmolVLM2-2.2B-Instruct
#   --policy.load_vlm_weights=true
# but doing so loses the pretrained SmolVLA action-expert weights
# (you'd be training the expert from scratch on top of a 2.2B VLM).
# We keep the 450M recipe active here and leave the 2.2B opt-in
# commented out at the bottom of the launch block.
#
# Reference settings (src/lerobot/policies/smolvla/configuration_smolvla.py
# defaults match the paper, so most of them stay implicit):
#   - num_train_steps = 20_000   (LeRobot docs SmolVLA quickstart)
#   - batch_size      = 64       (docs; we use 32 per-rank x 2 GPUs)
#   - optimizer       = AdamW lr=1e-4 betas=(0.9, 0.95) wd=1e-10
#   - scheduler       = warmup-cosine, warmup=1_000, decay=30_000 -> 2.5e-6
#   - dtype           = bfloat16 (via accelerate --mixed_precision=bf16;
#                                 SmolVLAConfig has no `dtype` field)
#   - freeze_vision_encoder=true, train_expert_only=true (paper fine-tune)
#
# Note: SmolVLAConfig does not expose `gradient_checkpointing` (only
#       `compile_model`), so we don't pass --policy.gradient_checkpointing.

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
#export NCCL_SHM_DISABLE=1

HF_USER=$(hf auth whoami | head -n 1)
echo "HF_USER=${HF_USER}"

JOB_NAME="fr5_smolvla_red_marker_base_single"

# lerobot-train \
#     --dataset.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box \
#     --policy.path=lerobot/smolvla_base \
#     --dataset.video_backend=pyav \
#     --policy.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box_smolvla_model \
#     --policy.push_to_hub=true \
#     --dataset.video_backend=pyav \
#     --policy.device=cuda \
#     --rename_map='{"observation.images.hand": "observation.images.camera1", "observation.images.top_left": "observation.images.camera2", "observation.images.top_right": "observation.images.camera3"}' \
#     --policy.compile_model=true \
#     --policy.freeze_vision_encoder=true \
#     --policy.train_expert_only=true \
#     --output_dir=outputs/train/${JOB_NAME} \
#     --job_name=${JOB_NAME} \
#     --wandb.enable=true \
#     --batch_size=32 \
#     --steps=20000 \
#     --save_freq=5000 \
#     --resume=false \
#     --num_workers=10 \
#     --seed=55 \
#     --tolerance_s=0.1
    
accelerate launch \
    --multi_gpu \
    --num_processes=3 \
    --mixed_precision=bf16 \
    "$(which lerobot-train)" \
    --dataset.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box \
    --policy.path=lerobot/smolvla_base \
    --dataset.video_backend=pyav \
    --policy.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box_smolvla_model \
    --policy.push_to_hub=true \
    --dataset.video_backend=pyav \
    --policy.device=cuda \
    --rename_map='{"observation.images.hand": "observation.images.camera1", "observation.images.top_left": "observation.images.camera2", "observation.images.top_right": "observation.images.camera3"}' \
    --policy.compile_model=true \
    --policy.freeze_vision_encoder=true \
    --policy.train_expert_only=true \
    --output_dir=outputs/train/${JOB_NAME} \
    --job_name=${JOB_NAME} \
    --wandb.enable=true \
    --batch_size=32 \
    --steps=20000 \
    --save_freq=5000 \
    --resume=false \
    --num_workers=10 \
    --seed=55 \
    --tolerance_s=0.1

# --- Opt-in: 2.2B SmolVLM2 backbone (no SmolVLA pretrained weights) ---
# Drop --policy.path above and add the two flags below to swap in the
# 2.2B SmolVLM2 backbone. Caveats: action expert trains from scratch
# (no SmolVLA fine-tune init), VRAM ~4-5x higher, and the paper
# hyperparameters here were tuned for the 500M backbone.
#
#     --policy.type=smolvla \
#     --policy.vlm_model_name=HuggingFaceTB/SmolVLM2-2.2B-Instruct \
#     --policy.load_vlm_weights=true \
