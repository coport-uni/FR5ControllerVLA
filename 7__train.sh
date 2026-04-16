source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot

HF_USER=$(hf auth whoami | head -n 1)
echo $HF_USER

# python ./src/lerobot/scripts/train.py \
# --policy.device=cuda  \
# --policy.type=smolvla   \
# --policy.repo_id=wego-hansu/piper_teleop_smolvla_B  \
# --policy.push_to_hub=true \
# --dataset.repo_id=wego-hansu/piper_tele_pick_cars_033_B   \
# --dataset.video_backend=pyav   \
# --batch_size=64   \
# --steps=200000   \
# --eval_freq=0   \
# --save_freq=5000 \
# --output_dir=outputs/train/piper_smolvla_teleop_033_B   \
# --job_name=piper_smolvla_cars   \
# --wandb.enable=false  \
# --resume=false  \

python ./src/lerobot/scripts/train.py \
--resume=true  \
--config_path="/home/sw-han/lerobot/outputs/train/piper_smolvla_teleop_033_B/checkpoints/last/pretrained_model/" \
