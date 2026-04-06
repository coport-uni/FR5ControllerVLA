source /opt/conda/etc/profile.d/conda.sh
conda activate lerobot

python ./src/lerobot/find_cameras.py
nautilus ./outputs/captured_images