# Read belows articles before start
# https://huggingface.co/docs/lerobot/installation

conda create -y -n lerobot python=3.12
conda activate lerobot
conda install ffmpeg -c conda-forge
apt-get install cmake build-essential python3-dev pkg-config libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libswresample-dev libavfilter-dev

git clone https://github.com/coport-uni/FR5ControllerVLA.git

# https://github.com/AgRoboticsResearch/lerobot_robot_piper
pip install lerobot_robot_piper

