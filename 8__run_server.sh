#!/bin/bash
# FR5 async-inference policy server.
# Loads the trained policy and serves action chunks to the client over gRPC.
# Paired with 9__run_client.sh (client connects to 127.0.0.1:8088).
source /home/inno-controller/anaconda3/etc/profile.d/conda.sh
conda activate lerobot

python -m lerobot.async_inference.policy_server \
    --host=127.0.0.1 \
    --port=8088 \
    --fps=20
