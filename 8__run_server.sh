#!/bin/bash
# FR5 async-inference policy server.
#
# Boots an empty PolicyServer on 127.0.0.1:8080. The actual policy and
# robot metadata are pushed by 9__run_client.sh during the initial
# gRPC handshake -- no checkpoint is loaded here.
#
# Prerequisite: pip install -e ".[async]"   (grpcio + matplotlib extras)
# Docs:        https://huggingface.co/docs/lerobot/async
# Paired with: 9__run_client.sh

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

python -m lerobot.async_inference.policy_server \
    --host=127.0.0.1 \
    --port=8080 \
    --fps=20 \
    --inference_latency=0.05
