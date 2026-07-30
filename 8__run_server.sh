#!/bin/bash
# FR5 async-inference policy server.
#
# Boots an empty PolicyServer on 127.0.0.1:8080. The actual policy and
# robot metadata are pushed by 9__run_client.sh during the initial
# gRPC handshake -- no checkpoint is loaded here.
#
# Prerequisite: pip install -e ".[async, smovla]"   (grpcio + matplotlib extras)
# Docs:        https://huggingface.co/docs/lerobot/async
# Paired with: 9__run_client.sh

_conda_sh=""
for _root in /NHNHOME/workspace/sungwoo/miniforge3 \
             /home/inno-controller/anaconda3 /opt/conda \
             "$HOME/anaconda3" "$HOME/miniconda3"; do
    if [ -f "$_root/etc/profile.d/conda.sh" ]; then
        _conda_sh="$_root/etc/profile.d/conda.sh"
        _conda_root="$_root"
        break
    fi
done

if [ -z "$_conda_sh" ]; then
    echo "ERROR: no conda install found (tried inno-controller, /opt/conda, \$HOME)" >&2
    exit 1
fi

source "$_conda_sh"
conda activate lerobot

# ACT. inference_latency is a floor on GetActions duration: the
# server pads faster calls up to it and never delays slower ones
# (policy_server.py GetActions). Measured in gh #113: pure ACT
# forward ~8 ms, full server pipeline 240-320 ms per 100-action
# chunk. 0.01 s sits at the forward-pass scale, below any real
# call, so chunks always flow at true pipeline speed -- even if
# pre/postprocessing gets faster later.
python3 -m lerobot.async_inference.policy_server \
    --host=0.0.0.0 \
    --port=17044 \
    --fps=20 \
    --inference_latency=0.01

# SmolVLA
# python3 -m lerobot.async_inference.policy_server \
#     --host=0.0.0.0 \
#     --port=17044 \
#     --fps=20 \
#     --inference_latency=0.1

# Pi0
# python3 -m lerobot.async_inference.policy_server \
#     --host=0.0.0.0 \
#     --port=17044 \
#     --fps=20 \
#     --inference_latency=1
