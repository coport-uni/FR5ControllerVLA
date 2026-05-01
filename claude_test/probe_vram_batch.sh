#!/usr/bin/env bash
# Probe peak per-GPU VRAM for `7__train_act_adv.sh` at a given
# `--batch_size`. Disables wandb / HF Hub push, runs accelerate
# launch in the background, samples nvidia-smi every 2 s, and kills
# the run once a stable peak is detected (or after a timeout).
#
# Usage: claude_test/probe_vram_batch.sh <per_gpu_batch_size> [duration_s]
#
# Lifetime: throwaway diagnostic for ToDo 2026-05-01. Safe to delete
# once batch_size for `7__train_act_adv.sh` is decided.

set -uo pipefail

BATCH="${1:?per-GPU batch_size required as arg 1}"
DURATION="${2:-150}"  # seconds to let training warm up before kill

_conda_sh=""
for _root in /home/inno-controller/anaconda3 /opt/conda \
             "$HOME/anaconda3" "$HOME/miniconda3"; do
    if [ -f "$_root/etc/profile.d/conda.sh" ]; then
        _conda_sh="$_root/etc/profile.d/conda.sh"
        break
    fi
done
if [ -z "$_conda_sh" ]; then
    echo "ERROR: no conda install found" >&2
    exit 1
fi

export NCCL_P2P_DISABLE=1
# Keep TQDM / log noise low so we can see kill signals.
export TQDM_DISABLE=1
export TRANSFORMERS_VERBOSITY=error

# shellcheck disable=SC1090
source "$_conda_sh"
conda activate lerobot

JOB_NAME="probe_act_bs${BATCH}_$(date +%s)"
OUT_DIR="/tmp/${JOB_NAME}"
LOG_FILE="/tmp/${JOB_NAME}.log"
SAMPLE_FILE="/tmp/${JOB_NAME}.nvsmi.csv"

echo "=== probe bs=${BATCH} duration=${DURATION}s ==="
echo "log:    ${LOG_FILE}"
echo "sample: ${SAMPLE_FILE}"

# Launch training in the background. We do NOT push to hub, do NOT
# log to wandb, write to /tmp so we don't pollute outputs/.
accelerate launch \
    --multi_gpu \
    --num_processes=3 \
    --mixed_precision=fp16 \
    "$(which lerobot-train)" \
    --dataset.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box \
    --dataset.video_backend=pyav \
    --policy.type=act \
    --policy.push_to_hub=false \
    --policy.device=cuda \
    --output_dir="${OUT_DIR}" \
    --job_name="${JOB_NAME}" \
    --wandb.enable=false \
    --num_workers=10 \
    --batch_size="${BATCH}" \
    --steps=1000 \
    --save_freq=999999 \
    --log_freq=10 \
    --resume=false \
    --seed=55 \
    --tolerance_s=0.1 \
    > "${LOG_FILE}" 2>&1 &
TRAIN_PID=$!
echo "train_pid=${TRAIN_PID}"

# Sample VRAM every 2 s for DURATION seconds.
echo "ts,gpu,mem_used_mib" > "${SAMPLE_FILE}"
START=$(date +%s)
while :; do
    NOW=$(date +%s)
    ELAPSED=$(( NOW - START ))
    if [ "${ELAPSED}" -ge "${DURATION}" ]; then break; fi
    if ! kill -0 "${TRAIN_PID}" 2>/dev/null; then
        echo "train process exited early (elapsed=${ELAPSED}s)"
        break
    fi
    nvidia-smi --query-gpu=index,memory.used \
               --format=csv,noheader,nounits \
        | awk -v t="${ELAPSED}" '{print t","$0}' \
        >> "${SAMPLE_FILE}"
    sleep 2
done

# Kill the training cleanly.
echo "killing train_pid=${TRAIN_PID} and children"
pkill -P "${TRAIN_PID}" 2>/dev/null
kill "${TRAIN_PID}" 2>/dev/null
sleep 3
pkill -9 -f "lerobot-train" 2>/dev/null
pkill -9 -f "accelerate" 2>/dev/null
sleep 2

# Compute peak per GPU.
echo "=== peak VRAM (MiB) per GPU, bs=${BATCH} ==="
awk -F, 'NR>1 {if ($3+0 > peak[$2]) peak[$2]=$3+0}
         END {for (g in peak) printf "  GPU %s: %d MiB (%.1f%% of 24576)\n",
                                   g, peak[g], 100.0*peak[g]/24576}' \
    "${SAMPLE_FILE}" | sort

# Show the last few log lines for context (helps spot OOM).
echo "=== last 25 log lines ==="
tail -n 25 "${LOG_FILE}"

# Cleanup output dir to keep disk tidy.
rm -rf "${OUT_DIR}"
