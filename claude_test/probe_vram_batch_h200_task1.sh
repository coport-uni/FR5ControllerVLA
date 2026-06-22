#!/usr/bin/env bash
# Probe peak per-GPU VRAM for `7__train_act_task1_h200.sh` at a
# given `--batch_size`. H200 NVL 141 GB x 2 counterpart of #56's
# `probe_vram_batch.sh`: num_processes=2, mixed_precision=bf16,
# task1 dataset, 143771 MiB denominator.
#
# Usage: claude_test/probe_vram_batch_h200_task1.sh <bs> [duration_s]
#
# Lifetime: throwaway diagnostic for gh issue #84. Safe to delete
# once batch_size for `7__train_act_task1_h200.sh` is decided.

set -uo pipefail

BATCH="${1:?per-GPU batch_size required as arg 1}"
DURATION="${2:-300}"  # seconds (first run is longer due to dataset cache miss)

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

# NCCL transport (issue #30 workaround). Quiet TQDM / HF logs so the
# kill signal is visible in tail.
export NCCL_P2P_DISABLE=1
export TQDM_DISABLE=1
export TRANSFORMERS_VERBOSITY=error

# shellcheck disable=SC1090
source "$_conda_sh"
conda activate lerobot

DATASET="coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_50"
JOB_NAME="probe_act_h200_task1_bs${BATCH}_$(date +%s)"
OUT_DIR="/tmp/${JOB_NAME}"
LOG_FILE="/tmp/${JOB_NAME}.log"
SAMPLE_FILE="/tmp/${JOB_NAME}.nvsmi.csv"
# H200 NVL total memory in MiB (nvidia-smi reports 143771).
GPU_TOTAL_MIB=143771

echo "=== probe bs=${BATCH} duration=${DURATION}s (H200 2x, bf16) ==="
echo "log:    ${LOG_FILE}"
echo "sample: ${SAMPLE_FILE}"

# Launch training in the background. wandb and HF Hub push are off;
# output goes to /tmp so we don't pollute outputs/.
accelerate launch \
    --multi_gpu \
    --num_processes=2 \
    --mixed_precision=bf16 \
    "$(which lerobot-train)" \
    --dataset.repo_id="${DATASET}" \
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
awk -F, -v total="${GPU_TOTAL_MIB}" \
    'NR>1 {if ($3+0 > peak[$2]) peak[$2]=$3+0}
     END {for (g in peak) printf "  GPU %s: %d MiB (%.1f%% of %d)\n",
                                g, peak[g], 100.0*peak[g]/total, total}' \
    "${SAMPLE_FILE}" | sort

# Show the last few log lines for context (helps spot OOM).
echo "=== last 25 log lines ==="
tail -n 25 "${LOG_FILE}"

# Cleanup output dir to keep disk tidy.
rm -rf "${OUT_DIR}"
