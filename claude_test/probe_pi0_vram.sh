#!/usr/bin/env bash
# probe_pi0_vram.sh -- Run a short Pi0 fine-tune at the given per-GPU
# batch_size and report peak VRAM. Probe only; production isolated.
#
# Usage: probe_pi0_vram.sh <per_gpu_batch_size>
#   STEPS=<n>     override step count (default 50)
#
# Outputs:
#   claude_test/probe_logs/vram_b${BATCH}.csv   nvidia-smi 1 Hz samples
#   claude_test/probe_logs/train_b${BATCH}.log  accelerate/lerobot stdout
#   outputs/train/fr5_pi0_vram_probe_b${BATCH}/  cleaned each run
#
# See ToDo 2026-05-01 / GitHub issue #55.

set -u

BATCH="${1:?Usage: $0 <per_gpu_batch_size>}"
STEPS="${STEPS:-50}"

# Mirror conda discovery from 7__train_pi0_adv.sh.
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
# shellcheck disable=SC1090
source "$_conda_sh"
conda activate lerobot

# NCCL P2P workaround (LP §1, issue #30).
export NCCL_P2P_DISABLE=1

JOB_NAME="fr5_pi0_vram_probe_b${BATCH}"
OUTPUT_DIR="outputs/train/${JOB_NAME}"
LOG_DIR="claude_test/probe_logs"
mkdir -p "${LOG_DIR}"
VRAM_LOG="${LOG_DIR}/vram_b${BATCH}.csv"
TRAIN_LOG="${LOG_DIR}/train_b${BATCH}.log"

# Wipe prior probe output so each run starts clean.
rm -rf "${OUTPUT_DIR}"

# Background sampler: 1 Hz, both GPUs.
echo "ts,gpu0_used_mib,gpu1_used_mib" > "${VRAM_LOG}"
(
    while :; do
        ts=$(date +%s)
        line=$(nvidia-smi --query-gpu=memory.used \
               --format=csv,noheader,nounits -i 0,1 \
               | tr '\n' ',' | sed 's/,$//')
        # line is "gpu0,gpu1"
        echo "${ts},${line}" >> "${VRAM_LOG}"
        sleep 1
    done
) &
MON_PID=$!
trap 'kill ${MON_PID} 2>/dev/null || true' EXIT INT TERM

echo "[probe] BATCH=${BATCH} STEPS=${STEPS} JOB_NAME=${JOB_NAME}"
echo "[probe] Logs: ${TRAIN_LOG} / ${VRAM_LOG}"

set +e
accelerate launch \
    --multi_gpu \
    --num_processes=2 \
    --mixed_precision=bf16 \
    "$(which lerobot-train)" \
    --dataset.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box \
    --policy.type=pi0 \
    --policy.pretrained_path=lerobot/pi0_base \
    --policy.push_to_hub=false \
    --dataset.video_backend=pyav \
    --policy.device=cuda \
    --policy.compile_model=true \
    --policy.gradient_checkpointing=true \
    --policy.dtype=bfloat16 \
    --policy.freeze_vision_encoder=false \
    --policy.train_expert_only=false \
    --output_dir="${OUTPUT_DIR}" \
    --job_name="${JOB_NAME}" \
    --wandb.enable=false \
    --batch_size="${BATCH}" \
    --steps="${STEPS}" \
    --save_freq=999999 \
    --num_workers=10 \
    --seed=55 \
    --tolerance_s=0.1 \
    > "${TRAIN_LOG}" 2>&1
EXIT_CODE=$?
set -e

# Stop sampler.
kill ${MON_PID} 2>/dev/null || true
wait ${MON_PID} 2>/dev/null || true

# Compute per-GPU peak.
PEAK0=$(awk -F, 'NR>1 && $2+0>m{m=$2+0}END{print m+0}' "${VRAM_LOG}")
PEAK1=$(awk -F, 'NR>1 && $3+0>m{m=$3+0}END{print m+0}' "${VRAM_LOG}")
PEAK_GB0=$(awk "BEGIN{printf \"%.2f\", ${PEAK0}/1024}")
PEAK_GB1=$(awk "BEGIN{printf \"%.2f\", ${PEAK1}/1024}")

# Detect OOM in the train log.
OOM=""
if grep -qiE "out of memory|OutOfMemoryError|CUDA error: out of memory" "${TRAIN_LOG}"; then
    OOM=" (OOM detected in log)"
fi

echo
echo "[probe] BATCH=${BATCH} done; exit=${EXIT_CODE}${OOM}"
echo "[probe] Peak GPU0: ${PEAK0} MiB (${PEAK_GB0} GB)"
echo "[probe] Peak GPU1: ${PEAK1} MiB (${PEAK_GB1} GB)"

exit ${EXIT_CODE}
