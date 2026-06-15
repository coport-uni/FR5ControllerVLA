#!/usr/bin/env bash
# probe_pi05_vram_b200.sh -- B200 (4-GPU) Pi0.5 VRAM probe.
# Run a short Pi0.5 fine-tune at the given per-GPU batch_size on
# 4 x B200 and report peak VRAM per GPU. Probe only; production isolated.
#
# Combines:
#   - pi05 policy options from probe_pi05_vram.sh (pi05_base, MEAN_STD
#     normalization, weight_decay=1e-10).
#   - B200 environment from probe_pi0_vram_b200.sh (miniforge3 conda
#     root + PATH fix, torchinductor/triton cache redirect for noexec
#     /tmp, NCCL P2P forced ON, 4 GPUs sampled).
#
# pi0 on this box fits per-GPU batch 176 @ 73.9 % (SUMMARY_b200.md);
# pi05 is ~15 % heavier per sample (SUMMARY_pi05.md), so the boundary
# is expected lower -- probe to find it.
#
# Usage: probe_pi05_vram_b200.sh <per_gpu_batch_size>
#   STEPS=<n>   override step count (default 50)
#
# Outputs:
#   claude_test/probe_logs/vram_pi05_b200_b${BATCH}.csv  nvidia-smi 1 Hz
#   claude_test/probe_logs/train_pi05_b200_b${BATCH}.log accelerate stdout
#   outputs/train/fr5_pi05_vram_probe_b200_b${BATCH}/    cleaned each run
#
# See ToDo 2026-06-15 / GitHub issue #89 (follows #58, #87).

set -u

BATCH="${1:?Usage: $0 <per_gpu_batch_size>}"
STEPS="${STEPS:-50}"
GPU_NUMBER=4

# Conda discovery: miniforge3 on this B200 box first, then the roots
# used by the H200/FR5 scripts.
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
    echo "ERROR: no conda install found" >&2
    exit 1
fi
# shellcheck disable=SC1090
source "$_conda_sh"
conda activate lerobot
# Force the env bin ahead of /usr/bin so bare `python` is the env one.
export PATH="${_conda_root}/envs/lerobot/bin:${PATH}"

# /tmp is mounted noexec on this box, so torch.compile/triton cannot
# dlopen the kernels it writes to /tmp/torchinductor_* ("failed to map
# segment from shared object"). Redirect both caches to an exec-allowed
# workspace dir. Required whenever compile_model=true on this host.
_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export TORCHINDUCTOR_CACHE_DIR="${_repo_root}/.cache/torchinductor"
export TRITON_CACHE_DIR="${_repo_root}/.cache/triton"
mkdir -p "${TORCHINDUCTOR_CACHE_DIR}" "${TRITON_CACHE_DIR}"

# NCCL: P2P must stay ENABLED on this bare-metal B200 NVLink/NVSwitch
# box. The inherited NCCL_P2P_DISABLE=1 (issue #30, a Docker-image
# workaround) HANGS the first allreduce here. See issue #87.
export NCCL_P2P_DISABLE=0

JOB_NAME="fr5_pi05_vram_probe_b200_b${BATCH}"
OUTPUT_DIR="outputs/train/${JOB_NAME}"
LOG_DIR="claude_test/probe_logs"
mkdir -p "${LOG_DIR}"
VRAM_LOG="${LOG_DIR}/vram_pi05_b200_b${BATCH}.csv"
TRAIN_LOG="${LOG_DIR}/train_pi05_b200_b${BATCH}.log"

# Wipe prior probe output so each run starts clean.
rm -rf "${OUTPUT_DIR}"

# Background sampler: 1 Hz, all 4 GPUs.
echo "ts,gpu0_used_mib,gpu1_used_mib,gpu2_used_mib,gpu3_used_mib" \
    > "${VRAM_LOG}"
(
    while :; do
        ts=$(date +%s)
        line=$(nvidia-smi --query-gpu=memory.used \
               --format=csv,noheader,nounits -i 0,1,2,3 \
               | tr '\n' ',' | sed 's/,$//')
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
    --num_processes=${GPU_NUMBER} \
    --mixed_precision=bf16 \
    "$(which lerobot-train)" \
    --dataset.repo_id=coport-uni/FR5_pick_red_colored_marker_to_box \
    --policy.type=pi05 \
    --policy.pretrained_path=models/pi05_base_v051compat \
    --policy.normalization_mapping='{"ACTION": "MEAN_STD", "STATE": "MEAN_STD", "VISUAL": "IDENTITY"}' \
    --policy.push_to_hub=false \
    --dataset.video_backend=pyav \
    --policy.device=cuda \
    --policy.compile_model=true \
    --policy.compile_mode=default \
    --policy.gradient_checkpointing=true \
    --policy.dtype=bfloat16 \
    --policy.freeze_vision_encoder=false \
    --policy.train_expert_only=false \
    --policy.optimizer_weight_decay=1e-10 \
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

# Per-GPU peak across all 4 columns; report the max and its percent of
# the 183359 MiB B200 capacity.
# `read` returns non-zero at EOF when awk emits no trailing newline;
# `|| true` keeps that from tripping `set -e` and skipping the summary.
read PEAK0 PEAK1 PEAK2 PEAK3 PEAKMAX < <(awk -F, '
    NR>1 {
        for (i=2; i<=5; i++) if ($i+0 > m[i]) m[i]=$i+0
    }
    END {
        mx=0; for (i=2; i<=5; i++) if (m[i]>mx) mx=m[i]
        printf "%d %d %d %d %d", m[2], m[3], m[4], m[5], mx
    }' "${VRAM_LOG}") || true
CAP_MIB=183359
PCT=$(awk "BEGIN{printf \"%.1f\", 100*${PEAKMAX}/${CAP_MIB}}")
PEAKMAX_GB=$(awk "BEGIN{printf \"%.2f\", ${PEAKMAX}/1024}")

OOM=""
if grep -qiE "out of memory|OutOfMemoryError|CUDA error: out of memory" \
        "${TRAIN_LOG}"; then
    OOM=" (OOM detected in log)"
fi

echo
echo "[probe] BATCH=${BATCH} done; exit=${EXIT_CODE}${OOM}"
echo "[probe] Peak per GPU (MiB): ${PEAK0} ${PEAK1} ${PEAK2} ${PEAK3}"
echo "[probe] Max peak: ${PEAKMAX} MiB (${PEAKMAX_GB} GB) = ${PCT}% of B200"

exit ${EXIT_CODE}
