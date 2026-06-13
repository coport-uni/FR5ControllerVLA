#!/usr/bin/env bash
# probe_pi0_vram_b200.sh -- B200 (4-GPU) variant of probe_pi0_vram.sh.
# Run a short Pi0 fine-tune at the given per-GPU batch_size on 4 x B200
# and report peak VRAM per GPU. Probe only; production isolated.
#
# Differences vs probe_pi0_vram.sh (2 x H200):
#   - num_processes=4 (matches the 4 x B200 production config).
#   - Samples all 4 GPUs (0,1,2,3), not just 0,1.
#   - Adds the miniforge3 conda root on this box to the discovery loop
#     and prepends the env bin to PATH (on this box `conda activate`
#     alone leaves bare `python` shadowed by /usr/bin/python).
#
# Usage: probe_pi0_vram_b200.sh <per_gpu_batch_size>
#   STEPS=<n>          override step count (default 50)
#   COMPILE_MODE=<m>   torch.compile mode (default "default"; pi0's own
#                      default "max-autotune" Triton-GEMM-autotunes and
#                      crashes with CUDA illegal-memory-access on this
#                      sm_100 box at per-GPU batch >= ~176)
#
# Outputs:
#   claude_test/probe_logs/vram_b200_b${BATCH}.csv  nvidia-smi 1 Hz
#   claude_test/probe_logs/train_b200_b${BATCH}.log accelerate stdout
#   outputs/train/fr5_pi0_vram_probe_b200_b${BATCH}/ cleaned each run
#
# See ToDo 2026-06-12 / GitHub issue #87 (follows #55).

set -u

BATCH="${1:?Usage: $0 <per_gpu_batch_size>}"
STEPS="${STEPS:-50}"
COMPILE_MODE="${COMPILE_MODE:-default}"
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

# NCCL: P2P must stay ENABLED on this bare-metal B200 NVLink/NVSwitch
# box. The inherited NCCL_P2P_DISABLE=1 (issue #30, a Docker-image
# workaround) HANGS the first allreduce here -- the NCCL sweep showed
# p2p_off=HANG, p2p_on=OK, and the first ladder run (p2p_off) died with
# CUDA illegal-memory-access. So we force P2P on. See issue #87.
export NCCL_P2P_DISABLE=0

JOB_NAME="fr5_pi0_vram_probe_b200_b${BATCH}"
OUTPUT_DIR="outputs/train/${JOB_NAME}"
LOG_DIR="claude_test/probe_logs"
mkdir -p "${LOG_DIR}"
VRAM_LOG="${LOG_DIR}/vram_b200_b${BATCH}.csv"
TRAIN_LOG="${LOG_DIR}/train_b200_b${BATCH}.log"

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
    --policy.type=pi0 \
    --policy.pretrained_path=models/pi0_base_v051compat \
    --policy.push_to_hub=false \
    --dataset.video_backend=pyav \
    --policy.device=cuda \
    --policy.compile_model=true \
    --policy.compile_mode="${COMPILE_MODE}" \
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

# Per-GPU peak across all 4 columns; report the max and its percent of
# the 183359 MiB B200 capacity.
read PEAK0 PEAK1 PEAK2 PEAK3 PEAKMAX < <(awk -F, '
    NR>1 {
        for (i=2; i<=5; i++) if ($i+0 > m[i]) m[i]=$i+0
    }
    END {
        mx=0; for (i=2; i<=5; i++) if (m[i]>mx) mx=m[i]
        printf "%d %d %d %d %d", m[2], m[3], m[4], m[5], mx
    }' "${VRAM_LOG}")
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
