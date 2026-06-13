#!/usr/bin/env bash
# probe_nccl_b200.sh -- Test whether a given NCCL transport config
# completes 4-GPU DDP on this B200 box (issue #30 forced
# NCCL_P2P_DISABLE=1 because P2P stalled the first allreduce inside a
# Docker image; on an NVLink/NVSwitch B200 node P2P may work and be
# faster). Reports: did the first allreduce/barrier clear (vs hang),
# and the steady per-step time.
#
# The caller exports the NCCL_* env to test BEFORE invoking; this
# script does not set any NCCL transport flag itself.
#
# Usage: NCCL_P2P_DISABLE=1 probe_nccl_b200.sh <label>
#   STEPS=<n>     step count (default 40)
#   BATCH=<n>     per-GPU batch (default 96; keep well under VRAM so the
#                 run is comm-bound, not memory-bound)
#   TIMEOUT=<s>   wall-clock kill (default 480); a hang on the first
#                 allreduce never returns, so a timeout == "this config
#                 hangs on B200".
#
# Outputs:
#   claude_test/probe_logs/nccl_b200_<label>.log

set -u
LABEL="${1:?Usage: $0 <label>}"
STEPS="${STEPS:-40}"
BATCH="${BATCH:-96}"
TIMEOUT="${TIMEOUT:-480}"
GPU_NUMBER=4

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
export PATH="${_conda_root}/envs/lerobot/bin:${PATH}"

# /tmp is noexec here; redirect torch.compile/triton caches to an
# exec-allowed dir (see probe_pi0_vram_b200.sh for the full reason).
_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export TORCHINDUCTOR_CACHE_DIR="${_repo_root}/.cache/torchinductor"
export TRITON_CACHE_DIR="${_repo_root}/.cache/triton"

# Surface NCCL transport selection and any P2P/SHM failures.
export NCCL_DEBUG="${NCCL_DEBUG:-WARN}"

JOB_NAME="fr5_pi0_nccl_probe_${LABEL}"
OUTPUT_DIR="outputs/train/${JOB_NAME}"
LOG_DIR="claude_test/probe_logs"
mkdir -p "${LOG_DIR}"
TRAIN_LOG="${LOG_DIR}/nccl_b200_${LABEL}.log"
rm -rf "${OUTPUT_DIR}"

echo "[nccl] label=${LABEL} batch=${BATCH} steps=${STEPS} timeout=${TIMEOUT}s"
echo "[nccl] NCCL_P2P_DISABLE=${NCCL_P2P_DISABLE:-unset} NCCL_SHM_DISABLE=${NCCL_SHM_DISABLE:-unset} NCCL_NVLS_ENABLE=${NCCL_NVLS_ENABLE:-unset}"

# `timeout` so a first-allreduce hang is caught instead of waiting
# forever. SIGKILL after a grace period if it ignores SIGTERM.
set +e
timeout -k 30 "${TIMEOUT}" accelerate launch \
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

# Classify the outcome.
RESULT="unknown"
if [ "${EXIT_CODE}" -eq 124 ] || [ "${EXIT_CODE}" -eq 137 ]; then
    RESULT="HANG/timeout (first allreduce never cleared)"
elif grep -qiE "out of memory|OutOfMemoryError" "${TRAIN_LOG}"; then
    RESULT="OOM (raise VRAM headroom / lower BATCH)"
elif grep -qiE "${STEPS}/${STEPS}|training has finished|Step ${STEPS}" \
        "${TRAIN_LOG}"; then
    RESULT="OK (completed ${STEPS} steps)"
elif [ "${EXIT_CODE}" -eq 0 ]; then
    RESULT="OK (exit 0)"
else
    RESULT="FAIL exit=${EXIT_CODE} (see log)"
fi

# Steady per-step time: last s/step reported by the tqdm bar.
SPS=$(grep -oE "[0-9]+\.[0-9]+s/step" "${TRAIN_LOG}" | tail -1)
[ -z "${SPS}" ] && SPS=$(grep -oE "[0-9]+\.[0-9]+it/s" "${TRAIN_LOG}" \
                          | tail -1)
# Any NCCL warning about falling back off P2P/SHM.
NCCL_WARN=$(grep -iE "NCCL WARN|failed|P2P.*disab|cannot|unable" \
            "${TRAIN_LOG}" | grep -i nccl | head -1)

echo
echo "[nccl] label=${LABEL} exit=${EXIT_CODE} -> ${RESULT}"
echo "[nccl] steady step time: ${SPS:-n/a}"
[ -n "${NCCL_WARN}" ] && echo "[nccl] NCCL note: ${NCCL_WARN}"

exit 0
