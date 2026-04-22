#!/usr/bin/env bash
# Benchmark: time to STEPS for 1-GPU vs 2-GPU `accelerate launch` on
# the FR5 dataset, parameterised by policy (ACT, Pi0, Pi0.5).
# Weak scaling (per-rank batch as per --batch arg). 5 trials per group.
#
# Usage:
#   bash claude_test/bench_accelerate_1v2gpu.sh \
#       --model {act|pi0|pi05} [--steps N] [--batch N] [--trials N] \
#       [--compile {true|false}] [--gradient-checkpointing {true|false}]
#
# Output (appended): claude_test/accelerate_mgpu_evidence/bench_1v2gpu.csv
# Per-trial logs:    /tmp/bench_act_1v2gpu/log_<model>_<n>gpu_t<trial>.log
#
# Schema:
#   model,trial,num_gpus,target_steps,batch_size,
#   wall_clock_s,training_loop_s,final_loss,timestamp
#
# See gh issue #34 and #30 for context.

set -uo pipefail

REPO_ROOT="/workspace/VLARelated/FR5ControllerVLA"
ENV_BIN="/opt/conda/envs/lerobot/bin"
ACCELERATE="${ENV_BIN}/accelerate"
LEROBOT_TRAIN="${ENV_BIN}/lerobot-train"

DATASET="coport-uni/FR5_pick_red_colored_marker_to_box"
WORK="/tmp/bench_act_1v2gpu"
OUT_DIR="${REPO_ROOT}/claude_test/accelerate_mgpu_evidence"
CSV="${OUT_DIR}/bench_1v2gpu.csv"

# Defaults
MODEL="act"
STEPS=100
BATCH=64
TRIALS=5
COMPILE="false"
GRAD_CKPT="false"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --model) MODEL="$2"; shift 2 ;;
        --steps) STEPS="$2"; shift 2 ;;
        --batch) BATCH="$2"; shift 2 ;;
        --trials) TRIALS="$2"; shift 2 ;;
        --compile) COMPILE="$2"; shift 2 ;;
        --gradient-checkpointing) GRAD_CKPT="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

mkdir -p "${WORK}" "${OUT_DIR}"
if [[ ! -f "${CSV}" ]]; then
    echo "model,trial,num_gpus,target_steps,batch_size,wall_clock_s,training_loop_s,final_loss,timestamp" > "${CSV}"
fi

# Build the policy-specific flags. Pi0 / Pi0.5 need a pretrained_path and a
# few extra knobs; ACT needs nothing beyond `--policy.type=act`.
build_policy_flags() {
    case "${MODEL}" in
        act)
            echo --policy.type=act
            ;;
        pi0)
            echo --policy.type=pi0 \
                 --policy.pretrained_path=lerobot/pi0_base \
                 --policy.compile_model="${COMPILE}" \
                 --policy.gradient_checkpointing="${GRAD_CKPT}" \
                 --policy.dtype=bfloat16 \
                 --policy.freeze_vision_encoder=false \
                 --policy.train_expert_only=false
            ;;
        pi05)
            echo --policy.type=pi05 \
                 --policy.pretrained_path=lerobot/pi05_base \
                 --policy.compile_model="${COMPILE}" \
                 --policy.gradient_checkpointing="${GRAD_CKPT}" \
                 --policy.dtype=bfloat16 \
                 --policy.freeze_vision_encoder=false \
                 --policy.train_expert_only=false
            ;;
        *) echo "unknown model: ${MODEL}" >&2; exit 2 ;;
    esac
}

# Parse training-loop time and final loss from a per-trial log file.
# `INFO ...` log lines are concatenated to the previous tqdm bar (no
# leading newline), so we have to use `grep -oE "INFO ... "` to isolate
# the structured substring before reading its date/time fields.
parse_log() {
    local log="$1"
    local target="$2"
    local start_line step_line start_iso step_iso start_epoch step_epoch loss
    start_line=$(grep -oE "INFO [0-9-]+ [0-9:]+ [a-zA-Z_]+\.py:[0-9]+ Start offline training" \
                 "${log}" | head -1 || true)
    step_line=$(grep -oE "INFO [0-9-]+ [0-9:]+ [a-zA-Z_]+\.py:[0-9]+ step:${target} [^[:cntrl:]]*" \
                "${log}" | head -1 || true)
    if [[ -z "${start_line}" || -z "${step_line}" ]]; then
        echo "NA NA"; return
    fi
    start_iso=$(echo "${start_line}" | awk '{print $2"T"$3}')
    step_iso=$(echo "${step_line}" | awk '{print $2"T"$3}')
    start_epoch=$(date -d "${start_iso}" +%s 2>/dev/null || echo "")
    step_epoch=$(date -d "${step_iso}" +%s 2>/dev/null || echo "")
    loss=$(echo "${step_line}" | grep -oE "loss:[0-9.]+" | head -1 | cut -d: -f2)
    if [[ -z "${start_epoch}" || -z "${step_epoch}" ]]; then
        echo "NA ${loss:-NA}"; return
    fi
    echo "$((step_epoch - start_epoch)) ${loss:-NA}"
}

run_one() {
    local n_gpu="$1"
    local trial="$2"
    local outdir="${WORK}/run_${MODEL}_${n_gpu}gpu_t${trial}"
    local log="${WORK}/log_${MODEL}_${n_gpu}gpu_t${trial}.log"
    rm -rf "${outdir}"

    local accelerate_flags=()
    if [[ "${n_gpu}" -eq 2 ]]; then
        accelerate_flags=(--multi_gpu --num_processes=2)
        export NCCL_P2P_DISABLE=1 NCCL_SHM_DISABLE=1
    else
        accelerate_flags=(--num_processes=1)
        unset NCCL_P2P_DISABLE NCCL_SHM_DISABLE 2>/dev/null || true
    fi
    accelerate_flags+=(--mixed_precision=bf16)

    local policy_flags
    read -r -a policy_flags <<<"$(build_policy_flags)"

    local t0 t1 wall
    t0=$(date +%s.%N)
    PYTHONUNBUFFERED=1 "${ACCELERATE}" launch \
        "${accelerate_flags[@]}" \
        "${LEROBOT_TRAIN}" \
        --dataset.repo_id="${DATASET}" \
        --dataset.video_backend=pyav \
        "${policy_flags[@]}" \
        --policy.device=cuda \
        --policy.push_to_hub=false \
        --output_dir="${outdir}" \
        --job_name="bench_${MODEL}_${n_gpu}gpu_t${trial}" \
        --wandb.enable=false \
        --num_workers=4 \
        --batch_size="${BATCH}" \
        --steps="${STEPS}" \
        --save_checkpoint=false \
        --log_freq=20 \
        --resume=false \
        > "${log}" 2>&1
    local rc=$?
    t1=$(date +%s.%N)
    wall=$(awk -v a="${t0}" -v b="${t1}" 'BEGIN{printf "%.2f", b-a}')

    if [[ ${rc} -ne 0 ]]; then
        echo "[FAIL ${MODEL} trial=${trial} ${n_gpu}gpu] exit=${rc} log=${log}"
        echo "${MODEL},${trial},${n_gpu},${STEPS},${BATCH},${wall},NA,NA,$(date -Iseconds)" >> "${CSV}"
        return
    fi

    local parsed loop_s loss
    parsed=$(parse_log "${log}" "${STEPS}")
    loop_s=$(echo "${parsed}" | awk '{print $1}')
    loss=$(echo "${parsed}" | awk '{print $2}')
    echo "[OK   ${MODEL} trial=${trial} ${n_gpu}gpu] wall=${wall}s loop=${loop_s}s loss=${loss}"
    echo "${MODEL},${trial},${n_gpu},${STEPS},${BATCH},${wall},${loop_s},${loss},$(date -Iseconds)" >> "${CSV}"
}

echo "=== model=${MODEL} steps=${STEPS} batch=${BATCH} trials=${TRIALS} compile=${COMPILE} gradckpt=${GRAD_CKPT} ==="

for trial in $(seq 1 "${TRIALS}"); do
    run_one 1 "${trial}"
done
for trial in $(seq 1 "${TRIALS}"); do
    run_one 2 "${trial}"
done

echo
echo "=== summary for model=${MODEL} (${CSV}) ==="
awk -F, -v M="${MODEL}" -v S="${STEPS}" 'NR>1 && $1==M && $4==S && $7 != "NA" {
    k=$3; w[k]+=$6; w2[k]+=$6*$6; l[k]+=$7; l2[k]+=$7*$7; n[k]++; b=$5
}
END {
    for (k=1; k<=2; k++) if (n[k]) {
        wm=w[k]/n[k]; lm=l[k]/n[k]
        ws=sqrt((w2[k]-n[k]*wm*wm)/n[k]); ls=sqrt((l2[k]-n[k]*lm*lm)/n[k])
        printf "%-5s steps=%d GPUs=%d n=%d wall=%.2f+/-%.2fs loop=%.2f+/-%.2fs samples/s=%.2f\n", \
            M, S, k, n[k], wm, ws, lm, ls, S*b*k/lm
    }
}' "${CSV}"
