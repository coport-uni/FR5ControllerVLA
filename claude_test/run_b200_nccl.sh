#!/usr/bin/env bash
# One-off runner (issue #87): compare NCCL transport configs on 4xB200.
# Fixed batch so the runs are comm-bound; the question is which configs
# complete the first allreduce (vs hang per issue #30) and which is
# fastest. Dataset must already be cached (run the VRAM ladder first).
set -u
export BATCH="${BATCH:-96}"
export STEPS="${STEPS:-40}"
export TIMEOUT="${TIMEOUT:-480}"
RESULTS="claude_test/probe_logs/b200_nccl_results.txt"
echo "label  result  steady_step  ($(date))" > "${RESULTS}"

run_one () {
    local label="$1"; shift
    echo "===== NCCL config: ${label} ====="
    # Run in a subshell so each config's env stays isolated.
    out=$(env "$@" bash claude_test/probe_nccl_b200.sh "${label}" 2>&1)
    echo "${out}" | grep -E "^\[nccl\]"
    res=$(echo "${out}" | grep -E "exit=.*->" | sed 's/.*-> //')
    sps=$(echo "${out}" | grep -E "steady step time" | sed 's/.*: //')
    echo "${label}  ${res}  ${sps}" >> "${RESULTS}"
}

# 1) Current production default: P2P forced off.
run_one p2p_off  NCCL_P2P_DISABLE=1
# 2) P2P enabled (NVLink/NVSwitch) -- the config issue #30 banned.
run_one p2p_on   NCCL_P2P_DISABLE=0
# 3) Pure socket: P2P and SHM both off.
run_one p2p_shm_off  NCCL_P2P_DISABLE=1 NCCL_SHM_DISABLE=1

echo "===== nccl sweep done ====="
cat "${RESULTS}"
