#!/usr/bin/env bash
# One-off runner (issue #87): sweep per-GPU batch ascending on 4xB200,
# stop after the first OOM. Records a compact result table.
set -u
RESULTS="claude_test/probe_logs/b200_ladder_results.txt"
echo "batch  exit  peak_mib  pct  note  ($(date))" > "${RESULTS}"
for B in 176 192 208 224 240; do
    echo "===== probing batch ${B} ====="
    out=$(bash claude_test/probe_pi0_vram_b200.sh "${B}" 2>&1)
    echo "${out}" | tail -4
    summary=$(echo "${out}" | grep -E "\[probe\] (BATCH=${B} done|Max peak)")
    echo "${B}  ${summary}" >> "${RESULTS}"
    if echo "${out}" | grep -qiE "OOM detected|out of memory"; then
        echo "batch ${B} OOM -> stopping ladder"
        echo "STOP_AT_OOM batch=${B}" >> "${RESULTS}"
        break
    fi
done
echo "===== ladder done ====="
