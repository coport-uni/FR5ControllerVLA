#!/usr/bin/env bash
# run_b200_pi05_ladder.sh -- Issue #89 one-off runner. Sweeps per-GPU
# pi05 batch sizes on 4 x B200 via probe_pi05_vram_b200.sh, stops after
# the first OOM, and writes a compact result table.

set -u
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

RESULTS="claude_test/probe_logs/pi05_b200_ladder_results.txt"
LADDER=(152 168 176 184)

echo "per_gpu_batch | effective(x4) | max_peak_MiB | pct | result" \
    > "${RESULTS}"

for b in "${LADDER[@]}"; do
    echo "=== ladder rung: per-GPU batch ${b} ==="
    # Run the probe directly (no stdout capture); rely on its exit code.
    bash claude_test/probe_pi05_vram_b200.sh "${b}"
    rc=$?
    csv="claude_test/probe_logs/vram_pi05_b200_b${b}.csv"
    log="claude_test/probe_logs/train_pi05_b200_b${b}.log"
    read peak pct < <(awk -F, 'NR>1{for(i=2;i<=5;i++) if($i+0>m[i]) m[i]=$i+0}
        END{mx=0; for(i=2;i<=5;i++) if(m[i]>mx) mx=m[i];
            printf "%d %.1f", mx, 100*mx/183359}' "${csv}")
    # A rung counts as OK only if the probe exited 0 AND the train log
    # shows the loop actually advanced (a step line), not just loaded.
    stepped="no"
    if grep -qiE "s/step|Training: *100%|Checkpoint policy after step" \
            "${log}"; then
        stepped="yes"
    fi
    if [ "${rc}" -eq 0 ] && [ "${stepped}" = "yes" ] \
       && ! grep -qiE "out of memory|OutOfMemoryError" "${log}"; then
        echo "${b} | $((b*4)) | ${peak:-NA} | ${pct:-NA} | OK" \
            >> "${RESULTS}"
    else
        reason="FAIL(rc=${rc},stepped=${stepped})"
        grep -qiE "out of memory|OutOfMemoryError" "${log}" && reason="OOM"
        echo "${b} | $((b*4)) | ${peak:-NA} | ${pct:-NA} | ${reason}" \
            >> "${RESULTS}"
        echo "[ladder] ${reason} at batch ${b}; stopping."
        break
    fi
done

echo "=== ladder done ==="
cat "${RESULTS}"
