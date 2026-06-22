#!/usr/bin/env bash
# Master finisher (issue #87): run the whole remaining GPU pipeline
# unattended once the compile smoke test passes. One ALL_DONE at the end.
# Output: claude_test/probe_logs/b200_digest.txt
set -u
DIG=claude_test/probe_logs/b200_digest.txt
: > "$DIG"

# 0) Wait for the in-flight compile smoke test to finish.
while pgrep -f "_pi0_compilecheck" >/dev/null; do sleep 15; done
if ! grep -qE "End of training" claude_test/probe_logs/compilecheck.log 2>/dev/null \
   || [ "$(grep -c 'failed to map segment' claude_test/probe_logs/compilecheck.log)" != "0" ]; then
    echo "SMOKE_FAILED -- aborting; see compilecheck.log" >> "$DIG"
    echo "ALL_DONE" >> "$DIG"
    exit 1
fi
echo "smoke test passed (compile cache redirect OK)" >> "$DIG"

# 1) VRAM ladder.
bash claude_test/run_b200_ladder.sh >> "$DIG" 2>&1

echo "" >> "$DIG"
echo "===== VRAM LADDER PEAKS (cap 183359 MiB) =====" >> "$DIG"
for B in 176 192 208 224 240; do
  log=claude_test/probe_logs/train_b200_b${B}.log
  csv=claude_test/probe_logs/vram_b200_b${B}.csv
  if grep -qiE "out of memory|OutOfMemoryError|CUDA error: out of memory" "$log" 2>/dev/null; then oom="OOM"; else oom="-"; fi
  fin=$(grep -cE "End of training" "$log" 2>/dev/null)
  peak=$(awk -F, 'NR>1{for(i=2;i<=5;i++) if($i+0>m)m=$i+0} END{printf "%d", m+0}' "$csv" 2>/dev/null)
  pct=$(awk "BEGIN{printf \"%.1f\", 100*${peak:-0}/183359}")
  echo "batch=$B peak=${peak}MiB (${pct}%) oom=$oom finished=$fin" >> "$DIG"
done

# 2) NCCL sweep.
echo "" >> "$DIG"
echo "===== NCCL SWEEP =====" >> "$DIG"
bash claude_test/run_b200_nccl.sh >> "$DIG" 2>&1

echo "" >> "$DIG"
echo "ALL_DONE" >> "$DIG"
