#!/usr/bin/env bash
# Master #3 (issue #87): rerun the VRAM ladder with compile_mode=default
# (driver default now), after max-autotune Triton GEMM autotune was
# identified as the illegal-memory-access cause on sm_100.
# Output: claude_test/probe_logs/b200_digest3.txt
set -u
DIG=claude_test/probe_logs/b200_digest3.txt
: > "$DIG"
bash claude_test/run_b200_ladder.sh >> "$DIG" 2>&1
echo "" >> "$DIG"
echo "===== VRAM LADDER PEAKS v3 / P2P-on, compile_mode=default =====" >> "$DIG"
for B in 176 192 208 224 240; do
  log=claude_test/probe_logs/train_b200_b${B}.log
  csv=claude_test/probe_logs/vram_b200_b${B}.csv
  [ -f "$log" ] || { echo "batch=$B not run" >> "$DIG"; continue; }
  if grep -qiE "out of memory|OutOfMemoryError" "$log" 2>/dev/null; then oom="OOM"; else oom="-"; fi
  ima=$(grep -c "illegal memory access" "$log" 2>/dev/null)
  fin=$(grep -cE "End of training" "$log" 2>/dev/null)
  step=$(grep -oE "[0-9]+/50 \[" "$log" 2>/dev/null | tail -1)
  sps=$(grep -oE "[0-9]+\.[0-9]+s/step" "$log" 2>/dev/null | tail -1)
  peak=$(awk -F, 'NR>1{for(i=2;i<=5;i++) if($i+0>m)m=$i+0} END{printf "%d", m+0}' "$csv" 2>/dev/null)
  pct=$(awk "BEGIN{printf \"%.1f\", 100*${peak:-0}/183359}")
  echo "batch=$B peak=${peak}MiB (${pct}%) laststep=${step:-NA} sps=${sps:-NA} finished=$fin oom=$oom illegal=$ima" >> "$DIG"
done
echo "" >> "$DIG"; echo "ALL_DONE3" >> "$DIG"
