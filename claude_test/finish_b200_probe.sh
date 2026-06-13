#!/usr/bin/env bash
# Chained finisher (issue #87): wait for the VRAM ladder to finish,
# digest its peaks, then run the NCCL sweep, then digest that. One exit
# = everything ready. Output: claude_test/probe_logs/b200_digest.txt
set -u
DIG=claude_test/probe_logs/b200_digest.txt
: > "$DIG"

# 1) Wait for the ladder runner to exit.
while pgrep -f run_b200_ladder.sh >/dev/null; do sleep 20; done

echo "===== VRAM LADDER PEAKS (cap 183359 MiB) =====" >> "$DIG"
for B in 176 192 208 224 240; do
  log=claude_test/probe_logs/train_b200_b${B}.log
  csv=claude_test/probe_logs/vram_b200_b${B}.csv
  if grep -qiE "out of memory|OutOfMemoryError|CUDA error: out of memory" "$log" 2>/dev/null; then oom="OOM"; else oom="-"; fi
  reg=$(grep -c "relative_actions_processor" "$log" 2>/dev/null)
  fin=$(grep -cE "End of training|50/50" "$log" 2>/dev/null)
  peak=$(awk -F, 'NR>1{for(i=2;i<=5;i++) if($i+0>m)m=$i+0} END{printf "%d", m+0}' "$csv" 2>/dev/null)
  pct=$(awk "BEGIN{printf \"%.1f\", 100*${peak:-0}/183359}")
  echo "batch=$B peak=${peak}MiB (${pct}%) oom=$oom finished=$fin regerr=$reg" >> "$DIG"
done

# 2) NCCL sweep (GPUs now free).
echo "" >> "$DIG"
echo "===== NCCL SWEEP =====" >> "$DIG"
bash claude_test/run_b200_nccl.sh >> "$DIG" 2>&1

echo "" >> "$DIG"
echo "ALL_DONE" >> "$DIG"
