#!/usr/bin/env bash
# Master #2 (issue #87): after the first master (NCCL sweep) exits,
# rerun the VRAM ladder with P2P now ENABLED in the driver (p2p_off was
# proven to hang/illegal-access on this B200). Valid steady-state peaks.
# Output: claude_test/probe_logs/b200_digest2.txt
set -u
DIG=claude_test/probe_logs/b200_digest2.txt
: > "$DIG"
while pgrep -f master_b200.sh >/dev/null; do sleep 15; done
echo "first master done; rerunning ladder with P2P enabled" >> "$DIG"
bash claude_test/run_b200_ladder.sh >> "$DIG" 2>&1
echo "" >> "$DIG"
echo "===== VRAM LADDER PEAKS v2 / P2P-on (cap 183359 MiB) =====" >> "$DIG"
for B in 176 192 208 224 240; do
  log=claude_test/probe_logs/train_b200_b${B}.log
  csv=claude_test/probe_logs/vram_b200_b${B}.csv
  if grep -qiE "out of memory|OutOfMemoryError" "$log" 2>/dev/null; then oom="OOM"; else oom="-"; fi
  ima=$(grep -c "illegal memory access" "$log" 2>/dev/null)
  fin=$(grep -cE "End of training" "$log" 2>/dev/null)
  step=$(grep -oE "[0-9]+/50 \[" "$log" 2>/dev/null | tail -1)
  peak=$(awk -F, 'NR>1{for(i=2;i<=5;i++) if($i+0>m)m=$i+0} END{printf "%d", m+0}' "$csv" 2>/dev/null)
  pct=$(awk "BEGIN{printf \"%.1f\", 100*${peak:-0}/183359}")
  echo "batch=$B peak=${peak}MiB (${pct}%) laststep=${step:-NA} finished=$fin oom=$oom illegal_access=$ima" >> "$DIG"
done
echo "" >> "$DIG"; echo "ALL_DONE2" >> "$DIG"
