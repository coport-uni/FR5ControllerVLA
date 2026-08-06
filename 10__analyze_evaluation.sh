#!/usr/bin/env bash
# Measure per-attempt elapsed time and success from an evaluation
# recording, using the video file alone.
#
# Usage:
#   ./10__analyze_evaluation.sh outputs/evaluation/task2_act_100.webm
#
# The three stages are run separately on purpose. Check
# analysis/<stem>/timeline.png after `segment`; if the boundaries are
# wrong, retune with --operator-k / --min-duration and re-run `segment`
# alone, without paying to re-render every contact sheet.

set -euo pipefail

VIDEO="${1:?usage: $0 <video.webm> [stage]}"
STAGE="${2:-all}"
RUN=(python -m lerobot.scripts.eval_video_report)

case "${STAGE}" in
    segment | sheets)
        "${RUN[@]}" "${STAGE}" "${VIDEO}"
        ;;
    report)
        STEM="$(basename "${VIDEO%.*}")"
        DIR="$(dirname "${VIDEO}")/analysis/${STEM}"
        "${RUN[@]}" report --labels "${DIR}/labels.csv"
        ;;
    all)
        "${RUN[@]}" segment "${VIDEO}"
        "${RUN[@]}" sheets "${VIDEO}"
        echo
        echo "Now review analysis/<stem>/timeline.png and the contact"
        echo "sheets, fill success/failure_mode/notes in labels.csv,"
        echo "then run: $0 ${VIDEO} report"
        ;;
    *)
        echo "unknown stage: ${STAGE} (segment|sheets|report|all)" >&2
        exit 1
        ;;
esac
