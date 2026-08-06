#!/usr/bin/env python

# Copyright 2026 APPEAL Automation team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.

"""Per-attempt timing and success reporting for evaluation videos.

The evaluation recordings under ``outputs/evaluation/`` are the only
record of how a policy behaved: no robot or server log is kept
alongside them. This tool derives the attempt boundaries, and thus
the elapsed time of every attempt, from the pixels alone.

The camera is fixed, so an attempt can be found by watching the arm
leave its parked home pose and come back. That interval is exactly
what a stopwatch would measure, and it also sidesteps the operator:
the props are reset while the arm is parked, so a reset never lands
inside an attempt and never has to be detected in its own right.
Motion energy alone is not enough -- a policy that stalls mid-attempt
would be split in two, and a bystander at the laptop keeps the bottom
of the frame busy from beginning to end.

Every trip away from home counts as an attempt, including the ones
that end in seconds because the policy turned round and drove back --
the operators log that as its own failure mode. Such spans are only
flagged, never dropped.

Elapsed time is recorded, never scored. Nothing here compares a
duration against a threshold or turns one into a score.

Usage::

    python -m lerobot.scripts.eval_video_report segment VIDEO
    python -m lerobot.scripts.eval_video_report sheets VIDEO
    python -m lerobot.scripts.eval_video_report report --labels CSV
"""

from __future__ import annotations

import argparse
import csv
import re
import statistics
import subprocess
import sys
from dataclasses import dataclass, fields
from pathlib import Path

import numpy as np

# Decoding is done on a heavily downscaled grayscale stream: the
# signal of interest is where motion happens, not what it looks like.
_sample_fps = 4
_signal_width = 160
_signal_height = 120

# Motion is smoothed over this window so that a single dropped frame
# or a compression artefact cannot open or close an attempt.
_smooth_s = 2.0

# Regions of interest are (x0, y0, x1, y1) fractions of the frame.
#
# The detection ROI frames the arm. Per-pixel motion maps over one
# recording per task show the three camera framings differ less than
# they look: in all of them the arm sweeps a vertical band around
# x 0.33-0.58, and all of them have an operator hotspot at x > 0.90,
# y > 0.68 that has to stay outside the ROI. Sweeping four candidate
# ROIs over the three tasks changed no attempt count at all, so one
# band serves every task and the value below is deliberately shared.
_workspace_roi = (0.28, 0.03, 0.62, 0.92)

# The contact-sheet crop is what genuinely varies per task, because it
# has to frame the props rather than the arm, and the props sit in a
# different place in each scene. At 640x480 they are small, so these
# crops are tight -- a wider one leaves the gripper too coarse to tell
# a grasp from a near miss.
_sheet_rois = {
    # Bottle on its pedestal and the round target it must be moved to.
    "task1": (0.24, 0.42, 0.62, 0.98),
    # Source and destination bottles, sitting higher up the bench.
    "task2": (0.18, 0.42, 0.60, 0.92),
    # Silver air valve and the fixtures either side of it.
    "task3": (0.28, 0.40, 0.66, 0.92),
}
_default_sheet_roi = (0.20, 0.40, 0.64, 0.96)

# A frame is "still" if its motion is below this percentile. Half is a
# deliberate over-estimate: the home template is refined afterwards, so
# it only has to be a superset of the frames where nothing moves.
_still_percentile = 50.0

# Spans shorter than this are camera noise, not arm movements at all.
_min_span_s = 5.0

# Some rollouts end within seconds: the policy leaves home, thinks
# better of it and drives straight back. The operators log this as its
# own failure mode ("갑자기 원점으로 돌아감"), so these count as
# attempts like any other -- they are flagged only to draw the
# reviewer's eye, never to drop them from the report. An earlier
# version treated them as noise and silently lost six of eleven
# attempts on task2_act_100.
#
# The cut-off is per task because a normal rollout lasts very
# different amounts of time in each: task2 rollouts run past a minute,
# while task1 and task3 rollouts finish in twenty-odd seconds, so the
# same threshold would flag their genuine attempts.
_quick_return_max_s = {"task1": 15.0, "task2": 20.0, "task3": 15.0}
_default_quick_return_max_s = 15.0

# A trial ("회차") in the evaluation protocol is two consecutive
# attempts, so attempts are paired when trial numbers are assigned.
_attempts_per_trial = 2

# Contact-sheet geometry. Sixteen keyframes in a 4x4 grid at this tile
# size stayed legible enough to tell a grasp from a near miss.
_keyframe_count = 16
_sheet_columns = 4
_sheet_rows = 4
_tile_width = 460
_tile_height = 440

_label_columns = ("success", "failure_mode", "notes")
_pending_note = "판정 보류 -- 사람 확인 필요"

_filename_pattern = re.compile(r"^task(?P<task>\d+)_(?P<model>[A-Za-z0-9]+)_(?P<dataset>\d+)$")


@dataclass
class Attempt:
    """One policy rollout located inside a recording.

    Attributes are ordered to match the CSV column order, which the
    label and report stages both rely on.
    """

    video: str
    task: str
    model: str
    dataset_size: str
    kind: str
    trial_index: int
    attempt_in_trial: int
    attempt_index: int
    start_s: float
    end_s: float
    duration_s: float
    success: str = ""
    failure_mode: str = ""
    notes: str = ""


def parse_roi(text: str) -> tuple[float, float, float, float]:
    """Parse an ``x0,y0,x1,y1`` region of interest.

    Args:
        text: Four comma-separated fractions of the frame size.

    Returns:
        The region as a tuple of floats.

    Raises:
        argparse.ArgumentTypeError: If the value is malformed or the
            corners are not in increasing order.
    """
    try:
        parts = tuple(float(value) for value in text.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"not numeric: {text}") from error
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(f"need 4 values: {text}")
    x0, y0, x1, y1 = parts
    if not (0.0 <= x0 < x1 <= 1.0 and 0.0 <= y0 < y1 <= 1.0):
        raise argparse.ArgumentTypeError(f"corners out of order: {text}")
    return x0, y0, x1, y1


def read_gray_frames(video_path: Path, sample_fps: int) -> np.ndarray:
    """Decode a video into a downscaled grayscale frame stack.

    A single ffmpeg pipe is used rather than per-frame seeking: these
    recordings are VP8 with sparse keyframes, where random access is
    both slow and imprecise.

    Args:
        video_path: Recording to decode.
        sample_fps: Frames per second to sample.

    Returns:
        Array of shape ``(n_frames, height, width)``.

    Raises:
        RuntimeError: If ffmpeg fails or yields no usable frames.
    """
    command = [
        "ffmpeg",
        "-v",
        "error",
        "-i",
        str(video_path),
        "-vf",
        f"fps={sample_fps},scale={_signal_width}:{_signal_height},format=gray",
        "-f",
        "rawvideo",
        "-",
    ]
    result = subprocess.run(command, capture_output=True, check=False)
    if result.returncode != 0:
        message = result.stderr.decode("utf-8", "replace").strip()
        raise RuntimeError(f"ffmpeg failed on {video_path.name}: {message}")

    pixels_per_frame = _signal_width * _signal_height
    buffer = np.frombuffer(result.stdout, dtype=np.uint8)
    n_frames = len(buffer) // pixels_per_frame
    if n_frames < 2:
        raise RuntimeError(f"decoded {n_frames} frames from {video_path}")
    frames = buffer[: n_frames * pixels_per_frame]
    return frames.reshape(n_frames, _signal_height, _signal_width)


def crop_to_roi(frames: np.ndarray, roi: tuple[float, float, float, float]) -> np.ndarray:
    """Crop a frame stack to a region given as frame fractions."""
    x0, y0, x1, y1 = roi
    height, width = frames.shape[1:]
    col_start, col_stop = int(x0 * width), max(int(x1 * width), int(x0 * width) + 1)
    row_start, row_stop = int(y0 * height), max(int(y1 * height), int(y0 * height) + 1)
    return frames[:, row_start:row_stop, col_start:col_stop]


def measure_motion(frames: np.ndarray) -> np.ndarray:
    """Return per-frame motion as the mean absolute frame difference.

    The last value is repeated so that the result lines up with the
    frame stack rather than being one sample shorter.
    """
    steps = np.abs(np.diff(frames.astype(np.int16), axis=0)).mean(axis=(1, 2))
    return np.append(steps, steps[-1] if len(steps) else 0.0)


def smooth(signal: np.ndarray, sample_fps: int) -> np.ndarray:
    """Apply a centred moving average of ``_smooth_s`` seconds."""
    width = max(int(round(_smooth_s * sample_fps)), 1)
    kernel = np.ones(width) / width
    return np.convolve(signal, kernel, mode="same")


def split_threshold(signal: np.ndarray) -> float:
    """Return Otsu's threshold between the two modes of a signal.

    The home-distance signal is strongly bimodal -- the arm is either
    parked at home or out working -- but the absolute levels drift
    between recordings and even within one, as the operator shifts the
    props around. Maximising between-class variance finds the split
    from the data instead of pinning it to a constant that would only
    suit the video it was tuned on.

    Args:
        signal: Values to split.

    Returns:
        The threshold maximising between-class variance.
    """
    counts, edges = np.histogram(signal, bins=256)
    centres = (edges[:-1] + edges[1:]) / 2
    weights = counts.cumsum()
    total = weights[-1]
    if total == 0:
        return float(signal.mean())
    moments = (counts * centres).cumsum()
    # Guard the empty-class ends, where the variance is undefined.
    low_weight = weights[:-1]
    high_weight = total - low_weight
    valid = (low_weight > 0) & (high_weight > 0)
    if not valid.any():
        return float(signal.mean())
    low_mean = np.divide(moments[:-1], low_weight, where=valid, out=np.zeros_like(moments[:-1]))
    high_mean = np.divide(
        moments[-1] - moments[:-1], high_weight, where=valid, out=np.zeros_like(moments[:-1])
    )
    variance = low_weight * high_weight * (low_mean - high_mean) ** 2
    variance[~valid] = -1.0
    return float(centres[int(np.argmax(variance))])


def find_home_template(frames: np.ndarray, motion: np.ndarray) -> np.ndarray:
    """Build a reference image of the robot parked in its home pose.

    The home pose is whatever the scene looks like most often while
    nothing is moving: the arm returns there after every attempt and
    waits for the operator to reset the props. Taking a plain median
    over all frames does not work -- it blends the home pose with the
    working poses and blunts the contrast the segmentation depends on.
    So the still frames are split once by their distance to the median
    still frame, and only the tight cluster is averaged.

    Args:
        frames: Grayscale frame stack, already cropped to the ROI.
        motion: Per-frame motion from :func:`measure_motion`.

    Returns:
        A float image with the same height and width as ``frames``.
    """
    still = frames[motion < np.percentile(motion, _still_percentile)]
    if len(still) == 0:
        return frames.astype(np.float32).mean(axis=0)
    flat = still.reshape(len(still), -1).astype(np.float32)
    seed = np.median(flat, axis=0)
    spread = np.abs(flat - seed).mean(axis=1)
    core = flat[spread <= split_threshold(spread)]
    if len(core) == 0:
        core = flat
    return core.mean(axis=0).reshape(still.shape[1:])


def measure_home_distance(frames: np.ndarray, template: np.ndarray) -> np.ndarray:
    """Return each frame's mean absolute difference from the template."""
    flat = frames.reshape(len(frames), -1).astype(np.float32)
    return np.abs(flat - template.reshape(-1)).mean(axis=1)


def find_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """Return ``[start, stop)`` index pairs for each run of ``True``."""
    runs: list[tuple[int, int]] = []
    index = 0
    while index < len(mask):
        if not mask[index]:
            index += 1
            continue
        stop = index
        while stop < len(mask) and mask[stop]:
            stop += 1
        runs.append((index, stop))
        index = stop
    return runs


def find_attempts(
    home_distance: np.ndarray,
    sample_fps: int,
    min_attempt_s: float,
) -> tuple[list[tuple[float, float]], float]:
    """Locate attempts as the spans where the arm is away from home.

    An attempt starts when the arm leaves its parked pose and ends when
    it comes back, which is exactly the interval a stopwatch would
    measure. Bounding attempts this way also sidesteps the operator
    entirely: the props are reset while the arm is parked, so the reset
    falls outside every attempt without having to be detected.

    Args:
        home_distance: Smoothed distance to the home template.
        sample_fps: Sampling rate of the signal.
        min_attempt_s: Shortest span accepted as an attempt.

    Returns:
        The ``(start_s, end_s)`` pairs in recording time, and the
        threshold used, for reporting on the timeline plot.
    """
    threshold = split_threshold(home_distance)
    min_samples = max(int(round(min_attempt_s * sample_fps)), 1)
    spans = [
        (start / sample_fps, stop / sample_fps)
        for start, stop in find_runs(home_distance > threshold)
        if stop - start >= min_samples
    ]
    return spans, threshold


def resolve_quick_return_max(task: str, override: float | None) -> float:
    """Return the quick-return cut-off for a task.

    Args:
        task: Task identifier such as ``"task2"``.
        override: Explicit cut-off from the CLI, or ``None``.

    Returns:
        The override when given, otherwise the task's preset, falling
        back to the shortest preset for an unrecognised task so that
        nothing is flagged that a known task would not have flagged.
    """
    if override is not None:
        return override
    return _quick_return_max_s.get(task, _default_quick_return_max_s)


def classify_spans(spans: list[tuple[float, float]], quick_return_max_s: float) -> list[str]:
    """Flag each span ``"rollout"`` or ``"quick_return"`` by duration.

    Both are attempts and both are counted. The flag exists because a
    rollout that ends in seconds nearly always ended the same way --
    the arm turned round and drove back to home -- and saying so up
    front saves the reviewer opening the contact sheet to find out.

    Args:
        spans: ``(start_s, end_s)`` pairs.
        quick_return_max_s: Longest span still called a quick return.

    Returns:
        One flag per span.
    """
    return ["quick_return" if end - start <= quick_return_max_s else "rollout" for start, end in spans]


def describe_video(video_path: Path) -> tuple[str, str, str]:
    """Split ``taskN_model_size`` metadata out of a filename.

    Args:
        video_path: Recording whose stem encodes the condition.

    Returns:
        ``(task, model, dataset_size)``. Unrecognised names yield the
        stem as the task and empty strings for the rest, so that an
        ad-hoc recording still produces a usable report.
    """
    match = _filename_pattern.match(video_path.stem)
    if match is None:
        return video_path.stem, "", ""
    return (
        f"task{match['task']}",
        match["model"],
        match["dataset"],
    )


def build_attempts(video_path: Path, spans: list[tuple[float, float]], kinds: list[str]) -> list[Attempt]:
    """Attach condition metadata and trial numbering to raw spans.

    Every span away from home is an attempt, however brief. Attempts
    are then paired into trials in recording order, following the
    evaluation protocol where one 회차 is two consecutive tries.
    """
    task, model, dataset_size = describe_video(video_path)
    attempts = []
    for position, ((start_s, end_s), kind) in enumerate(zip(spans, kinds, strict=True)):
        attempts.append(
            Attempt(
                video=video_path.name,
                task=task,
                model=model,
                dataset_size=dataset_size,
                kind=kind,
                trial_index=position // _attempts_per_trial + 1,
                attempt_in_trial=position % _attempts_per_trial + 1,
                attempt_index=position + 1,
                start_s=round(start_s, 2),
                end_s=round(end_s, 2),
                duration_s=round(end_s - start_s, 2),
            )
        )
    return attempts


def write_attempts(attempts: list[Attempt], csv_path: Path) -> None:
    """Write attempts to CSV, one row each, header included."""
    columns = [field.name for field in fields(Attempt)]
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for attempt in attempts:
            writer.writerow({name: getattr(attempt, name) for name in columns})


def read_attempts(csv_path: Path) -> list[Attempt]:
    """Read attempts back from a CSV written by :func:`write_attempts`.

    Args:
        csv_path: File to read.

    Returns:
        The attempts in file order.

    Raises:
        ValueError: If a required column is missing.
    """
    numeric = {"trial_index", "attempt_in_trial", "attempt_index"}
    with csv_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    attempts = []
    for row in rows:
        values = {}
        for field in fields(Attempt):
            if field.name not in row:
                raise ValueError(f"{csv_path}: missing column {field.name}")
            raw = (row[field.name] or "").strip()
            if field.name in numeric:
                values[field.name] = int(raw)
            elif field.name.endswith("_s"):
                values[field.name] = float(raw)
            else:
                values[field.name] = raw
        attempts.append(Attempt(**values))
    return attempts


def analysis_dir(video_path: Path, out_root: Path | None) -> Path:
    """Return the per-video output directory, creating it if needed."""
    root = out_root or video_path.parent / "analysis"
    directory = root / video_path.stem
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def render_timeline(
    home_distance: np.ndarray,
    motion: np.ndarray,
    threshold: float,
    attempts: list[Attempt],
    sample_fps: int,
    png_path: Path,
) -> None:
    """Plot the home distance and motion with the attempts shaded.

    This is the fastest way to catch a mis-split: a reset counted as
    an attempt, or one attempt reported as two. The home distance
    should read as a square wave, and every raised section should be
    shaded.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    seconds = np.arange(len(home_distance)) / sample_fps
    figure, axes = plt.subplots(2, 1, figsize=(18, 6), sharex=True)
    axes[0].plot(seconds, home_distance, linewidth=0.8, color="tab:blue")
    axes[0].axhline(
        threshold,
        color="tab:red",
        linewidth=0.8,
        label=f"away-from-home threshold = {threshold:.2f}",
    )
    axes[0].set_ylabel("distance to home pose")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[1].plot(seconds, motion, linewidth=0.8, color="tab:gray")
    axes[1].set_ylabel("motion energy")
    axes[1].set_xlabel("recording time (s)")

    for axis in axes:
        for attempt in attempts:
            axis.axvspan(attempt.start_s, attempt.end_s, alpha=0.18, color="tab:green")
        axis.grid(alpha=0.3)
    for attempt in attempts:
        axes[0].text(
            (attempt.start_s + attempt.end_s) / 2,
            axes[0].get_ylim()[1],
            str(attempt.attempt_index),
            ha="center",
            va="top",
            fontsize=8,
        )
    axes[0].set_title(f"{png_path.parent.name} -- {len(attempts)} attempts")
    figure.tight_layout()
    figure.savefig(png_path, dpi=110)
    plt.close(figure)


def render_contact_sheet(
    video_path: Path,
    attempt: Attempt,
    roi: tuple[float, float, float, float],
    keyframes: int,
    jpg_path: Path,
) -> None:
    """Render one attempt as a tiled, timestamped keyframe sheet.

    Args:
        video_path: Source recording.
        attempt: Attempt to illustrate.
        roi: Workspace region to crop to, as frame fractions.
        keyframes: Number of frames to sample across the attempt.
        jpg_path: Destination image.

    Raises:
        RuntimeError: If ffmpeg fails to produce the sheet.
    """
    duration = max(attempt.duration_s, 1.0 / _sample_fps)
    step = duration / keyframes
    x0, y0, x1, y1 = roi
    crop = f"crop=iw*{x1 - x0:.4f}:ih*{y1 - y0:.4f}:iw*{x0:.4f}:ih*{y0:.4f}"
    # The label is the absolute position in the recording, so a
    # reviewer can scrub straight to it in the source video.
    stamp = (
        f"drawtext=text='%{{eif\\:{attempt.start_s:.0f}+n*{step:.3f}\\:d}}s'"
        ":x=6:y=6:fontsize=22:fontcolor=yellow:box=1:boxcolor=black@0.6"
    )
    command = [
        "ffmpeg",
        "-v",
        "error",
        "-ss",
        f"{attempt.start_s:.3f}",
        "-t",
        f"{duration:.3f}",
        "-i",
        str(video_path),
        "-vf",
        f"fps={keyframes}/{duration:.4f},{crop},"
        f"scale={_tile_width}:{_tile_height},{stamp},"
        f"tile={_sheet_columns}x{_sheet_rows}:margin=4:padding=3",
        "-frames:v",
        "1",
        "-q:v",
        "3",
        "-y",
        str(jpg_path),
    ]
    result = subprocess.run(command, capture_output=True, check=False)
    if result.returncode != 0 or not jpg_path.exists():
        message = result.stderr.decode("utf-8", "replace").strip()
        raise RuntimeError(f"contact sheet failed for {jpg_path.name}: {message}")


def render_clip(video_path: Path, attempt: Attempt, mp4_path: Path) -> None:
    """Extract the attempt as a standalone clip for close review."""
    command = [
        "ffmpeg",
        "-v",
        "error",
        "-ss",
        f"{attempt.start_s:.3f}",
        "-t",
        f"{attempt.duration_s:.3f}",
        "-i",
        str(video_path),
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "veryfast",
        "-crf",
        "26",
        "-y",
        str(mp4_path),
    ]
    result = subprocess.run(command, capture_output=True, check=False)
    if result.returncode != 0:
        message = result.stderr.decode("utf-8", "replace").strip()
        raise RuntimeError(f"clip failed for {mp4_path.name}: {message}")


def is_success(attempt: Attempt) -> bool | None:
    """Interpret the ``success`` cell, treating blanks as undecided."""
    value = attempt.success.strip().upper()
    if value in {"TRUE", "T", "1", "Y", "YES", "O", "성공"}:
        return True
    if value in {"FALSE", "F", "0", "N", "NO", "X", "실패"}:
        return False
    return None


def format_stats(values: list[float]) -> str:
    """Render mean, standard deviation and range of durations."""
    if not values:
        return "-"
    mean = statistics.fmean(values)
    if len(values) < 2:
        return f"{mean:.1f} s (n=1)"
    deviation = statistics.stdev(values)
    return f"{mean:.1f} ± {deviation:.1f} s (min {min(values):.1f} / max {max(values):.1f})"


def format_clock(seconds: float) -> str:
    """Render a recording offset as ``mm:ss`` for scrubbing."""
    minutes, remainder = divmod(int(round(seconds)), 60)
    return f"{minutes:d}:{remainder:02d}"


def build_report(attempts: list[Attempt], task_name: str | None) -> str:
    """Compose the Markdown report for one condition.

    Elapsed time is reported as a measurement only. No threshold is
    applied to it and no score is derived from it.

    Args:
        attempts: Labelled attempts from a single recording.
        task_name: Human-readable task name, if the caller has one.

    Returns:
        The report as Markdown.
    """
    n_quick = sum(1 for attempt in attempts if attempt.kind == "quick_return")
    head = attempts[0]
    title = task_name or head.task
    lines = [
        f"# 평가 영상 분석 -- {title} / {head.model.upper()} / 데이터세트 {head.dataset_size}",
        "",
        f"원본 영상: `{head.video}`",
        "",
        "소요 시간은 측정 기록이며 점수가 아니다. 어떤 제한 시간과도 비교하지 않는다.",
        "",
        "## 수행별 기록",
        "",
        "| 회차 | 시도 | 시작 | 종료 | 소요시간(s) | 성공 | 실패 유형 | 비고 |",
        "|---|---|---|---|---|---|---|---|",
    ]

    durations: list[float] = []
    success_durations: list[float] = []
    n_success = 0
    n_pending = 0
    verdicts: dict[tuple[int, int], bool | None] = {}

    for attempt in attempts:
        verdict = is_success(attempt)
        verdicts[(attempt.trial_index, attempt.attempt_in_trial)] = verdict
        durations.append(attempt.duration_s)
        if verdict is True:
            n_success += 1
            success_durations.append(attempt.duration_s)
        elif verdict is None:
            n_pending += 1
        mark = {True: "성공", False: "실패", None: "보류"}[verdict]
        lines.append(
            f"| {attempt.trial_index} | {attempt.attempt_in_trial} "
            f"| {format_clock(attempt.start_s)} "
            f"| {format_clock(attempt.end_s)} "
            f"| {attempt.duration_s:.1f} | {mark} "
            f"| {attempt.failure_mode or '-'} | {attempt.notes or '-'} |"
        )

    n_total = len(attempts)
    n_decided = n_total - n_pending
    rate = f"{100.0 * n_success / n_decided:.1f}%" if n_decided else "-"
    trials = sorted({attempt.trial_index for attempt in attempts})
    n_paired = sum(
        1
        for trial in trials
        if all(verdicts.get((trial, slot)) is True for slot in range(1, _attempts_per_trial + 1))
    )

    lines += [
        "",
        "## 조건 요약",
        "",
        "| 항목 | 값 |",
        "|---|---|",
        f"| 총 수행 수 | {n_total} |",
        f"| 성공 수 | {n_success} |",
        f"| 성공률 (판정 완료분 기준) | {rate} |",
        f"| 전체 소요시간 | {format_stats(durations)} |",
        f"| 성공 수행 소요시간 | {format_stats(success_durations)} |",
        f"| 2연속 성공 회차 수 | {n_paired} / {len(trials)} |",
        f"| 판정 보류 | {n_pending} |",
        f"| 즉시 복귀 (quick_return) | {n_quick} |",
        "",
        "`quick_return`은 홈 자세를 벗어난 지 수 초 만에 되돌아온 "
        "수행이다. 다른 수행과 똑같이 집계하며, 검수 시 눈에 띄게 "
        "표시만 해 둔 것이다.",
        "",
    ]
    return "\n".join(lines)


def run_segment(args: argparse.Namespace) -> int:
    """Detect attempts and write ``episodes.csv``."""
    video_path = Path(args.video)
    frames = crop_to_roi(read_gray_frames(video_path, args.sample_fps), args.workspace_roi)
    motion = smooth(measure_motion(frames), args.sample_fps)
    template = find_home_template(frames, motion)
    home_distance = smooth(measure_home_distance(frames, template), args.sample_fps)
    spans, threshold = find_attempts(home_distance, args.sample_fps, args.min_duration)
    task = args.task or describe_video(video_path)[0]
    kinds = classify_spans(spans, resolve_quick_return_max(task, args.quick_return_max_duration))
    attempts = build_attempts(video_path, spans, kinds)

    directory = analysis_dir(video_path, args.out_root)
    write_attempts(attempts, directory / "episodes.csv")
    np.savez_compressed(
        directory / "signals.npz",
        home_distance=home_distance,
        motion=motion,
        threshold=threshold,
        sample_fps=args.sample_fps,
    )

    total_s = len(frames) / args.sample_fps
    n_quick = sum(1 for attempt in attempts if attempt.kind == "quick_return")
    print(f"{video_path.name}: {total_s / 60:.1f} min, {len(attempts)} attempts ({n_quick} quick returns)")
    for attempt in attempts:
        flag = "  <- quick return" if attempt.kind == "quick_return" else ""
        label = f"#{attempt.attempt_index:02d} trial {attempt.trial_index}-{attempt.attempt_in_trial}"
        print(
            f"  {label}  "
            f"{format_clock(attempt.start_s)} -> "
            f"{format_clock(attempt.end_s)}  "
            f"{attempt.duration_s:6.1f} s{flag}"
        )
    print(f"wrote {directory / 'episodes.csv'}")
    return 0


def run_sheets(args: argparse.Namespace) -> int:
    """Render the timeline and per-attempt contact sheets."""
    video_path = Path(args.video)
    directory = analysis_dir(video_path, args.out_root)
    episodes_path = directory / "episodes.csv"
    if not episodes_path.exists():
        print(f"run `segment` first: {episodes_path} is missing", file=sys.stderr)
        return 1
    attempts = read_attempts(episodes_path)

    signals_path = directory / "signals.npz"
    if signals_path.exists():
        cached = np.load(signals_path)
        render_timeline(
            cached["home_distance"],
            cached["motion"],
            float(cached["threshold"]),
            attempts,
            int(cached["sample_fps"]),
            directory / "timeline.png",
        )

    task = args.task or describe_video(video_path)[0]
    sheet_roi = args.sheet_roi or _sheet_rois.get(task, _default_sheet_roi)
    sheets_dir = directory / "sheets"
    sheets_dir.mkdir(parents=True, exist_ok=True)
    # Transits are rendered too, at their own numbering, so that a
    # reviewer can check the classifier rather than take it on trust.
    rendered = 0
    for attempt in attempts:
        stem = f"attempt_{attempt.attempt_index:02d}"
        render_contact_sheet(video_path, attempt, sheet_roi, args.keyframes, sheets_dir / f"{stem}.jpg")
        rendered += 1
        if args.clips:
            clips_dir = sheets_dir / "clips"
            clips_dir.mkdir(parents=True, exist_ok=True)
            render_clip(video_path, attempt, clips_dir / f"{stem}.mp4")

    labels_path = directory / "labels.csv"
    if not labels_path.exists():
        for attempt in attempts:
            attempt.notes = _pending_note
        write_attempts(attempts, labels_path)
        print(f"wrote {labels_path} -- fill in {', '.join(_label_columns)}")
    print(f"rendered {rendered} sheets into {sheets_dir}")
    return 0


def run_report(args: argparse.Namespace) -> int:
    """Turn a labelled CSV into the Markdown report."""
    labels_path = Path(args.labels)
    attempts = read_attempts(labels_path)
    if not attempts:
        print(f"{labels_path} has no rows", file=sys.stderr)
        return 1
    report = build_report(attempts, args.task_name)
    out_path = Path(args.out) if args.out else labels_path.parent / "report.md"
    out_path.write_text(report, encoding="utf-8")
    print(report)
    print(f"wrote {out_path}", file=sys.stderr)
    return 0


def build_parser() -> argparse.ArgumentParser:
    """Assemble the three-subcommand CLI."""
    parser = argparse.ArgumentParser(
        prog="eval_video_report",
        description=(
            "Measure per-attempt elapsed time and success from an "
            "evaluation recording, using the video alone."
        ),
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_out_root(target: argparse.ArgumentParser) -> None:
        target.add_argument(
            "--out-root",
            type=Path,
            default=None,
            help="output root (default: <video dir>/analysis)",
        )

    segment = subparsers.add_parser("segment", help="detect attempt spans")
    segment.add_argument("video", help="recording to analyse")
    add_out_root(segment)
    segment.add_argument(
        "--workspace-roi",
        type=parse_roi,
        default=_workspace_roi,
        help="region framing the arm, as x0,y0,x1,y1 frame fractions",
    )
    segment.add_argument(
        "--sample-fps",
        type=int,
        default=_sample_fps,
        help="frames per second to analyse",
    )
    segment.add_argument(
        "--min-duration",
        type=float,
        default=_min_span_s,
        help="shortest away-from-home span to keep at all, in seconds",
    )
    segment.add_argument(
        "--quick-return-max-duration",
        type=float,
        default=None,
        help="attempts no longer than this are flagged as quick returns (default: per task)",
    )
    segment.add_argument(
        "--task",
        default=None,
        help="task preset to use (default: read from the filename)",
    )
    segment.set_defaults(handler=run_segment)

    sheets = subparsers.add_parser("sheets", help="render review material")
    sheets.add_argument("video", help="recording the episodes came from")
    add_out_root(sheets)
    sheets.add_argument(
        "--sheet-roi",
        type=parse_roi,
        default=None,
        help="region framing the props, x0,y0,x1,y1 fractions (default: per task)",
    )
    sheets.add_argument(
        "--task",
        default=None,
        help="task preset to use (default: read from the filename)",
    )
    sheets.add_argument(
        "--keyframes",
        type=int,
        default=_keyframe_count,
        help="frames sampled across each attempt",
    )
    sheets.add_argument(
        "--clips",
        action="store_true",
        help="also extract one mp4 per attempt (large)",
    )
    sheets.set_defaults(handler=run_sheets)

    report = subparsers.add_parser("report", help="build the Markdown report")
    report.add_argument("--labels", required=True, help="labelled CSV")
    report.add_argument("--out", default=None, help="output Markdown path")
    report.add_argument(
        "--task-name",
        default=None,
        help="human-readable task name for the report heading",
    )
    report.set_defaults(handler=run_report)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Entry point for ``python -m lerobot.scripts.eval_video_report``."""
    args = build_parser().parse_args(argv)
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
