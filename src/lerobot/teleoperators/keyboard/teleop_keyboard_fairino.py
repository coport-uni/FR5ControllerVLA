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

"""Keyboard teleoperator for the Fairino FR5 6-DOF arm.

Outputs **absolute** joint positions (degrees) that are
compatible with the standard ``lerobot-teleoperate`` pipeline.
The teleoperator receives the robot's current observation via
``send_feedback(obs)`` each loop iteration, and adds keyboard-
driven deltas to produce the next target position.

Uses terminal stdin (termios) -- no X11 / display required.

Keyboard controls:
    1-6         Select active joint (joint1 .. joint6)
    W / Up      Increase selected joint angle by step_size
    S / Down    Decrease selected joint angle by step_size
    + / =       Increase step size
    - / _       Decrease step size
    P           Print current state info to logger
    R           Reset robot errors (sets reset_requested)
    Q           Quit (sets quit_requested)
"""

import logging
import select
import sys
import termios
import threading
import tty
from queue import Queue
from typing import Any

from lerobot.types import RobotAction
from lerobot.utils.decorators import (
    check_if_already_connected,
    check_if_not_connected,
)

from ..teleoperator import Teleoperator
from .configuration_keyboard import KeyboardFairinoTeleopConfig

logger = logging.getLogger(__name__)

JOINT_NAMES = [
    "joint1", "joint2", "joint3",
    "joint4", "joint5", "joint6",
]

# ANSI escape sequences produced by arrow keys.
_ARROW_UP = "\x1b[A"
_ARROW_DOWN = "\x1b[B"

# Polling interval inside the stdin reader thread [s].
_POLL_TIMEOUT_S = 0.05


# ---- background stdin reader --------------------------------

class _StdinReader(threading.Thread):
    """Read raw key presses from stdin in a background thread.

    Falls back to line-buffered input when stdin is not a real
    TTY (e.g. piped or non-interactive).
    """

    daemon = True

    def __init__(self, queue: Queue):
        super().__init__()
        self._queue = queue
        self._stop_event = threading.Event()

    def run(self):
        """Dispatch to raw or line-buffered reader."""
        fd = sys.stdin.fileno()
        if sys.stdin.isatty():
            self._run_raw(fd)
        else:
            self._run_line(fd)

    def stop(self):
        """Signal the reader thread to exit."""
        self._stop_event.set()

    # -- private readers --------------------------------------

    def _run_raw(self, fd: int) -> None:
        """Read one character at a time from a real terminal."""
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not self._stop_event.is_set():
                if not select.select(
                    [fd], [], [], _POLL_TIMEOUT_S
                )[0]:
                    continue
                ch = sys.stdin.read(1)
                if not ch:
                    break
                if ch == "\x1b":
                    # Might be an arrow-key escape sequence.
                    ch = self._read_escape(fd, ch)
                else:
                    self._queue.put(ch)
        finally:
            termios.tcsetattr(
                fd, termios.TCSADRAIN, old_settings
            )

    def _read_escape(self, fd: int, ch: str) -> str:
        """Consume a multi-byte ANSI escape and enqueue it."""
        if select.select(
            [fd], [], [], _POLL_TIMEOUT_S
        )[0]:
            ch += sys.stdin.read(1)
            if (
                ch[-1] == "["
                and select.select(
                    [fd], [], [], _POLL_TIMEOUT_S
                )[0]
            ):
                ch += sys.stdin.read(1)
        if ch == _ARROW_UP:
            self._queue.put("UP")
        elif ch == _ARROW_DOWN:
            self._queue.put("DOWN")
        else:
            self._queue.put("ESC")
        return ch

    def _run_line(self, fd: int) -> None:
        """Fallback: line-buffered input for non-TTY stdin."""
        logger.warning(
            "[KeyboardFairino] stdin is not a TTY; "
            "using line-buffered input."
        )
        while not self._stop_event.is_set():
            if select.select(
                [fd], [], [], _POLL_TIMEOUT_S
            )[0]:
                line = sys.stdin.readline().strip()
                for ch in line:
                    self._queue.put(ch)


# ---- teleoperator -------------------------------------------

class KeyboardFairinoTeleop(Teleoperator):
    """Joint-space keyboard teleop for the Fairino FR5 arm.

    Tracks an internal target joint vector and applies keyboard
    deltas on each ``get_action()`` call.  The initial target is
    set from the first ``send_feedback(obs)`` call, which syncs
    with the robot's real joint positions.

    The output dict contains only ``"joint{1..6}.pos"`` keys
    (absolute degrees) -- no meta keys -- so it is directly
    usable by ``FairinoFollower.send_action()``.
    """

    config_class = KeyboardFairinoTeleopConfig
    name = "keyboard_fairino"

    def __init__(self, config: KeyboardFairinoTeleopConfig):
        super().__init__(config)
        self.config = config

        self.event_queue: Queue = Queue()
        self.current_pressed: dict = {}
        self._reader: _StdinReader | None = None

        # Teleop state.
        self.selected_joint: int = 0
        self.step_size: float = config.step_size_deg
        self._quit_requested: bool = False
        self._reset_requested: bool = False

        # Absolute joint target [deg].
        # Initialised on the first ``send_feedback`` call.
        self._target_joints: list[float] | None = None

    # ---- properties (required by Teleoperator) --------------

    @property
    def action_features(self) -> dict:
        """Action schema: six joint positions [deg]."""
        return {
            f"{j}.pos": float for j in JOINT_NAMES
        }

    @property
    def feedback_features(self) -> dict:
        """Feedback schema: six joint positions [deg]."""
        return {
            f"{j}.pos": float for j in JOINT_NAMES
        }

    @property
    def is_connected(self) -> bool:
        return (
            self._reader is not None
            and self._reader.is_alive()
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    @property
    def reset_requested(self) -> bool:
        """True once after the user pressed 'R'."""
        val = self._reset_requested
        self._reset_requested = False
        return val

    # ---- lifecycle ------------------------------------------

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        """Start the background terminal key reader."""
        self._reader = _StdinReader(self.event_queue)
        self._reader.start()
        logger.info(
            "[KeyboardFairino] Listener started "
            "(no X11 needed)."
        )

    def calibrate(self) -> None:
        """No-op: keyboard needs no calibration."""

    def configure(self) -> None:
        """No-op: no runtime configuration needed."""

    @check_if_not_connected
    def disconnect(self) -> None:
        """Stop the background key reader thread."""
        if self._reader is not None:
            self._reader.stop()
            self._reader.join(timeout=2.0)
            self._reader = None
        logger.info(
            "[KeyboardFairino] Listener stopped."
        )

    # ---- feedback and action --------------------------------

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """Receive robot observation to initialise targets.

        Only the **first** call sets ``_target_joints``.
        Subsequent calls are ignored so the teleop keeps
        ownership of the target trajectory.

        Args:
            feedback: Robot observation dict with joint keys.
        """
        joints = []
        for jname in JOINT_NAMES:
            key = f"{jname}.pos"
            if key in feedback:
                joints.append(float(feedback[key]))
        if len(joints) != len(JOINT_NAMES):
            return
        if self._target_joints is not None:
            return

        self._target_joints = joints
        logger.info(
            "[KeyboardFairino] Targets initialised: %s",
            [f"{j:.2f}" for j in joints],
        )

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        """Produce an absolute joint-position action.

        Returns:
            Dict with "joint{1..6}.pos" target angles [deg].
        """
        self._drain_events()

        def pressed(k: str) -> bool:
            return self.current_pressed.get(k, False)

        # Joint selection (keys 1-6).
        for i, digit in enumerate(
            ["1", "2", "3", "4", "5", "6"]
        ):
            if pressed(digit):
                self.selected_joint = i

        # Joint movement (W/S or arrow keys).
        move_pos = (
            pressed("w") or pressed("W")
            or pressed("UP")
        )
        move_neg = (
            pressed("s") or pressed("S")
            or pressed("DOWN")
        )

        delta = 0.0
        if move_pos and not move_neg:
            delta = self.step_size
        elif move_neg and not move_pos:
            delta = -self.step_size

        # Step-size adjustment (+/-).
        if pressed("+") or pressed("="):
            self.step_size = min(
                self.step_size
                + self.config.step_increment_deg,
                self.config.max_step_deg,
            )
        if pressed("-"):
            self.step_size = max(
                self.step_size
                - self.config.step_increment_deg,
                self.config.min_step_deg,
            )

        # Utility keys.
        if pressed("p") or pressed("P"):
            self._log_state()
        if pressed("r") or pressed("R"):
            self._reset_requested = True
        if (
            pressed("q") or pressed("Q")
            or pressed("ESC")
        ):
            self._quit_requested = True

        self.current_pressed.clear()

        # Build absolute target action.
        if self._target_joints is None:
            return {
                f"{j}.pos": 0.0 for j in JOINT_NAMES
            }

        if abs(delta) > 1e-9:
            self._target_joints[self.selected_joint] += (
                delta
            )

        return {
            f"{j}.pos": self._target_joints[i]
            for i, j in enumerate(JOINT_NAMES)
        }

    # ---- private helpers ------------------------------------

    def _drain_events(self) -> None:
        """Consume all queued key events into pressed map."""
        while not self.event_queue.empty():
            key = self.event_queue.get_nowait()
            self.current_pressed[key] = True

    def _log_state(self) -> None:
        """Print current targets and step size to logger."""
        if self._target_joints is not None:
            logger.info(
                "[KeyboardFairino] Targets (deg): %s  "
                "selected=joint%d  step=%.1f",
                [f"{j:.2f}" for j in self._target_joints],
                self.selected_joint + 1,
                self.step_size,
            )
