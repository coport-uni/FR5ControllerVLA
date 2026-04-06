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
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Keyboard teleoperator for the Fairino FR5 6-DOF robot arm.

Outputs **absolute** joint positions (degrees) compatible with the standard
``lerobot-teleoperate`` pipeline.  The teleoperator receives the robot's
current observation via ``send_feedback(obs)`` each loop iteration and adds
keyboard-driven deltas to produce the next target position.

Uses terminal stdin (termios) for keyboard input — no X11/display required.

Keyboard controls:
    Joint selection:
        1-6         Select active joint (joint1 .. joint6)

    Joint movement:
        W / Up      Increase selected joint angle by step_size
        S / Down    Decrease selected joint angle by step_size

    Step size:
        + / =       Increase step size
        - / _       Decrease step size

    Utility:
        P           Print current state info to logger
        R           Reset robot errors (sets _reset_requested flag)
        Q           Quit (sets quit_requested flag)
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
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .configuration_keyboard import KeyboardFairinoTeleopConfig

logger = logging.getLogger(__name__)

# Joint names for Fairino FR5
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# ANSI escape sequences for arrow keys
_ARROW_UP = "\x1b[A"
_ARROW_DOWN = "\x1b[B"


class _StdinReader(threading.Thread):
    """Background thread that reads raw key presses from stdin (terminal).

    Falls back to line-buffered input if stdin is not a real TTY
    (e.g. when piped or in a non-interactive environment).
    """

    daemon = True

    def __init__(self, queue: Queue):
        super().__init__()
        self._queue = queue
        self._stop_event = threading.Event()

    def run(self):
        fd = sys.stdin.fileno()
        is_tty = sys.stdin.isatty()

        if is_tty:
            self._run_raw(fd)
        else:
            self._run_line(fd)

    def _run_raw(self, fd):
        """Read one character at a time from a real terminal."""
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not self._stop_event.is_set():
                if select.select([fd], [], [], 0.05)[0]:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break
                    if ch == "\x1b":
                        if select.select([fd], [], [], 0.05)[0]:
                            ch += sys.stdin.read(1)
                            if ch[-1] == "[" and select.select([fd], [], [], 0.05)[0]:
                                ch += sys.stdin.read(1)
                        if ch == _ARROW_UP:
                            self._queue.put("UP")
                        elif ch == _ARROW_DOWN:
                            self._queue.put("DOWN")
                        else:
                            self._queue.put("ESC")
                    else:
                        self._queue.put(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _run_line(self, fd):
        """Fallback: read line-buffered input (non-TTY stdin)."""
        logger.warning(
            "[KeyboardFairino] stdin is not a TTY — using line-buffered input. "
            "Type a key and press Enter."
        )
        while not self._stop_event.is_set():
            if select.select([fd], [], [], 0.05)[0]:
                line = sys.stdin.readline().strip()
                for ch in line:
                    self._queue.put(ch)

    def stop(self):
        self._stop_event.set()


class KeyboardFairinoTeleop(Teleoperator):
    """
    Keyboard teleoperator designed for the Fairino FR5 6-DOF robot arm.

    Outputs a RobotAction dict with keys ``"joint1.pos"`` .. ``"joint6.pos"``
    representing **absolute** target angles in degrees.  Internally it tracks
    the current joint state (updated via ``send_feedback``) and adds keyboard
    deltas on top.  This makes it fully compatible with the standard
    ``lerobot-teleoperate`` pipeline which passes actions directly to
    ``Robot.send_action()``.
    """

    config_class = KeyboardFairinoTeleopConfig
    name = "keyboard_fairino"

    def __init__(self, config: KeyboardFairinoTeleopConfig):
        super().__init__(config)
        self.config = config

        self.event_queue: Queue = Queue()
        self.current_pressed: dict = {}
        self._reader: _StdinReader | None = None

        # State
        self.selected_joint: int = 0  # index into JOINT_NAMES (0-5)
        self.step_size: float = config.step_size_deg
        self._quit_requested: bool = False
        self._reset_requested: bool = False

        # Absolute joint target (degrees).  Initialised from the first
        # ``send_feedback`` call with the robot's actual observation.
        self._target_joints: list[float] | None = None

    # ------------------------------------------------------------------
    # Abstract property implementations
    # ------------------------------------------------------------------

    @property
    def action_features(self) -> dict:
        return {f"{jname}.pos": float for jname in JOINT_NAMES}

    @property
    def feedback_features(self) -> dict:
        return {f"{jname}.pos": float for jname in JOINT_NAMES}

    @property
    def is_connected(self) -> bool:
        return self._reader is not None and self._reader.is_alive()

    @property
    def is_calibrated(self) -> bool:
        return True

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    @property
    def reset_requested(self) -> bool:
        """True when the user pressed 'R'.  Cleared after reading."""
        val = self._reset_requested
        self._reset_requested = False
        return val

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self._reader = _StdinReader(self.event_queue)
        self._reader.start()
        logger.info("[KeyboardFairino] Terminal keyboard listener started (no X11 needed).")

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_not_connected
    def disconnect(self) -> None:
        if self._reader is not None:
            self._reader.stop()
            self._reader.join(timeout=2.0)
            self._reader = None
        logger.info("[KeyboardFairino] Keyboard listener stopped.")

    # ------------------------------------------------------------------
    # Event processing
    # ------------------------------------------------------------------

    def _drain_events(self):
        """Process all pending keyboard events and update internal state."""
        while not self.event_queue.empty():
            key = self.event_queue.get_nowait()
            self.current_pressed[key] = True

    # ------------------------------------------------------------------
    # Feedback & action
    # ------------------------------------------------------------------

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """Receive current robot observation to keep joint targets in sync."""
        joints = []
        for jname in JOINT_NAMES:
            key = f"{jname}.pos"
            if key in feedback:
                joints.append(float(feedback[key]))
        if len(joints) == len(JOINT_NAMES):
            if self._target_joints is None:
                # First feedback — initialise targets from actual robot state.
                self._target_joints = joints
                logger.info(
                    "[KeyboardFairino] Initialised targets from robot: "
                    f"{[f'{j:.2f}' for j in joints]}"
                )
            # On subsequent calls we do NOT overwrite _target_joints, because
            # the teleop owns the target trajectory.  The robot observation is
            # only used for initialisation.

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        """
        Read keyboard state and produce an **absolute** joint-position action.

        Returns:
            dict with keys ``"joint1.pos"`` .. ``"joint6.pos"`` — target
            angles in degrees.
        """
        self._drain_events()

        # Helper to check if a key is currently pressed
        def pressed(k: str) -> bool:
            return self.current_pressed.get(k, False)

        # --- Joint selection (1-6) ---
        for i, digit in enumerate(["1", "2", "3", "4", "5", "6"]):
            if pressed(digit):
                self.selected_joint = i

        # --- Joint movement (W/S or Up/Down) ---
        move_positive = pressed("w") or pressed("W") or pressed("UP")
        move_negative = pressed("s") or pressed("S") or pressed("DOWN")

        delta = 0.0
        if move_positive and not move_negative:
            delta = self.step_size
        elif move_negative and not move_positive:
            delta = -self.step_size

        # --- Step size adjustment (+/-) ---
        if pressed("+") or pressed("="):
            self.step_size = min(self.step_size + self.config.step_increment_deg, self.config.max_step_deg)
        if pressed("-"):
            self.step_size = max(self.step_size - self.config.step_increment_deg, self.config.min_step_deg)

        # --- Utility keys ---
        if pressed("p") or pressed("P"):
            if self._target_joints is not None:
                logger.info(
                    f"[KeyboardFairino] Targets (deg): "
                    f"{[f'{j:.2f}' for j in self._target_joints]}  "
                    f"selected=joint{self.selected_joint+1}  step={self.step_size:.1f}"
                )
        if pressed("r") or pressed("R"):
            self._reset_requested = True
        if pressed("q") or pressed("Q") or pressed("ESC"):
            self._quit_requested = True

        # Clear pressed state so keys don't repeat
        self.current_pressed.clear()

        # --- Build absolute target action ---
        if self._target_joints is None:
            # No feedback received yet — output zeros (robot will stay in place)
            return {f"{jname}.pos": 0.0 for jname in JOINT_NAMES}

        # Apply delta to the selected joint
        if abs(delta) > 1e-9:
            self._target_joints[self.selected_joint] += delta

        return {f"{jname}.pos": self._target_joints[i] for i, jname in enumerate(JOINT_NAMES)}
