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

"""PiPER arm as a leader (teleoperator) for reading joint positions.

Connects to the PiPER arm in slave mode (torque off) and reads
joint angles passively via CAN bus. Outputs normalised values
in the range -100..100 for joints and 0..100 for the gripper.
"""

import logging
import time
from typing import Any

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.piper import PiperMotorsBus
from lerobot.types import RobotAction

from ..teleoperator import Teleoperator
from .config_piper_leader import PiperLeaderConfig

logger = logging.getLogger(__name__)

# Motor names shared with the PiPER follower definition.
JOINT_NAMES = [
    "joint1", "joint2", "joint3",
    "joint4", "joint5", "joint6",
]
MOTOR_NAMES = JOINT_NAMES + ["gripper"]


class PiperLeader(Teleoperator):
    """Read PiPER joint positions for use as a teleop leader.

    The arm runs in slave (passive) mode so the operator can
    move it freely by hand while the joint angles are streamed
    to a follower robot.

    The CAN bus object is created lazily in ``connect()``
    because the PiPER SDK opens the CAN socket at construction
    time.
    """

    config_class = PiperLeaderConfig
    name = "piper_leader"

    def __init__(self, config: PiperLeaderConfig):
        super().__init__(config)
        self.config = config
        # Bus is created lazily in connect() because the
        # piper_sdk opens the CAN socket at construction.
        self.bus: PiperMotorsBus | None = None

    def _create_bus(self) -> PiperMotorsBus:
        """Build the PiperMotorsBus (opens CAN socket)."""
        return PiperMotorsBus(
            id=self.config.id,
            port=self.config.port,
            motors={
                "joint1": Motor(
                    1, "AGILEX-M",
                    MotorNormMode.RANGE_M100_100,
                ),
                "joint2": Motor(
                    2, "AGILEX-M",
                    MotorNormMode.RANGE_M100_100,
                ),
                "joint3": Motor(
                    3, "AGILEX-M",
                    MotorNormMode.RANGE_M100_100,
                ),
                "joint4": Motor(
                    4, "AGILEX-S",
                    MotorNormMode.RANGE_M100_100,
                ),
                "joint5": Motor(
                    5, "AGILEX-S",
                    MotorNormMode.RANGE_M100_100,
                ),
                "joint6": Motor(
                    6, "AGILEX-S",
                    MotorNormMode.RANGE_M100_100,
                ),
                "gripper": Motor(
                    7, "AGILEX-S",
                    MotorNormMode.RANGE_0_100,
                ),
            },
            calibration={
                "joint1": MotorCalibration(
                    1, 0, 0, -150000, 150000,
                ),
                "joint2": MotorCalibration(
                    2, 0, 0, 0, 180000,
                ),
                "joint3": MotorCalibration(
                    3, 0, 0, -170000, 0,
                ),
                "joint4": MotorCalibration(
                    4, 0, 0, -100000, 100000,
                ),
                "joint5": MotorCalibration(
                    5, 0, 0, -65000, 65000,
                ),
                "joint6": MotorCalibration(
                    6, 0, 0, -100000, 130000,
                ),
                "gripper": MotorCalibration(
                    7, 0, 0, 0, 68000,
                ),
            },
        )

    # ---- properties (required by Teleoperator) ---------------

    @property
    def action_features(self) -> dict[str, type]:
        """Action schema: 6 joints + gripper normalised values."""
        return {
            f"{motor}.pos": float
            for motor in MOTOR_NAMES
        }

    @property
    def feedback_features(self) -> dict[str, type]:
        """No feedback needed for a passive leader."""
        return {}

    @property
    def is_connected(self) -> bool:
        return (
            self.bus is not None and self.bus.is_connected
        )

    @property
    def is_calibrated(self) -> bool:
        if self.bus is None:
            return True
        return self.bus.is_calibrated

    # ---- lifecycle -------------------------------------------

    def connect(self, calibrate: bool = True) -> None:
        """Open the CAN port and set PiPER to slave mode."""
        if self.is_connected:
            raise RuntimeError(
                "[PiperLeader] Already connected."
            )

        self.bus = self._create_bus()

        if not self.bus.connect():
            raise ConnectionError(
                "[PiperLeader] Failed to open CAN port "
                f"'{self.config.port}'."
            )

        # Slave mode: torque off, arm is free to move by hand.
        self.bus.set_slave()
        logger.info(
            "[PiperLeader] Connected on '%s' (slave mode).",
            self.config.port,
        )

    def calibrate(self) -> None:
        """No-op: PiPER uses absolute encoders."""

    def configure(self) -> None:
        """No-op: no extra configuration needed."""

    def disconnect(self) -> None:
        """Close the CAN port."""
        if self.bus is not None and self.bus.is_connected:
            self.bus.disconnect(disable_torque=False)
        self.bus = None
        logger.info("[PiperLeader] Disconnected.")

    # ---- action / feedback -----------------------------------

    def get_action(self) -> RobotAction:
        """Read current PiPER joint positions.

        Returns:
            Dict with "joint{1..6}.pos" and "gripper.pos"
            as normalised float values (-100..100 / 0..100).
        """
        start = time.perf_counter()
        raw = self.bus.get_action()
        action = {
            f"{motor}.pos": val
            for motor, val in raw.items()
        }
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(
            "[PiperLeader] read action: %.1fms", dt_ms,
        )
        return action

    def send_feedback(
        self, feedback: dict[str, Any],
    ) -> None:
        """No-op: passive leader ignores feedback."""
