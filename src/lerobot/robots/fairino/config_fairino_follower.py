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

"""Configuration dataclass for the Fairino FR5 robot."""

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("fairino_follower")
@dataclass(kw_only=True)
class FairinoFollowerConfig(RobotConfig):
    """Settings for the Fairino FR5 6-DOF collaborative arm.

    Attributes:
        ip_address: Controller network address.
        num_joints: Number of revolute joints.
        joint_names: Ordered list of joint identifiers.
        joint_limits_lower: Minimum angles [deg] per joint.
        joint_limits_upper: Maximum angles [deg] per joint.
        control_hz: ServoJ command rate [Hz].
        move_speed: Velocity scale 0.0-1.0 (maps to 0-100%).
        cameras: Optional camera configurations.
        max_relative_target: Safety cap on per-step motion.
    """

    # -- network ---------------------------------------------
    ip_address: str = "192.168.58.2"

    # -- kinematics ------------------------------------------
    num_joints: int = 6

    joint_names: list[str] = field(
        default_factory=lambda: [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6",
        ]
    )

    # Joint limits [deg] from the FR5 datasheet.
    joint_limits_lower: list[float] = field(
        default_factory=lambda: [
            -175.0, -265.0, -160.0,
            -265.0, -175.0, -175.0,
        ]
    )
    joint_limits_upper: list[float] = field(
        default_factory=lambda: [
            175.0, 85.0, 160.0,
            85.0, 175.0, 175.0,
        ]
    )

    # -- control ---------------------------------------------
    control_hz: float = 20.0
    servo_hz: float = 100.0
    move_speed: float = 0.1

    # -- peripherals -----------------------------------------
    cameras: dict[str, CameraConfig] = field(
        default_factory=dict
    )

    # -- gripper ---------------------------------------------
    # Enable gripper attached to tool flange.
    gripper_enabled: bool = False

    # Gripper manufacturer:
    #   1=Robotiq, 2=慧灵, 3=天机, 4=大寰, 5=知行
    gripper_company: int = 4

    # Device number (manufacturer-specific).
    #   Robotiq: 0=2F-85  天机: 0=TEG-110  大寰: 0=PGI-140
    gripper_device: int = 0

    # Gripper index (usually 1).
    gripper_index: int = 1

    # Default velocity for MoveGripper [0-100 %].
    gripper_vel: int = 50

    # Default force/torque for MoveGripper [0-100 %].
    gripper_force: int = 50

    # -- safety ----------------------------------------------
    max_relative_target: (
        float | dict[str, float] | None
    ) = None

    # Maximum joint velocity for the servo loop [deg/s].
    # ServoJ rejects large position jumps, so the servo
    # thread interpolates toward the target at this rate.
    max_servo_speed: float = 60.0

    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)
