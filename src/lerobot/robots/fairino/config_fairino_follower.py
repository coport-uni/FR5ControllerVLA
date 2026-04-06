#!/usr/bin/env python

# Copyright 2026 APPEAL Automation team. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("fairino_follower")
@dataclass(kw_only=True)
class FairinoFollowerConfig(RobotConfig):
    """Configuration for the Fairino FR5 6-DOF collaborative robot."""

    # Network
    ip_address: str = "192.168.58.2"

    # Kinematics
    num_joints: int = 6

    joint_names: list[str] = field(
        default_factory=lambda: [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6",
        ]
    )

    # Joint limits in degrees (Fairino FR5 spec)
    joint_limits_lower: list[float] = field(
        default_factory=lambda: [-175.0, -265.0, -160.0, -265.0, -175.0, -175.0]
    )
    joint_limits_upper: list[float] = field(
        default_factory=lambda: [175.0, 85.0, 160.0, 85.0, 175.0, 175.0]
    )

    # Control parameters
    control_hz: float = 20.0
    move_speed: float = 0.1  # 0.0 ~ 1.0, maps to 0-100% in SDK

    # Cameras
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Safety
    max_relative_target: float | dict[str, float] | None = None

    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)
