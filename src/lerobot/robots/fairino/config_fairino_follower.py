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
from pathlib import Path

from lerobot.cameras import CameraConfig

from ..config import RobotConfig

@RobotConfig.register_subclass("fairino_follower")
@dataclass(kw_only=True)
class FairinoFollowerConfig(RobotConfig):
    '''
    This function instance new robot class with given parameters.
    '''

    # IP address 
    ip_address : str = "192.168.58.2"

    # Kinematics info
    num_joints: int = 6

    joint_names: list[str] = field(
        default_factory=lambda: [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"
        ]
    )
    joint_limits_lower: list[float] = field(
        default_factory=lambda: [-175, -265, -160, -265, -175, -175]
    )
    joint_limits_upper: list[float] = field(
        default_factory=lambda: [175, 85, 160, 85, 175, 175]
    )
    control_hz: float = 20.0
    move_speed: float = 0.1

    # LeRobot은 basically use radian
    use_radian: bool = True

    # Camera setup. recycle from piper repo
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    
    disable_torque_on_disconnect: bool = True
    
    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a dictionary that maps motor
    # names to the max_relative_target value for that motor.
    max_relative_target: float | dict[str, float] | None = None

    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)