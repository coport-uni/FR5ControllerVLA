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

"""Configuration for the Fairino FR5 leader teleoperator."""

from dataclasses import dataclass, field

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("fairino_leader")
@dataclass(kw_only=True)
class FairinoLeaderConfig(TeleoperatorConfig):
    """Settings for an FR5 arm used as a drag-teach leader.

    The leader enters freedrive (drag-teach) mode so the
    operator can move it by hand.  Joint positions are read
    at each control step and forwarded to the follower.

    Attributes:
        ip_address: Controller network address.
        joint_names: Ordered joint identifiers (must match
            the follower's naming convention).
    """

    ip_address: str = "192.168.59.2"

    # -- gripper -----------------------------------------
    gripper_enabled: bool = False
    gripper_company: int = 4  # 4=大寰
    gripper_device: int = 0  # 0=PGI-140
    gripper_index: int = 1

    joint_names: list[str] = field(
        default_factory=lambda: [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
    )

    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)
