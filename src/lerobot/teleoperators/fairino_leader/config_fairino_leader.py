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

    # -- gripper (drag-teach: operator moves jaws by hand) --
    # The gripper is activated with a weak holding force so the
    # operator can back-drive the jaws; ``get_action()`` reads
    # the live position each loop and forwards it to the
    # follower.
    gripper_enabled: bool = False

    # Gripper manufacturer (matches follower).
    #   1=Robotiq, 2=慧灵, 3=天机, 4=大寰, 5=知行
    gripper_company: int = 4

    # Device number (manufacturer-specific).
    gripper_device: int = 0

    # Gripper index (usually 1).
    gripper_index: int = 1

    # Velocity for the compliance MoveGripper call [0-100 %].
    gripper_vel: int = 50

    # Holding force applied during drag-teach [0-100 %].
    # Low values (e.g. 1) keep the jaws easy to move by hand;
    # higher values (e.g. 20) add resistance for test runs.
    gripper_force: int = 1

    # -- gripper keepalive ----------------------------------
    # Compliance-refresh worker.  Even maxtime = int32 max
    # (issue #24) expires in practice, so we re-issue
    # MoveGripper while the leader is idle to keep jaws
    # back-drivable.  The nudge magnitude must be smaller
    # than the idle position threshold (otherwise the nudge
    # itself would count as activity and reset the clock).
    gripper_keepalive_enabled: bool = True

    # Idle duration before a nudge is issued [s].
    gripper_keepalive_interval_s: float = 10.0

    # Per-joint change threshold to count as activity [deg].
    gripper_idle_joint_deg: float = 0.1

    # Gripper position change threshold for activity [%].
    # Must exceed ``gripper_nudge_pct`` to avoid self-trigger.
    gripper_idle_pos_pct: float = 2.0

    # Nudge magnitude [%] -- how far the jaws briefly move.
    gripper_nudge_pct: float = 1.0

    # Delay between nudge-out and nudge-back [s].
    gripper_nudge_settle_s: float = 0.3

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
