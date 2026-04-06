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

"""Teleoperate Fairino FR5 using a PiPER arm as leader.

Uses the standard ``teleop_loop`` from ``lerobot_teleoperate``
with a ``PiperToFairinoStep`` processor that maps PiPER's
normalised joint values (-100..100) to Fairino degrees.

**Standalone usage** (simple CLI)::

    python -m lerobot.scripts.teleop_piper_fairino
    python -m lerobot.scripts.teleop_piper_fairino \\
        --can can0 --ip 192.168.58.2 --hz 20

**Standard lerobot-teleoperate CLI** (equivalent)::

    lerobot-teleoperate \\
        --robot.type=fairino_follower \\
        --robot.ip_address=192.168.58.2 \\
        --teleop.type=piper_leader \\
        --teleop.port=can0 \\
        --fps=20
"""

import logging
from dataclasses import asdict, dataclass
from pprint import pformat

from lerobot.configs import parser
from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_robot_action_processor,
    make_default_robot_observation_processor,
)
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.processor.piper_fairino_processor import (
    PiperToFairinoStep,
)
from lerobot.processor.pipeline import RobotProcessorPipeline
from lerobot.robots import RobotConfig, make_robot_from_config
from lerobot.robots.fairino import FairinoFollowerConfig  # noqa: F401
from lerobot.scripts.lerobot_teleoperate import teleop_loop
from lerobot.teleoperators import (
    TeleoperatorConfig,
    make_teleoperator_from_config,
)
from lerobot.teleoperators.piper_leader import (  # noqa: F401
    PiperLeaderConfig,
)
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging

logger = logging.getLogger(__name__)


def make_piper_to_fairino_teleop_processor() -> (
    RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ]
):
    """Build a teleop action processor with PiPER->Fairino mapping."""
    return RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ](
        steps=[PiperToFairinoStep()],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )


@dataclass
class PiperFairinoTeleoperateConfig:
    """Configuration for PiPER-to-Fairino teleoperation.

    Attributes:
        teleop: PiPER leader teleoperator configuration.
        robot: Fairino follower robot configuration.
        fps: Target control loop frequency [Hz].
        teleop_time_s: Maximum duration [s] (None = infinite).
        display_data: Show live joint data via Rerun.
    """

    teleop: TeleoperatorConfig
    robot: RobotConfig
    fps: int = 20
    teleop_time_s: float | None = None
    display_data: bool = False


@parser.wrap()
def teleoperate(cfg: PiperFairinoTeleoperateConfig):
    """Connect PiPER leader and Fairino follower, then run the teleop loop."""
    init_logging()
    logging.info(pformat(asdict(cfg)))

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)

    # PiPER->Fairino mapping in the teleop action processor.
    teleop_action_processor = (
        make_piper_to_fairino_teleop_processor()
    )
    robot_action_processor = (
        make_default_robot_action_processor()
    )
    robot_observation_processor = (
        make_default_robot_observation_processor()
    )

    teleop.connect()
    robot.connect()

    try:
        teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=cfg.fps,
            display_data=cfg.display_data,
            duration=cfg.teleop_time_s,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
        )
    except KeyboardInterrupt:
        pass
    finally:
        teleop.disconnect()
        robot.disconnect()


def main():
    """Entry point for standalone CLI."""
    register_third_party_plugins()
    teleoperate()


if __name__ == "__main__":
    main()
