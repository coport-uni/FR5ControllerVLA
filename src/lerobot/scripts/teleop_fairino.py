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

"""Standalone keyboard teleoperation for the Fairino FR5.

Usage::

    python -m lerobot.scripts.teleop_fairino --ip 192.168.58.2
    python -m lerobot.scripts.teleop_fairino --step 2.0 --speed 0.3
"""

import argparse
import logging
import sys
import time

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)

JOINT_NAMES = [
    "joint1", "joint2", "joint3",
    "joint4", "joint5", "joint6",
]

# ANSI terminal helpers.
_CLR = "\033[2K"
_UP = "\033[A"
_BOLD = "\033[1m"
_RST = "\033[0m"
_GRN = "\033[92m"
_YLW = "\033[93m"
_CYN = "\033[96m"

# Number of display lines to overwrite per frame.
_DISPLAY_LINES = len(JOINT_NAMES) + 3


def _print_banner() -> None:
    """Show the keyboard-control reference banner."""
    print(f"""
{_BOLD}{'=' * 56}
  Fairino FR5 Keyboard Teleoperation
{'=' * 56}
  1-6        Select joint
  W / Up     Joint angle +     S / Down   Joint angle -
  + / =      Step size +       -          Step size -
  P          Print state       R          Reset errors
  Q / Esc    Quit
{'=' * 56}{_RST}
""")


def _format_joints(
    joints_deg: list[float],
    selected: int,
    step: float,
) -> str:
    """Build a compact multi-line joint-state display."""
    lines = [f"  Step: {_CYN}{step:.1f}{_RST} deg", ""]
    for i, (name, val) in enumerate(
        zip(JOINT_NAMES, joints_deg, strict=True)
    ):
        marker = f"{_GRN}>{_RST}" if i == selected else " "
        lines.append(f"  {marker} {name}: {val:>8.2f} deg")
    return "\n".join(lines)


def run_teleop(
    ip: str,
    step_size: float,
    speed: float,
    hz: float,
) -> None:
    """Main teleoperation loop.

    Args:
        ip: Fairino controller IP address.
        step_size: Initial angular step per keypress [deg].
        speed: Movement speed scale (0.0 - 1.0).
        hz: Control loop frequency [Hz].
    """
    # Lazy imports keep --help fast even without deps.
    from lerobot.robots.fairino.config_fairino_follower import (
        FairinoFollowerConfig,
    )
    from lerobot.robots.fairino.fairino_follower import (
        FairinoFollower,
    )
    from lerobot.teleoperators.keyboard.configuration_keyboard import (
        KeyboardFairinoTeleopConfig,
    )
    from lerobot.teleoperators.keyboard.teleop_keyboard_fairino import (
        KeyboardFairinoTeleop,
    )

    robot_cfg = FairinoFollowerConfig(
        id="fr5", ip_address=ip, move_speed=speed,
    )
    robot = FairinoFollower(robot_cfg)

    teleop_cfg = KeyboardFairinoTeleopConfig(
        id="kb_fairino", step_size_deg=step_size,
    )
    teleop = KeyboardFairinoTeleop(teleop_cfg)

    try:
        robot.connect()
    except Exception as exc:
        logger.error("Cannot connect to %s: %s", ip, exc)
        sys.exit(1)

    teleop.connect()
    _print_banner()

    # Initialise teleop targets from actual joint state.
    obs = robot.get_observation()
    teleop.send_feedback(obs)

    print(_format_joints(
        [obs[f"{j}.pos"] for j in JOINT_NAMES],
        0, step_size,
    ))

    period = 1.0 / hz

    try:
        while True:
            t_start = time.perf_counter()

            if teleop.quit_requested:
                logger.info("Quit requested.")
                break
            if teleop.reset_requested:
                logger.info("Resetting robot errors ...")
                robot.reset_errors()

            action = teleop.get_action()
            sent = robot.send_action(action)

            obs = robot.get_observation()
            teleop.send_feedback(obs)

            joints = [
                sent[f"{j}.pos"] for j in JOINT_NAMES
            ]
            for _ in range(_DISPLAY_LINES):
                sys.stdout.write(_UP + _CLR)
            print(_format_joints(
                joints,
                teleop.selected_joint,
                teleop.step_size,
            ))

            remaining = period - (
                time.perf_counter() - t_start
            )
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        logger.info("Interrupted (Ctrl+C).")
    finally:
        teleop.disconnect()
        robot.disconnect()
        print(f"\n{_GRN}Teleoperation ended.{_RST}")


def main():
    """Parse CLI arguments and launch teleop."""
    ap = argparse.ArgumentParser(
        description="Keyboard teleop for Fairino FR5",
    )
    ap.add_argument(
        "--ip", default="192.168.58.2",
        help="Controller IP (default: 192.168.58.2)",
    )
    ap.add_argument(
        "--step", type=float, default=1.0,
        help="Initial step size [deg] (default: 1.0)",
    )
    ap.add_argument(
        "--speed", type=float, default=0.1,
        help="Move speed 0.0-1.0 (default: 0.1)",
    )
    ap.add_argument(
        "--hz", type=float, default=20.0,
        help="Control loop Hz (default: 20.0)",
    )
    args = ap.parse_args()
    run_teleop(args.ip, args.step, args.speed, args.hz)


if __name__ == "__main__":
    main()
