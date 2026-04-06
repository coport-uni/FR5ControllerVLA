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
Keyboard teleoperation script for Fairino FR5 robot.

Usage:
    python -m lerobot.scripts.teleop_fairino --ip 192.168.58.2
    python -m lerobot.scripts.teleop_fairino --ip 192.168.58.2 --step 2.0 --speed 0.3

Keyboard controls:
    1-6         Select active joint
    W / Up      Increase selected joint angle
    S / Down    Decrease selected joint angle
    + / =       Increase step size
    - / _       Decrease step size
    P           Print current joint positions & TCP pose
    R           Reset robot errors and re-enable
    Q / Esc     Quit
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

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# ── ANSI helpers for terminal display ──────────────────────────────────
CLEAR_LINE = "\033[2K"
MOVE_UP = "\033[A"
BOLD = "\033[1m"
RESET_STYLE = "\033[0m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"


def print_banner():
    print(f"""
{BOLD}╔══════════════════════════════════════════════════════════╗
║          Fairino FR5 Keyboard Teleoperation              ║
╠══════════════════════════════════════════════════════════╣
║  1-6        Select joint                                 ║
║  W/↑        Joint angle +                                ║
║  S/↓        Joint angle -                                ║
║  +/=        Step size +    -  Step size -                ║
║  P          Print state    R  Reset errors               ║
║  Q/Esc      Quit                                         ║
╚══════════════════════════════════════════════════════════╝{RESET_STYLE}
""")


def format_joints_display(joints_deg: list[float], selected: int, step: float) -> str:
    lines = []
    lines.append(f"  Step size: {CYAN}{step:.1f}{RESET_STYLE} deg")
    lines.append("")
    for i, (name, val) in enumerate(zip(JOINT_NAMES, joints_deg)):
        marker = f"{GREEN}▶{RESET_STYLE}" if i == selected else " "
        lines.append(f"  {marker} {name}: {val:>8.2f}°")
    return "\n".join(lines)


def run_teleop(ip: str, step_size: float, speed: float, hz: float):
    from lerobot.robots.fairino.config_fairino_follower import FairinoFollowerConfig
    from lerobot.robots.fairino.fairino_follower import FairinoFollower
    from lerobot.teleoperators.keyboard.configuration_keyboard import KeyboardFairinoTeleopConfig
    from lerobot.teleoperators.keyboard.teleop_keyboard_fairino import KeyboardFairinoTeleop

    # ── Configure & connect robot ──
    robot_cfg = FairinoFollowerConfig(id="fr5", ip_address=ip, move_speed=speed)
    robot = FairinoFollower(robot_cfg)

    teleop_cfg = KeyboardFairinoTeleopConfig(id="kb_fairino", step_size_deg=step_size)
    teleop = KeyboardFairinoTeleop(teleop_cfg)

    try:
        robot.connect()
    except Exception as e:
        logger.error(f"Failed to connect to robot at {ip}: {e}")
        sys.exit(1)

    teleop.connect()

    print_banner()

    # Feed initial observation to teleop so it knows current joint positions
    obs = robot.get_observation()
    teleop.send_feedback(obs)

    display_lines = len(JOINT_NAMES) + 3
    print(format_joints_display(
        [obs[f"{j}.pos"] for j in JOINT_NAMES],
        0, step_size,
    ))

    period = 1.0 / hz

    try:
        while True:
            loop_start = time.perf_counter()

            # ── Check control flags ──
            if teleop.quit_requested:
                logger.info("Quit requested.")
                break

            if teleop.reset_requested:
                logger.info("Resetting robot errors ...")
                robot.reset_errors()

            # ── Get absolute target action from keyboard teleop ──
            action = teleop.get_action()

            # ── Send to robot ──
            sent = robot.send_action(action)

            # Feed back the observation so display stays up-to-date
            obs = robot.get_observation()
            teleop.send_feedback(obs)

            # ── Update terminal display ──
            current_joints = [sent[f"{j}.pos"] for j in JOINT_NAMES]
            for _ in range(display_lines):
                sys.stdout.write(MOVE_UP + CLEAR_LINE)
            print(format_joints_display(
                current_joints,
                teleop.selected_joint,
                teleop.step_size,
            ))

            # ── Rate limiting ──
            elapsed = time.perf_counter() - loop_start
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        logger.info("Interrupted by user (Ctrl+C).")
    finally:
        teleop.disconnect()
        robot.disconnect()
        print(f"\n{GREEN}Teleoperation ended.{RESET_STYLE}")


def main():
    parser = argparse.ArgumentParser(
        description="Keyboard teleoperation for Fairino FR5 robot",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--ip", type=str, default="192.168.58.2",
                        help="Fairino controller IP address (default: 192.168.58.2)")
    parser.add_argument("--step", type=float, default=1.0,
                        help="Initial joint step size in degrees (default: 1.0)")
    parser.add_argument("--speed", type=float, default=0.1,
                        help="Movement speed 0.0-1.0, maps to 0-100%% in SDK (default: 0.1)")
    parser.add_argument("--hz", type=float, default=20.0,
                        help="Control loop frequency in Hz (default: 20.0)")
    args = parser.parse_args()

    run_teleop(ip=args.ip, step_size=args.step, speed=args.speed, hz=args.hz)


if __name__ == "__main__":
    main()
