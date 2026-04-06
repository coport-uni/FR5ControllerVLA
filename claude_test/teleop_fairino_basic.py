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
Keyboard teleoperation for Fairino FR5 using LeRobot's built-in KeyboardTeleop.

This script uses the basic KeyboardTeleop (pynput-based) from LeRobot to read
keyboard input, then interprets those key presses as joint-space commands for
the Fairino robot.

Usage:
    python -m lerobot.scripts.teleop_fairino_basic --ip 192.168.58.2
    python -m lerobot.scripts.teleop_fairino_basic --ip 192.168.58.2 --step 2.0 --speed 0.3

Keyboard controls:
    1-6         Select active joint
    w           Increase selected joint angle by step_size
    s           Decrease selected joint angle by step_size
    a           Previous joint (selected_joint - 1)
    d           Next joint (selected_joint + 1)
    =           Increase step size (+0.5 deg)
    -           Decrease step size (-0.5 deg)
    p           Print current joint positions & TCP pose
    r           Reset robot errors and re-enable
    q           Quit
    Esc         Quit (handled by KeyboardTeleop internally)
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

# ANSI terminal helpers
CLEAR_LINE = "\033[2K"
MOVE_UP = "\033[A"
BOLD = "\033[1m"
RST = "\033[0m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"


def print_banner():
    print(f"""
{BOLD}╔══════════════════════════════════════════════════════════╗
║     Fairino FR5 Teleop (LeRobot KeyboardTeleop)         ║
╠══════════════════════════════════════════════════════════╣
║  1-6        Select joint                                 ║
║  w          Joint angle +        s   Joint angle -       ║
║  a          Prev joint           d   Next joint          ║
║  = (+)      Step size +          -   Step size -         ║
║  p          Print state          r   Reset errors        ║
║  q / Esc    Quit                                         ║
╚══════════════════════════════════════════════════════════╝{RST}
""")


def format_joints(joints_deg: list[float], selected: int, step: float) -> str:
    lines = [f"  Step: {CYAN}{step:.1f}{RST} deg"]
    lines.append("")
    for i, (name, val) in enumerate(zip(JOINT_NAMES, joints_deg)):
        marker = f"{GREEN}▶{RST}" if i == selected else " "
        lines.append(f"  {marker} {name}: {val:>8.2f}°")
    return "\n".join(lines)


DISPLAY_LINES = len(JOINT_NAMES) + 3  # step line + blank + 6 joints


def run_teleop(ip: str, step_size: float, speed: float, hz: float):
    # Lazy imports
    from lerobot.robots.fairino.config_fairino_follower import FairinoFollowerConfig
    from lerobot.robots.fairino.fairino_follower import FairinoFollower
    from lerobot.teleoperators.keyboard.configuration_keyboard import KeyboardTeleopConfig
    from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop

    # ── Robot setup ──
    robot_cfg = FairinoFollowerConfig(id="fr5", ip_address=ip, move_speed=speed)
    robot = FairinoFollower(robot_cfg)

    # ── Use LeRobot's built-in KeyboardTeleop ──
    teleop_cfg = KeyboardTeleopConfig(id="kb_basic")
    teleop = KeyboardTeleop(teleop_cfg)

    # ── Connect ──
    try:
        robot.connect()
    except Exception as e:
        logger.error(f"Failed to connect to robot at {ip}: {e}")
        sys.exit(1)

    teleop.connect()

    print_banner()

    # State
    current_joints = robot.get_joint_positions_deg()
    selected_joint = 0
    logger.info(f"Initial joints (deg): {[f'{j:.2f}' for j in current_joints]}")

    print(format_joints(current_joints, selected_joint, step_size))

    period = 1.0 / hz

    try:
        while True:
            loop_start = time.perf_counter()

            # ── Read keyboard via LeRobot's basic KeyboardTeleop ──
            # Returns dict like {"w": None, "1": None, ...} for currently pressed keys
            if not teleop.is_connected:
                logger.info("Teleop disconnected (ESC pressed).")
                break

            raw_action = teleop.get_action()
            pressed_keys = set(raw_action.keys())

            # ── Interpret keys ──
            has_movement = False
            request_print = False
            request_reset = False
            request_quit = False

            # Joint selection via number keys
            for i, digit in enumerate(["1", "2", "3", "4", "5", "6"]):
                if digit in pressed_keys:
                    selected_joint = i

            # Joint selection via a/d
            if "a" in pressed_keys:
                selected_joint = max(0, selected_joint - 1)
            if "d" in pressed_keys:
                selected_joint = min(5, selected_joint + 1)

            # Joint movement
            if "w" in pressed_keys:
                current_joints[selected_joint] += step_size
                has_movement = True
            if "s" in pressed_keys:
                current_joints[selected_joint] -= step_size
                has_movement = True

            # Step size adjustment
            if "=" in pressed_keys:
                step_size = min(step_size + 0.5, 30.0)
            if "-" in pressed_keys:
                step_size = max(step_size - 0.5, 0.1)

            # Utility
            if "p" in pressed_keys:
                request_print = True
            if "r" in pressed_keys:
                request_reset = True
            if "q" in pressed_keys:
                request_quit = True

            # ── Handle commands ──
            if request_quit:
                logger.info("Quit requested.")
                break

            if request_reset:
                logger.info("Resetting robot errors ...")
                robot.reset_errors()

            if request_print:
                tcp = robot.get_tcp_pose()
                print(f"\n{YELLOW}── Current State ──{RST}")
                print(f"  Joints (deg): {[f'{j:.2f}' for j in current_joints]}")
                print(f"  TCP (mm/deg): {[f'{v:.2f}' for v in tcp]}")
                print(f"{YELLOW}───────────────────{RST}\n")
                print(format_joints(current_joints, selected_joint, step_size))
                continue

            # ── Send movement to robot ──
            if has_movement:
                robot_action = {
                    f"{jname}.pos": current_joints[i]
                    for i, jname in enumerate(JOINT_NAMES)
                }
                sent = robot.send_action(robot_action)

                # Update to clamped values
                for i, jname in enumerate(JOINT_NAMES):
                    current_joints[i] = sent[f"{jname}.pos"]

            # ── Update display ──
            for _ in range(DISPLAY_LINES):
                sys.stdout.write(MOVE_UP + CLEAR_LINE)
            print(format_joints(current_joints, selected_joint, step_size))

            # ── Rate limit ──
            elapsed = time.perf_counter() - loop_start
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        logger.info("Interrupted (Ctrl+C).")
    finally:
        teleop.disconnect()
        robot.disconnect()
        print(f"\n{GREEN}Teleoperation ended.{RST}")


def main():
    parser = argparse.ArgumentParser(
        description="Fairino FR5 teleop using LeRobot's built-in KeyboardTeleop",
    )
    parser.add_argument("--ip", type=str, default="192.168.58.2",
                        help="Fairino controller IP (default: 192.168.58.2)")
    parser.add_argument("--step", type=float, default=1.0,
                        help="Initial step size in degrees (default: 1.0)")
    parser.add_argument("--speed", type=float, default=0.1,
                        help="Movement speed 0.0-1.0 (default: 0.1)")
    parser.add_argument("--hz", type=float, default=20.0,
                        help="Control loop Hz (default: 20.0)")
    args = parser.parse_args()

    run_teleop(ip=args.ip, step_size=args.step, speed=args.speed, hz=args.hz)


if __name__ == "__main__":
    main()
