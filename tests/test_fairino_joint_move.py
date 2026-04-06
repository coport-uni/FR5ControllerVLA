#!/usr/bin/env python
"""Move Fairino FR5 joint1 by +10 deg and verify.

Usage::

    python tests/test_fairino_joint_move.py
    python tests/test_fairino_joint_move.py --ip 192.168.58.2
"""

import argparse
import sys
import time

JOINT_NAMES = [
    "joint1", "joint2", "joint3",
    "joint4", "joint5", "joint6",
]
MOVE_DEG = 10.0       # [deg] target displacement
TOLERANCE_DEG = 1.5   # [deg] acceptable error
N_STEPS = 50          # number of ServoJ increments


def main():
    """Connect, move joint1, verify, disconnect."""
    parser = argparse.ArgumentParser(
        description="Test joint1 move by 10 degrees",
    )
    parser.add_argument(
        "--ip", default="192.168.58.2", help="Robot IP",
    )
    args = parser.parse_args()

    from lerobot.robots.fairino.config_fairino_follower import (
        FairinoFollowerConfig,
    )
    from lerobot.robots.fairino.fairino_follower import (
        FairinoFollower,
    )

    cfg = FairinoFollowerConfig(
        id="fr5_test",
        ip_address=args.ip,
        move_speed=0.15,
    )
    robot = FairinoFollower(cfg)

    print(f"[1/5] Connecting to {args.ip} ...")
    robot.connect()
    print("      Connected (servo mode active).")

    print("[2/5] Reading initial joint positions ...")
    obs_before = robot.get_observation()
    joints_before = [
        obs_before[f"{j}.pos"] for j in JOINT_NAMES
    ]
    print(f"      Before: "
          f"{[f'{v:.2f}' for v in joints_before]}")

    # Move joint1 in small ServoJ increments.
    print(
        f"[3/5] Moving joint1 by +{MOVE_DEG} deg "
        f"({N_STEPS} steps) ..."
    )
    step_deg = MOVE_DEG / N_STEPS
    current = list(joints_before)
    for _ in range(N_STEPS):
        current[0] += step_deg
        action = {
            f"{JOINT_NAMES[j]}.pos": current[j]
            for j in range(6)
        }
        robot.send_action(action)
        time.sleep(1.0 / cfg.control_hz)

    print("[4/5] Waiting for motion to settle ...")
    time.sleep(1.0)

    print("[5/5] Reading final joint positions ...")
    obs_after = robot.get_observation()
    joints_after = [
        obs_after[f"{j}.pos"] for j in JOINT_NAMES
    ]
    print(f"      After:  "
          f"{[f'{v:.2f}' for v in joints_after]}")

    # Verify the displacement.
    diff = joints_after[0] - joints_before[0]
    print()
    print("=== Result ===")
    print(f"  joint1 before : {joints_before[0]:.2f} deg")
    print(f"  joint1 after  : {joints_after[0]:.2f} deg")
    print(f"  difference    : {diff:.2f} deg "
          f"(expected: {MOVE_DEG:.2f} deg)")

    if abs(diff - MOVE_DEG) <= TOLERANCE_DEG:
        print(
            f"  PASS -- moved {diff:.2f} deg "
            f"(within +/-{TOLERANCE_DEG} deg)"
        )
    else:
        print(
            f"  FAIL -- expected ~{MOVE_DEG} deg, "
            f"got {diff:.2f} deg"
        )
        robot.disconnect()
        sys.exit(1)

    # Check that other joints stayed still.
    for i in range(1, 6):
        other_diff = abs(joints_after[i] - joints_before[i])
        if other_diff > TOLERANCE_DEG:
            print(
                f"  WARNING: {JOINT_NAMES[i]} moved "
                f"{other_diff:.2f} deg (unexpected)"
            )

    robot.disconnect()
    print("\nDone. Robot disconnected.")


if __name__ == "__main__":
    main()
