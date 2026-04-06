#!/usr/bin/env python
"""
Test: Move Fairino FR5 joint 0 (joint1) by +10 degrees and verify.

Usage:
    python tests/test_fairino_joint_move.py
    python tests/test_fairino_joint_move.py --ip 192.168.58.2
"""

import argparse
import sys
import time

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
MOVE_DEG = 10.0
TOLERANCE_DEG = 1.5  # acceptable error


def main():
    parser = argparse.ArgumentParser(description="Test joint1 move by 10 degrees")
    parser.add_argument("--ip", default="192.168.58.2", help="Robot IP")
    args = parser.parse_args()

    from lerobot.robots.fairino.config_fairino_follower import FairinoFollowerConfig
    from lerobot.robots.fairino.fairino_follower import FairinoFollower

    cfg = FairinoFollowerConfig(id="fr5_test", ip_address=args.ip, move_speed=0.15)
    robot = FairinoFollower(cfg)

    print(f"[1/5] Connecting to Fairino at {args.ip} ...")
    robot.connect()
    print("       Connected (servo mode active).")

    # Read initial joint positions
    print("[2/5] Reading initial joint positions ...")
    obs_before = robot.get_observation()
    joints_before = [obs_before[f"{j}.pos"] for j in JOINT_NAMES]
    print(f"       Before: {[f'{v:.2f}' for v in joints_before]}")

    # Move joint1 by +10 degrees via multiple small ServoJ steps
    target_joints = list(joints_before)
    target_joints[0] += MOVE_DEG
    print(f"[3/5] Moving joint1 by +{MOVE_DEG}° (target: {target_joints[0]:.2f}°) ...")

    # ServoJ works best with small incremental steps
    n_steps = 50
    step_deg = MOVE_DEG / n_steps
    current = list(joints_before)
    for i in range(n_steps):
        current[0] += step_deg
        action = {f"{JOINT_NAMES[j]}.pos": current[j] for j in range(6)}
        sent = robot.send_action(action)
        time.sleep(1.0 / cfg.control_hz)

    print(f"       Sent {n_steps} ServoJ commands.")

    # Wait for motion to settle
    print("[4/5] Waiting for motion to settle ...")
    time.sleep(1.0)

    # Read final joint positions
    print("[5/5] Reading final joint positions ...")
    obs_after = robot.get_observation()
    joints_after = [obs_after[f"{j}.pos"] for j in JOINT_NAMES]
    print(f"       After:  {[f'{v:.2f}' for v in joints_after]}")

    # Verify
    diff = joints_after[0] - joints_before[0]
    print()
    print(f"=== Result ===")
    print(f"  joint1 before: {joints_before[0]:.2f}°")
    print(f"  joint1 after:  {joints_after[0]:.2f}°")
    print(f"  difference:    {diff:.2f}° (expected: {MOVE_DEG:.2f}°)")

    if abs(diff - MOVE_DEG) <= TOLERANCE_DEG:
        print(f"  PASS -- joint1 moved by {diff:.2f}° (within +/-{TOLERANCE_DEG}° tolerance)")
    else:
        print(f"  FAIL -- expected ~{MOVE_DEG}°, got {diff:.2f}°")
        robot.disconnect()
        sys.exit(1)

    # Check other joints didn't move significantly
    for i in range(1, 6):
        other_diff = abs(joints_after[i] - joints_before[i])
        if other_diff > TOLERANCE_DEG:
            print(f"  WARNING: {JOINT_NAMES[i]} moved by {other_diff:.2f}° (unexpected)")

    robot.disconnect()
    print("\nDone. Robot disconnected.")


if __name__ == "__main__":
    main()
