#!/usr/bin/env python
"""
Test: Move Fairino FR5 joint1 by +10 degrees, then -10 degrees back.

Usage:
    python tests/test_fairino_joint1.py --ip 192.168.58.2

This script:
  1. Connects to the robot
  2. Reads current joint positions
  3. Moves joint1 +10 degrees
  4. Reads new joint positions to confirm
  5. Moves joint1 -10 degrees (back to original)
  6. Reads final positions to confirm
  7. Disconnects
"""

import argparse
import sys
import time

sys.path.insert(0, "src")


def main():
    parser = argparse.ArgumentParser(description="Test Fairino joint1 ±10 deg movement")
    parser.add_argument("--ip", type=str, default="192.168.58.2")
    parser.add_argument("--speed", type=float, default=0.1)
    args = parser.parse_args()

    from lerobot.robots.fairino.config_fairino_follower import FairinoFollowerConfig
    from lerobot.robots.fairino.fairino_follower import FairinoFollower

    JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    cfg = FairinoFollowerConfig(id="fr5_test", ip_address=args.ip, move_speed=args.speed)
    robot = FairinoFollower(cfg)

    print(f"[TEST] Connecting to {args.ip} ...")
    robot.connect()

    # Step 1: Read initial position
    initial = robot.get_joint_positions_deg()
    print(f"[TEST] Initial joints (deg): {[f'{v:.2f}' for v in initial]}")

    # Step 2: Move joint1 +10 degrees
    target = list(initial)
    target[0] += 10.0
    print(f"\n[TEST] Moving joint1 +10 deg → target: {target[0]:.2f}")
    action = {f"{jname}.pos": target[i] for i, jname in enumerate(JOINT_NAMES)}
    sent = robot.send_action(action)
    print(f"[TEST] Sent action: joint1.pos = {sent['joint1.pos']:.2f}")

    time.sleep(1.0)  # Wait for motion to settle

    after_plus = robot.get_joint_positions_deg()
    print(f"[TEST] After +10 deg: {[f'{v:.2f}' for v in after_plus]}")

    delta = after_plus[0] - initial[0]
    print(f"[TEST] joint1 actual delta: {delta:.2f} deg (expected ~10.00)")

    if abs(delta - 10.0) < 2.0:
        print("[TEST] ✓ +10 deg movement OK")
    else:
        print(f"[TEST] ✗ +10 deg movement FAILED (delta={delta:.2f})")

    # Step 3: Move joint1 -10 degrees (back to original)
    target2 = list(after_plus)
    target2[0] -= 10.0
    print(f"\n[TEST] Moving joint1 -10 deg → target: {target2[0]:.2f}")
    action2 = {f"{jname}.pos": target2[i] for i, jname in enumerate(JOINT_NAMES)}
    sent2 = robot.send_action(action2)
    print(f"[TEST] Sent action: joint1.pos = {sent2['joint1.pos']:.2f}")

    time.sleep(1.0)

    final = robot.get_joint_positions_deg()
    print(f"[TEST] After -10 deg: {[f'{v:.2f}' for v in final]}")

    delta_back = final[0] - initial[0]
    print(f"[TEST] joint1 net delta from start: {delta_back:.2f} deg (expected ~0.00)")

    if abs(delta_back) < 2.0:
        print("[TEST] ✓ Return to original OK")
    else:
        print(f"[TEST] ✗ Return to original FAILED (net delta={delta_back:.2f})")

    robot.disconnect()

    print("\n[TEST] === Summary ===")
    print(f"  Initial joint1: {initial[0]:.2f}")
    print(f"  After +10:      {after_plus[0]:.2f}")
    print(f"  After -10:      {final[0]:.2f}")
    print("[TEST] Done.")


if __name__ == "__main__":
    main()
