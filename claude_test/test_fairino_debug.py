#!/usr/bin/env python
"""Debug Fairino connection and MoveJ parameters."""
import sys
import time
sys.path.insert(0, "src")

from lerobot.robots.fairino.fairino.Robot import RPC

ip = "192.168.58.2"
print(f"Connecting to {ip} ...")
robot = RPC(ip)
time.sleep(1)  # Wait for state thread to initialize

# Check safety
safety = robot.GetSafetyCode()
print(f"SafetyCode: {safety}")

# Read state
ret, joints = robot.GetActualJointPosDegree()
print(f"GetActualJointPosDegree: ret={ret}, joints={joints}")

ret2, tcp = robot.GetActualTCPPose()
print(f"GetActualTCPPose: ret={ret2}, tcp={tcp}")

# Check robot mode and errors
pkg = robot.robot_state_pkg
print(f"robot_state: {pkg.robot_state}")
print(f"program_state: {pkg.program_state}")
print(f"main_code: {pkg.main_code}")
print(f"sub_code: {pkg.sub_code}")
print(f"robot_mode: {pkg.robot_mode}")

# Try ResetAllError + RobotEnable first
print("\nResetting errors and enabling robot ...")
ret_reset = robot.ResetAllError()
print(f"ResetAllError: {ret_reset}")
time.sleep(0.5)

ret_enable = robot.RobotEnable(1)
print(f"RobotEnable(1): {ret_enable}")
time.sleep(0.5)

# Check safety again
safety2 = robot.GetSafetyCode()
print(f"SafetyCode after reset: {safety2}")
print(f"main_code: {pkg.main_code}, sub_code: {pkg.sub_code}")

# Try MoveJ with correct parameters
ret3, joints2 = robot.GetActualJointPosDegree()
print(f"\nCurrent joints: {joints2}")

target = list(joints2)
target[0] += 10.0
print(f"Target: {target}")

# Try MoveJ directly with SDK defaults
print("\nCalling MoveJ ...")
ret_move = robot.MoveJ(target, 0, 0, vel=20.0, acc=0.0, ovl=100.0, blendT=-1.0)
print(f"MoveJ result: {ret_move}")

time.sleep(2)

ret4, joints3 = robot.GetActualJointPosDegree()
print(f"After MoveJ: {joints3}")
print(f"Delta joint1: {joints3[0] - joints2[0]:.2f} deg")

robot.CloseRPC()
print("Done.")
