#!/usr/bin/env python
"""Direct XMLRPC test bypassing SDK wrapper."""
import sys
import time
sys.path.insert(0, "src")

from lerobot.robots.fairino.fairino.Robot import RPC

ip = "192.168.58.2"
robot = RPC(ip)
time.sleep(1.5)

pkg = robot.robot_state_pkg
print(f"robot_mode: {pkg.robot_mode}, main_code: {pkg.main_code}, sub_code: {pkg.sub_code}")

# Read current joints
ret, joints = robot.GetActualJointPosDegree()
print(f"Current joints: {[f'{j:.2f}' for j in joints]}")

target = list(joints)
target[0] += 10.0

# Test 1: GetForwardKin directly
print(f"\n--- GetForwardKin test ---")
fk_ret = robot.robot.GetForwardKin(list(map(float, target)))
print(f"GetForwardKin({[f'{v:.2f}' for v in target]})")
print(f"  Result: {fk_ret}")
print(f"  Error code: {fk_ret[0]}")

# If FK returns error 154, it's a kinematics error
if fk_ret[0] != 0:
    print(f"  FK FAILED with error {fk_ret[0]}")
    print(f"  This means the target position may be unreachable or ")
    print(f"  there's a robot configuration issue.")

    # Try passing desc_pos explicitly computed from current TCP
    ret2, tcp = robot.GetActualTCPPose()
    print(f"\n  Current TCP: {[f'{v:.2f}' for v in tcp]}")

    # Try direct XMLRPC MoveJ bypassing FK
    print(f"\n--- Direct XMLRPC MoveJ (bypass FK) ---")
    try:
        direct_ret = robot.robot.MoveJ(
            list(map(float, target)),     # joint_pos
            list(map(float, tcp)),        # desc_pos (use current TCP)
            0, 0,                         # tool, user
            20.0, 0.0, 100.0,            # vel, acc, ovl
            [0.0, 0.0, 0.0, 0.0],        # exaxis_pos
            -1.0,                         # blendT
            0,                            # offset_flag
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # offset_pos
        )
        print(f"  Direct MoveJ result: {direct_ret}")
    except Exception as e:
        print(f"  Direct MoveJ exception: {e}")
else:
    desc = [fk_ret[1], fk_ret[2], fk_ret[3], fk_ret[4], fk_ret[5], fk_ret[6]]
    print(f"  FK desc_pos: {[f'{v:.2f}' for v in desc]}")

    # Try MoveJ with FK result
    print(f"\n--- MoveJ with FK result ---")
    r = robot.MoveJ(target, 0, 0, desc_pos=desc, vel=20.0, blendT=-1.0)
    print(f"  MoveJ result: {r}")

# Test 2: Try smaller movement
print(f"\n--- Small movement test (joint1 +1 deg) ---")
small_target = list(joints)
small_target[0] += 1.0
fk2 = robot.robot.GetForwardKin(list(map(float, small_target)))
print(f"FK for +1 deg: error={fk2[0]}")

# Test 3: Check error codes with Mode
print(f"\n--- Mode switch detailed ---")
# Try Mode(0) again
r_mode = robot.robot.Mode(0)
print(f"Direct XMLRPC Mode(0): {r_mode}")
time.sleep(1)
print(f"robot_mode: {pkg.robot_mode}")

# Test 4: Check if RobotEnable is truly enabled
print(f"\n--- GetRobotEnableState ---")
try:
    r_en = robot.robot.GetControllerStatus()
    print(f"GetControllerStatus: {r_en}")
except Exception as e:
    print(f"GetControllerStatus error: {e}")

time.sleep(2)
ret3, joints3 = robot.GetActualJointPosDegree()
print(f"\nFinal joints: {[f'{j:.2f}' for j in joints3]}")

robot.CloseRPC()
print("Done.")
