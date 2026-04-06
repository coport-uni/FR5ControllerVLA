#!/usr/bin/env python
"""Directly call XMLRPC MoveJ and analyze return."""
import sys
import time
import xmlrpc.client
sys.path.insert(0, "src")

from lerobot.robots.fairino.fairino.Robot import RPC

ip = "192.168.58.2"
robot = RPC(ip)
time.sleep(1.5)

pkg = robot.robot_state_pkg
print(f"mode={pkg.robot_mode} state={pkg.robot_state} main={pkg.main_code} sub={pkg.sub_code}")

ret, joints = robot.GetActualJointPosDegree()
print(f"Joints: {joints}")

target = list(map(float, joints))
target[0] += 10.0

# Get FK
fk = robot.robot.GetForwardKin(target)
desc = [fk[1], fk[2], fk[3], fk[4], fk[5], fk[6]]
print(f"FK ok: desc={desc}")

# Direct XMLRPC call
print("\n--- Direct XMLRPC self.robot.MoveJ ---")
try:
    result = robot.robot.MoveJ(
        target,           # joint_pos
        desc,             # desc_pos
        0,                # tool
        0,                # user
        20.0,             # vel
        0.0,              # acc
        100.0,            # ovl
        [0.0, 0.0, 0.0, 0.0],  # exaxis_pos
        -1.0,             # blendT
        0,                # offset_flag
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # offset_pos
    )
    print(f"XMLRPC MoveJ result: {result}")
    print(f"Type: {type(result)}")
except xmlrpc.client.Fault as f:
    print(f"Fault: {f}")
except Exception as e:
    print(f"Exception: {type(e).__name__}: {e}")

time.sleep(3)

# Check if status changed
print(f"\nmode={pkg.robot_mode} state={pkg.robot_state} main={pkg.main_code} sub={pkg.sub_code}")
ret2, joints2 = robot.GetActualJointPosDegree()
print(f"After: {[f'{j:.2f}' for j in joints2]}")

# List all available XMLRPC methods
print("\n--- Available XMLRPC methods (related to error/status) ---")
try:
    methods = robot.robot.system.listMethods()
    for m in sorted(methods):
        if any(kw in m.lower() for kw in ['error', 'status', 'mode', 'enable', 'state', 'fault', 'alarm']):
            print(f"  {m}")
except:
    print("  (listMethods not available)")

# Try to get error description
print("\n--- Try GetRobotErrorCode ---")
for method_name in ['GetRobotErrorCode', 'GetErrorCode', 'GetErrorInfo', 'GetRobotCurError']:
    try:
        result = getattr(robot.robot, method_name)()
        print(f"  {method_name}: {result}")
    except Exception as e:
        print(f"  {method_name}: {e}")

robot.CloseRPC()
print("\nDone.")
