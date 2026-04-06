#!/usr/bin/env python
"""Full reset sequence then MoveJ test."""
import sys
import time
sys.path.insert(0, "src")

from lerobot.robots.fairino.fairino.Robot import RPC

ip = "192.168.58.2"
robot = RPC(ip)
time.sleep(1.5)

pkg = robot.robot_state_pkg

def status():
    print(f"  mode={pkg.robot_mode} state={pkg.robot_state} main={pkg.main_code} sub={pkg.sub_code} Estop={pkg.EmergencyStop}")

print("=== Initial state ===")
status()

# Step 1: Full reset sequence
print("\n=== Reset sequence ===")
print("ResetAllError ...")
r = robot.ResetAllError()
print(f"  ret={r}")
time.sleep(1)
status()

print("RobotEnable(0) - disable ...")
r = robot.RobotEnable(0)
print(f"  ret={r}")
time.sleep(1)
status()

print("RobotEnable(1) - enable ...")
r = robot.RobotEnable(1)
print(f"  ret={r}")
time.sleep(1)
status()

print("Mode(0) - auto ...")
r = robot.Mode(0)
print(f"  ret={r}")
time.sleep(2)
status()

# Extra reset after mode switch
print("ResetAllError again ...")
r = robot.ResetAllError()
print(f"  ret={r}")
time.sleep(1)
status()

print("RobotEnable(1) again ...")
r = robot.RobotEnable(1)
print(f"  ret={r}")
time.sleep(1)
status()

# Step 2: Try MoveJ
print("\n=== MoveJ test ===")
ret, joints = robot.GetActualJointPosDegree()
print(f"Current: {[f'{j:.2f}' for j in joints]}")

target = list(joints)
target[0] += 10.0

# Get FK desc_pos
fk = robot.robot.GetForwardKin(list(map(float, target)))
if fk[0] == 0:
    desc = [fk[1], fk[2], fk[3], fk[4], fk[5], fk[6]]
else:
    desc = [0.0] * 6

print(f"Target: {[f'{t:.2f}' for t in target]}")
print(f"FK desc: {[f'{d:.2f}' for d in desc]}")

# Try MoveJ with desc_pos
r_move = robot.MoveJ(target, 0, 0, desc_pos=desc, vel=20.0, blendT=-1.0)
print(f"MoveJ ret: {r_move}")
time.sleep(3)

ret2, joints2 = robot.GetActualJointPosDegree()
print(f"After:   {[f'{j:.2f}' for j in joints2]}")
print(f"Delta J1: {joints2[0] - joints[0]:.2f}")

if abs(joints2[0] - joints[0]) > 1.0:
    print(">>> SUCCESS!")

    # Move back
    r_back = robot.MoveJ(list(joints), 0, 0, vel=20.0, blendT=-1.0)
    print(f"Move back ret: {r_back}")
    time.sleep(3)
    ret3, joints3 = robot.GetActualJointPosDegree()
    print(f"Returned: {[f'{j:.2f}' for j in joints3]}")
else:
    print(">>> FAILED to move")
    print("\nPossible causes:")
    print("  1. Robot controller is in MANUAL mode (hardware switch)")
    print("  2. Robot has an active error (check teach pendant)")
    print("  3. E-stop is pressed")
    print("  4. Firmware requires specific initialization")
    print(f"\nCurrent: mode={pkg.robot_mode} main_code={pkg.main_code}")

robot.CloseRPC()
print("\nDone.")
