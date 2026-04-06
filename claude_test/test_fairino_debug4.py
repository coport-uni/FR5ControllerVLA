#!/usr/bin/env python
"""
Comprehensive Fairino FR5 diagnostic & movement test.

Tries multiple approaches:
  1. Mode(0) + RobotEnable(1) + MoveJ
  2. If mode switch fails, try StartJOG (works in manual mode on some FW)
  3. Report all robot state for user diagnosis
"""
import sys
import time
sys.path.insert(0, "src")

from lerobot.robots.fairino.fairino.Robot import RPC

ip = "192.168.58.2"
print(f"=== Fairino FR5 Diagnostic ===")
print(f"Connecting to {ip} ...")
robot = RPC(ip)
time.sleep(1.5)  # Give state thread time to populate

# ─── Full state dump ───
pkg = robot.robot_state_pkg
print(f"\n--- Robot State ---")
print(f"  robot_mode:     {pkg.robot_mode}  (0=auto, 1=manual)")
print(f"  robot_state:    {pkg.robot_state}  (1=stop, 2=run, 3=pause, 4=drag)")
print(f"  program_state:  {pkg.program_state}  (1=stop, 2=run, 3=pause)")
print(f"  main_code:      {pkg.main_code}")
print(f"  sub_code:       {pkg.sub_code}")
print(f"  EmergencyStop:  {pkg.EmergencyStop}")

ret, joints = robot.GetActualJointPosDegree()
print(f"\n  Joints (deg): {[f'{j:.2f}' for j in joints]}")

ret2, tcp = robot.GetActualTCPPose()
print(f"  TCP (mm/deg): {[f'{v:.2f}' for v in tcp]}")

# Check SDK version
try:
    err, ver = robot.GetSDKVersion()
    print(f"  SDK version: {ver}")
except:
    print("  SDK version: (could not retrieve)")

# ─── Step 1: Try reset + enable + auto mode ───
print(f"\n--- Step 1: ResetAllError + RobotEnable + Mode(0) ---")
r1 = robot.ResetAllError()
print(f"  ResetAllError: {r1}")
time.sleep(0.5)

r2 = robot.RobotEnable(1)
print(f"  RobotEnable(1): {r2}")
time.sleep(0.5)

r3 = robot.Mode(0)
print(f"  Mode(0): {r3}")
time.sleep(1)

print(f"  robot_mode after: {pkg.robot_mode}")
print(f"  main_code: {pkg.main_code}, sub_code: {pkg.sub_code}")

# ─── Step 2: Try MoveJ ───
print(f"\n--- Step 2: MoveJ (joint1 +10 deg) ---")
ret, j0 = robot.GetActualJointPosDegree()
target = list(j0)
target[0] += 10.0
print(f"  Current: {[f'{v:.2f}' for v in j0]}")
print(f"  Target:  {[f'{v:.2f}' for v in target]}")

r4 = robot.MoveJ(target, 0, 0, vel=20.0, blendT=-1.0)
print(f"  MoveJ result: {r4}")
time.sleep(2)

ret, j1 = robot.GetActualJointPosDegree()
delta = j1[0] - j0[0]
print(f"  After:   {[f'{v:.2f}' for v in j1]}")
print(f"  Delta joint1: {delta:.2f} deg")

if abs(delta) > 1.0:
    print(f"  >>> MoveJ SUCCESS!")
else:
    print(f"  >>> MoveJ did NOT move. Error code {r4}")

    # ─── Step 3: Try StartJOG ───
    print(f"\n--- Step 3: Try StartJOG ---")
    # StartJOG(ref, nb, dir, max_dis, vel, acc)
    # ref: 0=base/world, 2=joint, 4=tool, 8=world cartesian
    # nb: joint number 1-6 (0=stop all)
    # dir: 0=positive, 1=negative

    for ref_val in [0, 2, 4, 8]:
        ret, j_before = robot.GetActualJointPosDegree()
        r = robot.StartJOG(ref_val, 1, 0, 10.0, vel=20.0, acc=100.0)
        print(f"  StartJOG(ref={ref_val}, nb=1, dir=0, max_dis=10): ret={r}")
        time.sleep(2)
        robot.StopJOG(ref_val)
        time.sleep(0.5)
        ret, j_after = robot.GetActualJointPosDegree()
        d = j_after[0] - j_before[0]
        print(f"    Delta joint1: {d:.2f} deg")
        if abs(d) > 1.0:
            print(f"    >>> JOG with ref={ref_val} SUCCESS!")
            break

    # ─── Step 4: Check if we need to exit drag teach mode ───
    print(f"\n--- Step 4: Check drag teach ---")
    try:
        err, is_drag = robot.IsInDragTeach()
        print(f"  IsInDragTeach: err={err}, is_drag={is_drag}")
        if is_drag == 1:
            print("  Robot is in drag-teach mode! Exiting ...")
            r_dt = robot.DragTeachSwitch(0)
            print(f"  DragTeachSwitch(0): {r_dt}")
            time.sleep(1)
            # Retry Mode + MoveJ
            robot.Mode(0)
            time.sleep(1)
            r5 = robot.MoveJ(target, 0, 0, vel=20.0, blendT=-1.0)
            print(f"  MoveJ after exiting drag: {r5}")
            time.sleep(2)
            ret, j_final = robot.GetActualJointPosDegree()
            print(f"  Final joints: {[f'{v:.2f}' for v in j_final]}")
    except Exception as e:
        print(f"  IsInDragTeach check failed: {e}")

print(f"\n--- Diagnosis ---")
print(f"  robot_mode={pkg.robot_mode}, robot_state={pkg.robot_state}")
if pkg.robot_mode == 1:
    print("  >> Robot is in MANUAL mode.")
    print("  >> Please switch the teach pendant / controller to AUTO mode.")
    print("  >> Then re-run this test.")
    print("  >> (On FR5: physical key switch or web interface at http://<IP>)")

robot.CloseRPC()
print("\nDone.")
