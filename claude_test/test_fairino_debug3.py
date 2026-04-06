#!/usr/bin/env python
"""Debug: try Mode switch and StartJOG as fallback."""
import sys
import time
sys.path.insert(0, "src")

from lerobot.robots.fairino.fairino.Robot import RPC

ip = "192.168.58.2"
print(f"Connecting to {ip} ...")
robot = RPC(ip)
time.sleep(1)

pkg = robot.robot_state_pkg
print(f"robot_mode: {pkg.robot_mode} (0=auto, 1=manual)")
print(f"robot_state: {pkg.robot_state} (1=stop, 2=run, 3=pause, 4=drag)")

# Try switching to auto mode
print("\n--- Attempt 1: Switch to auto mode (Mode(0)) ---")
ret_mode = robot.Mode(0)
print(f"Mode(0) result: {ret_mode}")
time.sleep(1)
print(f"robot_mode after Mode(0): {pkg.robot_mode}")

if pkg.robot_mode == 0:
    # Now try MoveJ
    ret, joints = robot.GetActualJointPosDegree()
    target = list(joints)
    target[0] += 10.0
    print(f"Current joints: {joints}")
    print(f"Target joints: {target}")
    fk = robot.robot.GetForwardKin(list(map(float, target)))
    desc = [fk[1], fk[2], fk[3], fk[4], fk[5], fk[6]] if fk[0] == 0 else [0]*6
    ret_move = robot.MoveJ(target, 0, 0, desc_pos=desc, vel=20.0)
    print(f"MoveJ result: {ret_move}")
    time.sleep(2)
    ret2, joints2 = robot.GetActualJointPosDegree()
    print(f"After MoveJ: {joints2}")
    print(f"Delta joint1: {joints2[0] - joints[0]:.2f}")
else:
    print("Could not switch to auto mode. Trying JOG commands...")

    # StartJOG: ref=0(base), nb=joint_number(1-6), dir=0(+)/1(-), max_dis, vel, acc
    print("\n--- Attempt 2: StartJOG (joint1, +10 deg) ---")
    ret, joints = robot.GetActualJointPosDegree()
    print(f"Current joints: {joints}")

    # ref: 0=base, 2=joint space, 4=tool space, 8=world space
    # nb: 1-6 for joints
    # dir: 0=positive, 1=negative
    # max_dis: max distance in degrees
    print("StartJOG(ref=0, nb=1, dir=0, max_dis=10.0, vel=20.0) ...")
    ret_jog = robot.StartJOG(0, 1, 0, 10.0, vel=20.0, acc=100.0)
    print(f"StartJOG result: {ret_jog}")
    time.sleep(3)

    # Stop JOG
    ret_stop = robot.StopJOG(1)
    print(f"StopJOG result: {ret_stop}")
    time.sleep(0.5)

    ret3, joints3 = robot.GetActualJointPosDegree()
    print(f"After JOG: {joints3}")
    print(f"Delta joint1: {joints3[0] - joints[0]:.2f} deg")

    if abs(joints3[0] - joints[0]) < 0.5:
        # JOG with ref=0 didn't work, try ref=2
        print("\n--- Attempt 3: StartJOG with ref=2 (joint ref) ---")
        ret_jog2 = robot.StartJOG(2, 1, 0, 10.0, vel=20.0, acc=100.0)
        print(f"StartJOG(ref=2) result: {ret_jog2}")
        time.sleep(3)
        ret_stop2 = robot.StopJOG(1)
        print(f"StopJOG result: {ret_stop2}")
        time.sleep(0.5)
        ret4, joints4 = robot.GetActualJointPosDegree()
        print(f"After JOG ref=2: {joints4}")
        print(f"Delta joint1: {joints4[0] - joints[0]:.2f} deg")

robot.CloseRPC()
print("Done.")
