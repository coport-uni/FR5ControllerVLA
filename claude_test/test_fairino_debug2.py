#!/usr/bin/env python
"""Debug: test GetForwardKin and MoveJ with explicit desc_pos."""
import sys
import time
sys.path.insert(0, "src")

from lerobot.robots.fairino.fairino.Robot import RPC

ip = "192.168.58.2"
print(f"Connecting to {ip} ...")
robot = RPC(ip)
time.sleep(1)

# Get current state
ret, joints = robot.GetActualJointPosDegree()
print(f"Current joints: {joints}")

ret2, tcp = robot.GetActualTCPPose()
print(f"Current TCP: {tcp}")

# Test GetForwardKin
target = list(joints)
target[0] += 10.0
print(f"\nTarget joints: {target}")

fk_ret = robot.robot.GetForwardKin(target)
print(f"GetForwardKin result: {fk_ret}")

# If FK failed, try MoveJ with explicit desc_pos from FK or current TCP
# Method 1: compute FK ourselves or use a known good desc_pos
# Let's try passing the current TCP as desc_pos
print(f"\nTrying MoveJ with explicit desc_pos (current TCP) ...")
ret_move = robot.MoveJ(target, 0, 0, desc_pos=tcp, vel=20.0, acc=0.0, ovl=100.0, blendT=-1.0)
print(f"MoveJ with tcp desc_pos: {ret_move}")
time.sleep(2)

ret3, joints2 = robot.GetActualJointPosDegree()
print(f"After MoveJ: {joints2}")
print(f"Delta joint1: {joints2[0] - joints[0]:.2f} deg")

# If still failing, try calling XMLRPC MoveJ directly
if ret_move != 0:
    print(f"\nTrying XMLRPC MoveJ directly ...")
    # The SDK's robot.MoveJ has signature: MoveJ(joint_pos, desc_pos, tool, user, vel, acc, ovl, exaxis_pos, blendT, offset_flag, offset_pos)
    try:
        # First get the FK result for target
        fk = robot.robot.GetForwardKin(list(map(float, target)))
        print(f"FK for target: {fk}")
        if fk[0] == 0:
            desc = [fk[1], fk[2], fk[3], fk[4], fk[5], fk[6]]
        else:
            desc = list(map(float, tcp))

        direct_ret = robot.robot.MoveJ(
            list(map(float, target)),
            desc,
            0, 0,  # tool, user
            20.0, 0.0, 100.0,  # vel, acc, ovl
            [0.0, 0.0, 0.0, 0.0],  # exaxis_pos
            -1.0,  # blendT
            0,  # offset_flag
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # offset_pos
        )
        print(f"Direct XMLRPC MoveJ result: {direct_ret}")
    except Exception as e:
        print(f"Direct call failed: {e}")

    time.sleep(2)
    ret4, joints3 = robot.GetActualJointPosDegree()
    print(f"After direct MoveJ: {joints3}")
    print(f"Delta joint1: {joints3[0] - joints[0]:.2f} deg")

robot.CloseRPC()
print("Done.")
