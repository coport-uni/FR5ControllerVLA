#!/usr/bin/env python
"""Direct gripper hardware test for the Fairino FR5.

Connects to the real robot, configures the gripper, and
cycles open/close while printing position feedback.

Usage:
    python claude_test/test_fr5_gripper_hw.py [--ip 192.168.58.2]
"""

import argparse
import sys
import time


def run_test(ip: str):
    from lerobot.robots.fairino.fairino.Robot import (
        RPC as FairinoRPC,
    )

    print(f"Connecting to {ip} ...")
    rpc = FairinoRPC(ip)
    time.sleep(1.5)

    # ---- configure gripper ----
    # Change company/device to match your gripper:
    #   1=Robotiq, 2=慧灵, 3=天机, 4=大寰, 5=知行
    company = 4
    device = 0
    index = 1

    print(f"SetGripperConfig(company={company}, device={device})")
    ret = rpc.SetGripperConfig(company, device)
    print(f"  -> ret={ret}")
    time.sleep(0.5)

    print("GetGripperConfig()")
    ret, cfg = rpc.GetGripperConfig()
    print(f"  -> ret={ret}, config={cfg}")

    # ---- reset + activate ----
    print(f"ActGripper(index={index}, action=0)  # reset")
    ret = rpc.ActGripper(index, 0)
    print(f"  -> ret={ret}")
    time.sleep(1.0)

    print(f"ActGripper(index={index}, action=1)  # activate")
    ret = rpc.ActGripper(index, 1)
    print(f"  -> ret={ret}")
    time.sleep(2.0)

    # ---- read initial position ----
    ret, fault, pos = rpc.GetGripperCurPosition()
    print(f"Initial position: {pos}%  (fault={fault})")

    # ---- cycle: open -> close -> open ----
    targets = [80, 10, 50]
    vel, force = 50, 50

    for target in targets:
        print(f"\nMoveGripper -> {target}% "
              f"(vel={vel}, force={force}, non-blocking)")

        # Direct XMLRPC call (bypasses SDK safety gate).
        ret = rpc.robot.MoveGripper(
            index, target, vel, force,
            30000,  # maxtime [ms]
            1,      # block: 1=non-blocking
            0,      # type=parallel
            0.0, 0, 0,
        )
        print(f"  -> ret={ret}")

        # Poll position for 3 seconds.
        for _ in range(30):
            time.sleep(0.1)
            ret2, fault2, cur_pos = (
                rpc.GetGripperCurPosition()
            )
            print(
                f"  pos={cur_pos}%  fault={fault2}",
                end="\r",
            )
        print()

        # Also check motion-done status.
        ret3, state = rpc.GetGripperMotionDone()
        print(f"  MotionDone: {state}")

    # ---- final read ----
    ret, fault, final_pos = rpc.GetGripperCurPosition()
    print(f"\nFinal position: {final_pos}%  (fault={fault})")

    # ---- cleanup ----
    print("Deactivating gripper ...")
    rpc.ActGripper(index, 0)
    time.sleep(0.5)
    rpc.CloseRPC()
    print("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ip", default="192.168.58.2",
    )
    args = parser.parse_args()
    try:
        run_test(args.ip)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)
