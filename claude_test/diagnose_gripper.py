#!/usr/bin/env python
"""Diagnose gripper control on the real Fairino FR5.

Tests gripper in three modes:
  1. Direct XMLRPC (no SDK, no servo)
  2. Through SDK (no servo)
  3. While ServoJ is running (servo mode)

Run on the real robot:
    python claude_test/diagnose_gripper.py --ip 192.168.58.2
"""

import argparse
import socket
import time
import xmlrpc.client


def test_direct_xmlrpc(ip):
    """Test 1: raw XMLRPC, no SDK, no servo mode."""
    print("\n=== TEST 1: Direct XMLRPC (no servo) ===")
    link = f"http://{ip}:20003"
    proxy = xmlrpc.client.ServerProxy(link)

    # Check connection
    try:
        socket.setdefaulttimeout(2)
        proxy.GetControllerIP()
        socket.setdefaulttimeout(None)
        print("  XMLRPC connection OK")
    except Exception as e:
        print(f"  XMLRPC connection FAILED: {e}")
        return False

    # Configure gripper (try common configs)
    configs = [
        (4, 0, "大寰 PGI-140"),
        (3, 0, "天机 TEG-110"),
        (1, 0, "Robotiq 2F-85"),
    ]

    for company, device, name in configs:
        print(f"\n  Trying SetGripperConfig({company}, {device}) "
              f"= {name}")
        try:
            ret = proxy.SetGripperConfig(
                company, device, 0, 0,
            )
            print(f"    ret={ret}")
            if ret == 0:
                break
        except Exception as e:
            print(f"    EXCEPTION: {e}")
    else:
        print("  WARNING: no config succeeded, "
              "continuing anyway")

    # Read current config
    try:
        ret = proxy.GetGripperConfig()
        print(f"  GetGripperConfig: {ret}")
    except Exception as e:
        print(f"  GetGripperConfig EXCEPTION: {e}")

    # Activate
    print("\n  ActGripper(1, 0) = reset")
    try:
        ret = proxy.ActGripper(1, 0)
        print(f"    ret={ret}")
    except Exception as e:
        print(f"    EXCEPTION: {e}")
    time.sleep(1.0)

    print("  ActGripper(1, 1) = activate")
    try:
        ret = proxy.ActGripper(1, 1)
        print(f"    ret={ret}")
    except Exception as e:
        print(f"    EXCEPTION: {e}")
    time.sleep(2.0)

    # Move gripper
    for target in [50, 0, 100, 50]:
        print(f"\n  MoveGripper -> {target}% "
              "(vel=50, force=50, non-blocking)")
        try:
            ret = proxy.MoveGripper(
                1, target, 50, 50,
                30000, 1, 0, 0.0, 0, 0,
            )
            print(f"    ret={ret}")
        except Exception as e:
            print(f"    EXCEPTION: {e}")
        time.sleep(2.0)

    # Deactivate
    try:
        proxy.ActGripper(1, 0)
    except Exception:
        pass

    print("\n  TEST 1 COMPLETE")
    return True


def test_with_sdk(ip):
    """Test 2: through SDK, no servo mode."""
    print("\n=== TEST 2: Through SDK (no servo) ===")
    try:
        from lerobot.robots.fairino.fairino.Robot import (
            RPC as FairinoRPC,
        )
    except ImportError:
        print("  SKIP: cannot import SDK")
        return False

    rpc = FairinoRPC(ip)
    time.sleep(1.5)

    print("  SDK connected")

    print("  SetGripperConfig(4, 0, 0, 0)")
    try:
        ret = rpc.SetGripperConfig(4, 0, 0, 0)
        print(f"    ret={ret}")
    except Exception as e:
        print(f"    EXCEPTION: {e}")

    print("  ActGripper(1, 0) reset")
    try:
        ret = rpc.ActGripper(1, 0)
        print(f"    ret={ret}")
    except Exception as e:
        print(f"    EXCEPTION: {e}")
    time.sleep(1.0)

    print("  ActGripper(1, 1) activate")
    try:
        ret = rpc.ActGripper(1, 1)
        print(f"    ret={ret}")
    except Exception as e:
        print(f"    EXCEPTION: {e}")
    time.sleep(2.0)

    # Move via SDK method (has GetSafetyCode check)
    print("\n  MoveGripper via SDK method -> 50%")
    try:
        ret = rpc.MoveGripper(
            1, 50, 50, 50, 30000, 1, 0, 0.0, 0, 0,
        )
        print(f"    SDK ret={ret}")
    except Exception as e:
        print(f"    SDK EXCEPTION: {e}")
    time.sleep(2.0)

    # Move via direct XMLRPC (bypasses safety)
    print("  MoveGripper via XMLRPC direct -> 0%")
    try:
        ret = rpc.robot.MoveGripper(
            1, 0, 50, 50, 30000, 1, 0, 0.0, 0, 0,
        )
        print(f"    XMLRPC ret={ret}")
    except Exception as e:
        print(f"    XMLRPC EXCEPTION: {e}")
    time.sleep(2.0)

    # Read position
    try:
        ret = rpc.GetGripperCurPosition()
        print(f"  GetGripperCurPosition: {ret}")
    except Exception as e:
        print(f"  GetGripperCurPosition EXCEPTION: {e}")

    rpc.ActGripper(1, 0)
    rpc.CloseRPC()
    print("\n  TEST 2 COMPLETE")
    return True


def test_with_servo(ip):
    """Test 3: gripper while ServoJ is running."""
    print("\n=== TEST 3: Gripper during ServoJ ===")
    link = f"http://{ip}:20003"
    main_proxy = xmlrpc.client.ServerProxy(link)
    servo_proxy = xmlrpc.client.ServerProxy(link)
    grip_proxy = xmlrpc.client.ServerProxy(link)

    # Read current joints
    try:
        result = main_proxy.GetActualJointPosDegree(1)
        if result[0] != 0:
            print(f"  Joint read failed: {result}")
            return False
        joints = list(result[1:7])
        print(f"  Current joints: {joints}")
    except Exception as e:
        print(f"  Joint read EXCEPTION: {e}")
        return False

    # Init servo mode
    print("  Initialising servo mode ...")
    try:
        main_proxy.ServoMoveEnd()
    except Exception:
        pass
    time.sleep(0.2)
    main_proxy.RobotEnable(0)
    time.sleep(0.3)
    main_proxy.ResetAllError()
    time.sleep(0.3)
    main_proxy.RobotEnable(1)
    time.sleep(0.3)
    main_proxy.Mode(0)
    time.sleep(0.5)
    ret = main_proxy.ServoMoveStart()
    print(f"  ServoMoveStart ret={ret}")
    time.sleep(0.3)

    # Configure gripper
    print("  SetGripperConfig + ActGripper ...")
    main_proxy.SetGripperConfig(4, 0, 0, 0)
    time.sleep(0.3)
    main_proxy.ActGripper(1, 0)
    time.sleep(0.5)
    main_proxy.ActGripper(1, 1)
    time.sleep(2.0)

    # Start ServoJ hold-position loop
    import threading
    running = True

    def servo_loop():
        period = 0.02  # 50Hz
        while running:
            t0 = time.perf_counter()
            try:
                servo_proxy.ServoJ(
                    joints, [0.0, 0.0, 0.0, 0.0],
                    0.0, 0.0, period, 0.0, 0.0,
                )
            except Exception:
                pass
            elapsed = time.perf_counter() - t0
            remain = period - elapsed
            if remain > 0:
                time.sleep(remain)

    t = threading.Thread(target=servo_loop, daemon=True)
    t.start()
    print("  ServoJ loop running at 50Hz")
    time.sleep(0.5)

    # Now try gripper while servo is active
    for target in [80, 10, 50]:
        print(f"\n  MoveGripper -> {target}% "
              "(via separate proxy)")
        try:
            ret = grip_proxy.MoveGripper(
                1, target, 50, 50,
                30000, 1, 0, 0.0, 0, 0,
            )
            print(f"    ret={ret}")
        except Exception as e:
            print(f"    EXCEPTION: {e}")
        time.sleep(3.0)

    # Stop servo
    running = False
    t.join(timeout=2.0)
    try:
        main_proxy.ServoMoveEnd()
    except Exception:
        pass
    main_proxy.ActGripper(1, 0)

    print("\n  TEST 3 COMPLETE")
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.58.2")
    args = parser.parse_args()

    print(f"Fairino Gripper Diagnostic — {args.ip}")
    print("=" * 50)

    test_direct_xmlrpc(args.ip)
    time.sleep(1.0)
    test_with_sdk(args.ip)
    time.sleep(1.0)
    test_with_servo(args.ip)

    print("\n" + "=" * 50)
    print("DIAGNOSTIC COMPLETE")
    print("Check which tests show ret=0 and which show errors.")
    print("If Test 1/2 work but Test 3 fails, gripper commands")
    print("are blocked during servo mode on this firmware.")


if __name__ == "__main__":
    main()
