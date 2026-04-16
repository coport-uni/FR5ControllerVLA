"""Debug leader-follower: read leader joints, send to follower via ServoJ."""

import contextlib
import sys
import time
import xmlrpc.client

LEADER_IP = "192.168.59.2"
FOLLOWER_IP = "192.168.58.2"


def make_proxy(ip):
    return xmlrpc.client.ServerProxy(f"http://{ip}:20003")


def read_joints(proxy, label):
    result = proxy.GetActualJointPosDegree(1)
    if result[0] != 0:
        print(f"[{label}] Joint read error: {result[0]}")
        return None
    joints = list(result[1:7])
    return joints


def init_follower_servo(proxy):
    """Full reset + servo-mode init for the follower."""
    print("[Follower] ServoMoveEnd (cleanup) ...")
    with contextlib.suppress(Exception):
        proxy.ServoMoveEnd()
    time.sleep(0.5)

    with contextlib.suppress(Exception):
        proxy.StopMotion()
    time.sleep(0.5)

    print("[Follower] RobotEnable(0) ...")
    ret = proxy.RobotEnable(0)
    print(f"  -> {ret}")
    time.sleep(1.0)

    print("[Follower] ResetAllError ...")
    ret = proxy.ResetAllError()
    print(f"  -> {ret}")
    time.sleep(1.0)

    print("[Follower] RobotEnable(1) ...")
    ret = proxy.RobotEnable(1)
    print(f"  -> {ret}")
    time.sleep(1.0)

    print("[Follower] Mode(0) ...")
    ret = proxy.Mode(0)
    print(f"  -> {ret}")
    time.sleep(1.0)

    print("[Follower] ServoMoveStart ...")
    ret = proxy.ServoMoveStart()
    print(f"  -> {ret}")
    time.sleep(0.5)

    return ret


def init_leader_drag(proxy):
    """Enable leader and enter drag-teach mode."""
    print("[Leader] RobotEnable(0) ...")
    ret = proxy.RobotEnable(0)
    print(f"  -> {ret}")
    time.sleep(0.3)

    print("[Leader] ResetAllError ...")
    ret = proxy.ResetAllError()
    print(f"  -> {ret}")
    time.sleep(0.3)

    print("[Leader] RobotEnable(1) ...")
    ret = proxy.RobotEnable(1)
    print(f"  -> {ret}")
    time.sleep(0.3)

    print("[Leader] DragTeachSwitch(1) ...")
    ret = proxy.DragTeachSwitch(1)
    print(f"  -> {ret}")
    time.sleep(0.2)

    return ret


def main():
    leader = make_proxy(LEADER_IP)
    follower = make_proxy(FOLLOWER_IP)

    # Step 1: Read joints from both robots.
    print("=" * 50)
    print("Step 1: Read current joint positions")
    print("=" * 50)

    leader_joints = read_joints(leader, "Leader")
    follower_joints = read_joints(follower, "Follower")

    if leader_joints is None or follower_joints is None:
        print("FAIL: Cannot read joints. Aborting.")
        sys.exit(1)

    print(f"  Leader  joints: {[f'{j:.2f}' for j in leader_joints]}")
    print(f"  Follower joints: {[f'{j:.2f}' for j in follower_joints]}")

    # Step 2: Init leader drag-teach mode.
    print()
    print("=" * 50)
    print("Step 2: Init leader drag-teach mode")
    print("=" * 50)
    init_leader_drag(leader)

    # Step 3: Init follower servo mode.
    print()
    print("=" * 50)
    print("Step 3: Init follower servo mode")
    print("=" * 50)
    ret = init_follower_servo(follower)
    if ret != 0:
        print(f"FAIL: ServoMoveStart returned {ret}")
        sys.exit(1)

    # Read follower's current position as commanded baseline.
    commanded = read_joints(follower, "Follower")
    if commanded is None:
        print("FAIL: Cannot read follower joints after servo init.")
        sys.exit(1)
    print(f"  Follower baseline: {[f'{j:.2f}' for j in commanded]}")

    # Step 4: Mirror leader -> follower for 10 seconds.
    print()
    print("=" * 50)
    print("Step 4: Leader -> Follower mirror (10s)")
    print("         Move the leader arm by hand!")
    print("=" * 50)

    servo_hz = 100.0
    period = 1.0 / servo_hz
    max_speed = 90.0  # deg/s
    max_step = max_speed * period
    axis_pos = [0.0, 0.0, 0.0, 0.0]

    duration = 10.0
    t0 = time.time()
    iteration = 0
    errors = 0

    try:
        while time.time() - t0 < duration:
            loop_start = time.perf_counter()

            # Read leader joints.
            target = read_joints(leader, "Leader")
            if target is None:
                errors += 1
                time.sleep(period)
                continue

            # Velocity-limited ramp toward target.
            for i in range(6):
                delta = target[i] - commanded[i]
                if abs(delta) > max_step:
                    commanded[i] += max_step if delta > 0 else -max_step
                else:
                    commanded[i] = target[i]

            # Send ServoJ.
            ret = follower.ServoJ(
                commanded,
                axis_pos,
                0.0,
                0.0,
                period,
                0.0,
                0.0,
            )

            if ret != 0:
                errors += 1
                if iteration < 5 or errors < 5:
                    print(f"  [iter {iteration}] ServoJ error: {ret}")

            iteration += 1

            # Maintain servo frequency.
            elapsed = time.perf_counter() - loop_start
            sleep_t = period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n  Interrupted by user.")

    print()
    print("=" * 50)
    print(f"Done. {iteration} iterations, {errors} errors.")
    print("=" * 50)

    # Read final positions.
    leader_final = read_joints(leader, "Leader")
    follower_final = read_joints(follower, "Follower")
    if leader_final and follower_final:
        print(f"  Leader  final: {[f'{j:.2f}' for j in leader_final]}")
        print(f"  Follower final: {[f'{j:.2f}' for j in follower_final]}")
        diffs = [abs(lj - fj) for lj, fj in zip(leader_final, follower_final, strict=True)]
        print(f"  Diffs (deg):   {[f'{d:.2f}' for d in diffs]}")
        print(f"  Max diff: {max(diffs):.2f} deg")

    # Cleanup.
    print("\nCleaning up ...")
    with contextlib.suppress(Exception):
        follower.ServoMoveEnd()
    with contextlib.suppress(Exception):
        leader.DragTeachSwitch(0)

    print("Done.")


if __name__ == "__main__":
    main()
