"""FR5-to-FR5 leader-follower teleoperation test.

Reads leader joint positions (drag-teach mode) and sends them
to follower via ServoJ with velocity-limited interpolation.
Confirms follower reaches leader position.
"""

import logging
import socket
import sys
import time
import xmlrpc.client

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
)
logger = logging.getLogger(__name__)

LEADER_IP = "192.168.59.2"
FOLLOWER_IP = "192.168.58.2"

SERVO_HZ = 100
MAX_SERVO_SPEED = 60.0  # deg/s


def main():
    socket.setdefaulttimeout(None)
    sys.path.insert(
        0, "/workspace/FR5ControllerVLA/src",
    )
    from lerobot.robots.fairino.fairino.Robot import (
        RPC as FairinoRPC,
    )

    # ---- Connect leader ----
    logger.info("Connecting leader at %s ...", LEADER_IP)
    leader = FairinoRPC(LEADER_IP)
    time.sleep(1.5)

    leader.RobotEnable(0)
    time.sleep(0.3)
    leader.ResetAllError()
    time.sleep(0.3)
    leader.RobotEnable(1)
    time.sleep(0.3)
    ret = leader.DragTeachSwitch(1)
    logger.info("Leader DragTeachSwitch(1) -> %d", ret)
    time.sleep(0.5)

    # ---- Connect follower ----
    logger.info("Connecting follower at %s ...", FOLLOWER_IP)
    follower = FairinoRPC(FOLLOWER_IP)
    time.sleep(1.5)

    follower.ServoMoveEnd()
    time.sleep(0.2)
    follower.RobotEnable(0)
    time.sleep(0.3)
    follower.ResetAllError()
    time.sleep(0.3)
    follower.RobotEnable(1)
    time.sleep(0.5)
    follower.Mode(0)
    time.sleep(0.5)
    ret = follower.ServoMoveStart()
    logger.info("Follower ServoMoveStart -> %d", ret)
    time.sleep(0.3)

    servo_proxy = xmlrpc.client.ServerProxy(
        f"http://{FOLLOWER_IP}:20003",
    )

    try:
        # Read initial positions.
        _, leader_j = leader.GetActualJointPosDegree()
        _, follower_j = follower.GetActualJointPosDegree()
        leader_j = list(leader_j)
        follower_j = list(follower_j)

        logger.info(
            "Leader:   %s",
            [f"{j:.2f}" for j in leader_j],
        )
        logger.info(
            "Follower: %s",
            [f"{j:.2f}" for j in follower_j],
        )
        logger.info(
            "Diff:     %s",
            [
                f"J{i+1}:{abs(leader_j[i]-follower_j[i]):.1f}"
                for i in range(6)
            ],
        )

        # Move follower to leader position.
        logger.info(
            "Moving follower -> leader (interpolated) ..."
        )
        period = 1.0 / SERVO_HZ
        max_step = MAX_SERVO_SPEED * period
        axis_pos = [0.0, 0.0, 0.0, 0.0]
        commanded = list(follower_j)

        start = time.monotonic()
        count = 0
        errs = 0

        # Run for up to 10 seconds or until converged.
        while time.monotonic() - start < 10.0:
            t0 = time.perf_counter()

            converged = True
            for i in range(6):
                delta = leader_j[i] - commanded[i]
                if abs(delta) > max_step:
                    commanded[i] += (
                        max_step if delta > 0 else -max_step
                    )
                    converged = False
                else:
                    commanded[i] = leader_j[i]

            ret = servo_proxy.ServoJ(
                commanded, axis_pos,
                0.0, 0.0, period, 0.0, 0.0,
            )
            count += 1
            if ret != 0:
                errs += 1

            elapsed = time.perf_counter() - t0
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)

            if converged and count > 50:
                # Hold at target for 0.5s to stabilize.
                hold_end = time.monotonic() + 0.5
                while time.monotonic() < hold_end:
                    servo_proxy.ServoJ(
                        commanded, axis_pos,
                        0.0, 0.0, period, 0.0, 0.0,
                    )
                    count += 1
                    time.sleep(period)
                break

        elapsed_s = time.monotonic() - start
        logger.info(
            "Sent %d cmds in %.1fs (%d non-zero returns)",
            count, elapsed_s, errs,
        )

        # Verify.
        _, final_j = follower.GetActualJointPosDegree()
        final_j = list(final_j)
        errors = [
            abs(final_j[i] - leader_j[i])
            for i in range(6)
        ]
        max_err = max(errors)
        logger.info(
            "Final follower: %s",
            [f"{j:.2f}" for j in final_j],
        )
        logger.info(
            "Errors: %s (max=%.2f)",
            [
                f"J{i+1}:{e:.2f}"
                for i, e in enumerate(errors)
            ],
            max_err,
        )
        if max_err < 2.0:
            logger.info(
                "SUCCESS: follower reached leader "
                "position (max err %.2f deg)",
                max_err,
            )
        else:
            logger.warning(
                "INCOMPLETE: max error %.2f deg", max_err,
            )

    except KeyboardInterrupt:
        logger.info("Interrupted.")
    finally:
        logger.info("Cleaning up ...")
        leader.DragTeachSwitch(0)
        follower.ServoMoveEnd()
        logger.info("Done.")


if __name__ == "__main__":
    main()
