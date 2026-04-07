#!/usr/bin/env python
"""Simulated keyboard teleoperation test for the Fairino FR5.

Mocks the Fairino RPC connection and injects keyboard events
to verify the full teleop pipeline without hardware.

Usage:
    python claude_test/test_fr5_keyboard_sim.py
"""

import logging
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

# ------------------------------------------------------------------
# Mock the Fairino SDK *before* any lerobot imports that pull it in.
# The SDK tries to open real sockets on import, so we intercept the
# RPC class at the module boundary.
# ------------------------------------------------------------------

_SIM_JOINTS = [0.0, -90.0, 0.0, 0.0, 0.0, 0.0]


def _make_mock_rpc(ip="192.168.58.2"):
    """Build a mock that behaves like the Fairino RPC object."""
    rpc = MagicMock()
    rpc.robot = MagicMock()

    # Simulated joint state (mutable so send_action updates it).
    joint_state = list(_SIM_JOINTS)

    def get_joints(flag=1):
        return (0, list(joint_state))

    rpc.GetActualJointPosDegree = get_joints
    rpc.robot.GetActualJointPosDegree = (
        lambda flag=1: [0] + list(joint_state)
    )
    rpc.GetActualTCPPose = lambda: (
        0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )

    rpc.RobotEnable = MagicMock(return_value=0)
    rpc.ResetAllError = MagicMock(return_value=0)
    rpc.Mode = MagicMock(return_value=0)
    rpc.ServoMoveStart = MagicMock(return_value=0)
    rpc.ServoMoveEnd = MagicMock(return_value=0)
    rpc.StopMotion = MagicMock(return_value=0)
    rpc.CloseRPC = MagicMock()
    rpc.robot.ServoJ = MagicMock(return_value=0)

    # Provide a fake robot_state_pkg that passes the
    # isinstance(pkg, Structure) check.
    import ctypes

    class FakeState(ctypes.Structure):
        _fields_ = [("jt_cur_pos", ctypes.c_double * 6)]

    state = FakeState()
    for i in range(6):
        state.jt_cur_pos[i] = joint_state[i]
    rpc.robot_state_pkg = state

    return rpc, joint_state


# ------------------------------------------------------------------
# Simulated keyboard event injector
# ------------------------------------------------------------------

def inject_keys(
    teleop, keys, delay_s=0.05,
):
    """Push key events into the teleoperator's event queue.

    Args:
        teleop: KeyboardFairinoTeleop instance.
        keys: Iterable of key strings (e.g. "1", "w", "UP").
        delay_s: Delay between injections [s].
    """
    for key in keys:
        time.sleep(delay_s)
        teleop.event_queue.put(key)


# ------------------------------------------------------------------
# Test harness
# ------------------------------------------------------------------

def run_test():
    """Run the simulated keyboard teleoperation test."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s  %(name)s  %(message)s",
    )
    log = logging.getLogger("test_fr5_keyboard_sim")

    # ------ build mocks ------
    mock_rpc, joint_state = _make_mock_rpc()

    # Patch FairinoRPC so the follower never touches HW.
    with patch(
        "lerobot.robots.fairino.fairino_follower.FairinoRPC",
        return_value=mock_rpc,
    ):
        from lerobot.robots.fairino.fairino_follower import (
            FairinoFollower,
        )
        from lerobot.robots.fairino.config_fairino_follower import (
            FairinoFollowerConfig,
        )
        from lerobot.teleoperators.keyboard.teleop_keyboard_fairino import (
            KeyboardFairinoTeleop,
        )
        from lerobot.teleoperators.keyboard.configuration_keyboard import (
            KeyboardFairinoTeleopConfig,
        )
        from lerobot.processor import make_default_processors

        # ------ create robot (patched) ------
        robot_cfg = FairinoFollowerConfig(
            ip_address="192.168.58.2",
        )
        robot = FairinoFollower(robot_cfg)

        # Bypass the real TCP probe, state-data wait,
        # and servo proxy creation.
        robot._probe_tcp = staticmethod(
            lambda ip, port, timeout_s=2.0: True
        )
        robot._wait_for_state_data = lambda timeout_s=3.0: None
        robot._create_servo_proxy = lambda: setattr(
            robot, "_servo_proxy", MagicMock(),
        )
        robot._rpc = mock_rpc

        robot.connect()
        log.info(
            "Robot connected (sim). Joints: %s",
            robot.get_joint_positions_deg(),
        )

        # ------ create keyboard teleop ------
        teleop_cfg = KeyboardFairinoTeleopConfig()
        teleop = KeyboardFairinoTeleop(teleop_cfg)

        # Connect but override the _StdinReader so it does
        # not touch the real terminal.
        teleop._reader = SimpleNamespace(
            is_alive=lambda: True,
            stop=lambda: None,
            join=lambda timeout=None: None,
        )

        # ------ processors (identity) ------
        (
            teleop_proc,
            robot_proc,
            obs_proc,
        ) = make_default_processors()

        # ------ define key sequence to simulate ------
        # Select joint 2, press W three times (increase),
        # then select joint 4, press S twice (decrease).
        key_sequence = [
            "2", "w", "w", "w",
            "4", "s", "s",
        ]

        # Inject keys on a background thread.
        injector = threading.Thread(
            target=inject_keys,
            args=(teleop, key_sequence, 0.08),
        )
        injector.start()

        # ------ run a short teleop loop ------
        step_size = teleop_cfg.step_size_deg
        num_iters = 15
        log.info(
            "Running %d teleop iterations "
            "(step_size=%.1f deg) ...",
            num_iters, step_size,
        )

        actions_log = []
        for i in range(num_iters):
            obs = robot.get_observation()
            teleop.send_feedback(obs)
            raw_action = teleop.get_action()
            action = teleop_proc((raw_action, obs))
            to_send = robot_proc((action, obs))
            robot.send_action(to_send)
            actions_log.append(dict(to_send))
            time.sleep(0.05)

        injector.join(timeout=3.0)

        # ------ verify results ------
        initial = dict(zip(
            robot_cfg.joint_names,
            _SIM_JOINTS,
        ))
        final = actions_log[-1]

        log.info("Initial joints: %s", initial)
        log.info("Final action:   %s", final)

        # Joint 2 should have increased by ~3 * step_size.
        j2_key = "joint2.pos"
        j2_expected = _SIM_JOINTS[1] + 3 * step_size
        j2_actual = final[j2_key]
        log.info(
            "joint2 expected ~%.1f, got %.1f",
            j2_expected, j2_actual,
        )
        assert abs(j2_actual - j2_expected) < 0.01, (
            f"joint2 mismatch: {j2_actual} != {j2_expected}"
        )

        # Joint 4 should have decreased by ~2 * step_size.
        j4_key = "joint4.pos"
        j4_expected = _SIM_JOINTS[3] - 2 * step_size
        j4_actual = final[j4_key]
        log.info(
            "joint4 expected ~%.1f, got %.1f",
            j4_expected, j4_actual,
        )
        assert abs(j4_actual - j4_expected) < 0.01, (
            f"joint4 mismatch: {j4_actual} != {j4_expected}"
        )

        # Other joints should be unchanged.
        unchanged = ["joint1", "joint3", "joint5", "joint6"]
        for jname in unchanged:
            key = f"{jname}.pos"
            idx = robot_cfg.joint_names.index(jname)
            expected = _SIM_JOINTS[idx]
            actual = final[key]
            assert abs(actual - expected) < 0.01, (
                f"{jname} changed unexpectedly: "
                f"{actual} != {expected}"
            )

        log.info("--- ALL ASSERTIONS PASSED ---")

        # ------ cleanup ------
        robot.disconnect()
        log.info("Robot disconnected.")


if __name__ == "__main__":
    try:
        run_test()
    except Exception as exc:
        logging.error("TEST FAILED: %s", exc, exc_info=True)
        sys.exit(1)
