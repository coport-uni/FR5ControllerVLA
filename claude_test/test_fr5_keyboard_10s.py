#!/usr/bin/env python
"""10-second simulated keyboard+gripper teleoperation test.

Mocks the Fairino RPC and runs the full teleop pipeline for
10 seconds with mixed joint and gripper key events, verifying
no errors occur throughout.

Usage:
    python claude_test/test_fr5_keyboard_10s.py
"""

import logging
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

_SIM_JOINTS = [0.0, -90.0, 0.0, 0.0, 0.0, 0.0]


def _make_mock_rpc():
    rpc = MagicMock()
    rpc.robot = MagicMock()

    joint_state = list(_SIM_JOINTS)
    gripper_state = {"pos": 0.0}
    gripper_cmds = []

    def get_joints(flag=1):
        return (0, list(joint_state))

    rpc.GetActualJointPosDegree = get_joints
    rpc.robot.GetActualJointPosDegree = (
        lambda flag=1: [0] + list(joint_state)
    )

    def get_gripper_pos():
        return (0, 0, gripper_state["pos"])

    rpc.GetGripperCurPosition = get_gripper_pos

    def move_gripper(
        index, pos, vel, force,
        maxtime, block, gtype,
        rot_num, rot_vel, rot_torque,
    ):
        gripper_state["pos"] = float(pos)
        gripper_cmds.append(pos)
        return 0

    rpc.robot.MoveGripper = move_gripper

    rpc.SetGripperConfig = MagicMock(return_value=0)
    rpc.ActGripper = MagicMock(return_value=0)
    rpc.RobotEnable = MagicMock(return_value=0)
    rpc.ResetAllError = MagicMock(return_value=0)
    rpc.Mode = MagicMock(return_value=0)
    rpc.ServoMoveStart = MagicMock(return_value=0)
    rpc.ServoMoveEnd = MagicMock(return_value=0)
    rpc.StopMotion = MagicMock(return_value=0)
    rpc.CloseRPC = MagicMock()
    rpc.robot.ServoJ = MagicMock(return_value=0)
    rpc.GetActualTCPPose = lambda: (0, [0.0] * 6)

    import ctypes

    class FakeState(ctypes.Structure):
        _fields_ = [("jt_cur_pos", ctypes.c_double * 6)]

    state = FakeState()
    for i in range(6):
        state.jt_cur_pos[i] = joint_state[i]
    rpc.robot_state_pkg = state

    return rpc, gripper_state, gripper_cmds


def inject_keys(teleop, stop_event):
    """Inject a repeating pattern of keys for 10 seconds."""
    patterns = [
        ["2", "w", "w"],
        ["o", "o"],
        ["3", "s"],
        ["c"],
        ["1", "w"],
        ["o"],
        ["+"],
        ["4", "w", "w"],
        ["c", "c"],
        ["-"],
    ]
    idx = 0
    while not stop_event.is_set():
        for key in patterns[idx % len(patterns)]:
            if stop_event.is_set():
                return
            teleop.event_queue.put(key)
            time.sleep(0.03)
        idx += 1
        time.sleep(0.05)


def run_test():
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s  %(name)s  %(message)s",
    )
    log = logging.getLogger("test_10s")

    mock_rpc, gripper_state, gripper_cmds = _make_mock_rpc()

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

        robot_cfg = FairinoFollowerConfig(
            ip_address="192.168.58.2",
            gripper_enabled=True,
            gripper_company=4,
            gripper_device=0,
        )
        robot = FairinoFollower(robot_cfg)

        robot._probe_tcp = staticmethod(
            lambda ip, port, timeout_s=2.0: True
        )
        robot._wait_for_state_data = (
            lambda timeout_s=3.0: None
        )

        def _mock_create_proxies():
            mock_servo = MagicMock()
            mock_servo.MoveGripper = (
                mock_rpc.robot.MoveGripper
            )
            mock_servo.ServoMoveEnd = MagicMock(
                return_value=0,
            )
            mock_servo.ServoMoveStart = MagicMock(
                return_value=0,
            )
            robot._servo_proxy = mock_servo

        robot._create_servo_proxy = _mock_create_proxies
        robot._rpc = mock_rpc

        robot.connect()
        log.info("Robot connected with gripper.")

        teleop_cfg = KeyboardFairinoTeleopConfig()
        teleop = KeyboardFairinoTeleop(teleop_cfg)
        teleop._reader = SimpleNamespace(
            is_alive=lambda: True,
            stop=lambda: None,
            join=lambda timeout=None: None,
        )

        teleop_proc, robot_proc, obs_proc = (
            make_default_processors()
        )

        stop_event = threading.Event()
        injector = threading.Thread(
            target=inject_keys,
            args=(teleop, stop_event),
            daemon=True,
        )
        injector.start()

        duration_s = 10.0
        start = time.perf_counter()
        iterations = 0
        errors = 0

        log.info("Running teleop loop for %.0fs ...", duration_s)

        while time.perf_counter() - start < duration_s:
            try:
                obs = robot.get_observation()
                teleop.send_feedback(obs)
                raw_action = teleop.get_action()
                action = teleop_proc((raw_action, obs))
                to_send = robot_proc((action, obs))
                robot.send_action(to_send)
                iterations += 1
            except Exception as exc:
                errors += 1
                log.error("Loop error: %s", exc)
                if errors > 5:
                    raise
            time.sleep(1.0 / 60)

        stop_event.set()
        injector.join(timeout=2.0)

        elapsed = time.perf_counter() - start
        hz = iterations / elapsed if elapsed > 0 else 0

        log.info(
            "Completed: %d iterations in %.1fs (%.0f Hz), "
            "%d errors, %d gripper commands",
            iterations, elapsed, hz, errors,
            len(gripper_cmds),
        )

        assert errors == 0, f"{errors} errors during run"
        assert iterations > 100, (
            f"Only {iterations} iterations in {duration_s}s"
        )
        assert len(gripper_cmds) > 0, (
            "No gripper commands sent"
        )

        log.info("--- 10-SECOND TEST PASSED ---")

        robot.disconnect()


if __name__ == "__main__":
    try:
        run_test()
    except Exception as exc:
        logging.error("TEST FAILED: %s", exc, exc_info=True)
        sys.exit(1)
