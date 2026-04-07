#!/usr/bin/env python
"""Simulated gripper teleoperation test for the Fairino FR5.

Mocks the Fairino RPC connection, injects keyboard events to
open/close the gripper, and verifies gripper position changes
propagate through the full teleop pipeline.

Usage:
    python claude_test/test_fr5_gripper.py
"""

import logging
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

# ------------------------------------------------------------------
# Simulated hardware state
# ------------------------------------------------------------------

_SIM_JOINTS = [0.0, -90.0, 0.0, 0.0, 0.0, 0.0]
_SIM_GRIPPER_POS = 0.0  # starts closed


def _make_mock_rpc():
    """Build a mock RPC that tracks gripper commands."""
    rpc = MagicMock()
    rpc.robot = MagicMock()

    joint_state = list(_SIM_JOINTS)
    gripper_state = {"pos": _SIM_GRIPPER_POS}
    gripper_log = []

    # -- joint reads --
    def get_joints(flag=1):
        return (0, list(joint_state))

    rpc.GetActualJointPosDegree = get_joints
    rpc.robot.GetActualJointPosDegree = (
        lambda flag=1: [0] + list(joint_state)
    )

    # -- gripper reads (SDK path via state packet) --
    def get_gripper_pos():
        return (0, 0, gripper_state["pos"])

    rpc.GetGripperCurPosition = get_gripper_pos

    # -- gripper commands --
    def move_gripper(
        index, pos, vel, force,
        maxtime, block, gtype,
        rot_num, rot_vel, rot_torque,
    ):
        gripper_state["pos"] = float(pos)
        gripper_log.append({
            "index": index,
            "pos": pos,
            "vel": vel,
            "force": force,
        })
        return 0

    rpc.MoveGripper = move_gripper
    rpc.robot.MoveGripper = move_gripper

    # -- gripper init --
    rpc.SetGripperConfig = MagicMock(return_value=0)
    rpc.ActGripper = MagicMock(return_value=0)

    # -- other standard mocks --
    rpc.RobotEnable = MagicMock(return_value=0)
    rpc.ResetAllError = MagicMock(return_value=0)
    rpc.Mode = MagicMock(return_value=0)
    rpc.ServoMoveStart = MagicMock(return_value=0)
    rpc.ServoMoveEnd = MagicMock(return_value=0)
    rpc.StopMotion = MagicMock(return_value=0)
    rpc.CloseRPC = MagicMock()
    rpc.robot.ServoJ = MagicMock(return_value=0)
    rpc.GetActualTCPPose = lambda: (
        0, [0.0] * 6
    )

    # Fake state packet for isinstance check.
    import ctypes

    class FakeState(ctypes.Structure):
        _fields_ = [("jt_cur_pos", ctypes.c_double * 6)]

    state = FakeState()
    for i in range(6):
        state.jt_cur_pos[i] = joint_state[i]
    rpc.robot_state_pkg = state

    return rpc, gripper_state, gripper_log


def inject_keys(teleop, keys, delay_s=0.05):
    """Push key events into the teleoperator's queue."""
    for key in keys:
        time.sleep(delay_s)
        teleop.event_queue.put(key)


# ------------------------------------------------------------------
# Test
# ------------------------------------------------------------------

def run_test():
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s  %(name)s  %(message)s",
    )
    log = logging.getLogger("test_fr5_gripper")

    mock_rpc, gripper_state, gripper_log = _make_mock_rpc()

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

        # ---- robot with gripper enabled ----
        robot_cfg = FairinoFollowerConfig(
            ip_address="192.168.58.2",
            gripper_enabled=True,
            gripper_company=4,
            gripper_device=0,
            gripper_index=1,
            gripper_vel=50,
            gripper_force=50,
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
        log.info(
            "Robot connected. Joints: %s  Gripper: %.0f%%",
            robot.get_joint_positions_deg(),
            gripper_state["pos"],
        )

        # Verify gripper init was called.
        mock_rpc.SetGripperConfig.assert_called_once()
        assert mock_rpc.ActGripper.call_count >= 2, (
            "Expected reset + activate calls"
        )
        log.info("Gripper init OK (SetGripperConfig + ActGripper).")

        # Verify features include gripper.
        assert "gripper.pos" in robot.observation_features
        assert "gripper.pos" in robot.action_features
        log.info("Gripper in obs/action features OK.")

        # ---- keyboard teleop ----
        teleop_cfg = KeyboardFairinoTeleopConfig(
            gripper_step_pct=10.0,
        )
        teleop = KeyboardFairinoTeleop(teleop_cfg)
        teleop._reader = SimpleNamespace(
            is_alive=lambda: True,
            stop=lambda: None,
            join=lambda timeout=None: None,
        )

        teleop_proc, robot_proc, obs_proc = (
            make_default_processors()
        )

        # ---- key sequence ----
        # O x 5 = open to 50%, then C x 2 = close to 30%
        key_sequence = (
            ["o"] * 5 + ["c"] * 2
        )

        injector = threading.Thread(
            target=inject_keys,
            args=(teleop, key_sequence, 0.06),
        )
        injector.start()

        # ---- teleop loop ----
        num_iters = 20
        actions_log = []
        log.info(
            "Running %d iterations (gripper_step=%.0f%%) ...",
            num_iters, teleop_cfg.gripper_step_pct,
        )

        for _ in range(num_iters):
            obs = robot.get_observation()
            teleop.send_feedback(obs)
            raw_action = teleop.get_action()
            action = teleop_proc((raw_action, obs))
            to_send = robot_proc((action, obs))
            robot.send_action(to_send)
            actions_log.append(dict(to_send))
            time.sleep(0.04)

        injector.join(timeout=3.0)

        # ---- verify gripper movement ----
        final = actions_log[-1]
        log.info("Final action: %s", final)
        log.info(
            "Gripper commands sent: %d", len(gripper_log),
        )
        for i, cmd in enumerate(gripper_log):
            log.info(
                "  cmd[%d]: pos=%d vel=%d force=%d",
                i, cmd["pos"], cmd["vel"], cmd["force"],
            )

        # Final gripper target should be ~30%
        # (5 opens x 10% = 50%, 2 closes x 10% = -20%)
        expected_grip = 30.0
        actual_grip = final["gripper.pos"]
        log.info(
            "Gripper expected ~%.0f%%, got %.0f%%",
            expected_grip, actual_grip,
        )
        assert abs(actual_grip - expected_grip) < 1.0, (
            f"Gripper mismatch: {actual_grip} != "
            f"{expected_grip}"
        )

        # Gripper position in obs should reflect commands.
        last_obs = robot.get_observation()
        log.info(
            "Observation gripper.pos = %.0f%%",
            last_obs["gripper.pos"],
        )

        # Verify MoveGripper was called with correct params.
        assert len(gripper_log) > 0, (
            "No MoveGripper commands were sent"
        )
        assert gripper_log[-1]["vel"] == 50
        assert gripper_log[-1]["force"] == 50

        # ---- verify joints unchanged ----
        for jname in robot_cfg.joint_names:
            key = f"{jname}.pos"
            idx = robot_cfg.joint_names.index(jname)
            expected = _SIM_JOINTS[idx]
            actual = final[key]
            assert abs(actual - expected) < 0.01, (
                f"{jname} changed: {actual} != {expected}"
            )
        log.info("Joints unchanged OK.")

        log.info("--- ALL ASSERTIONS PASSED ---")

        robot.disconnect()
        log.info("Robot disconnected.")


if __name__ == "__main__":
    try:
        run_test()
    except Exception as exc:
        logging.error("TEST FAILED: %s", exc, exc_info=True)
        sys.exit(1)
