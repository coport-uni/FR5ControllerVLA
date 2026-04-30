#!/usr/bin/env python

# Copyright APPEAL Automation team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.

"""LeRobot-compatible wrapper for the Fairino FR5 robot.

Communicates with the Fairino controller via two channels:
    XMLRPC (port 20003) -- command interface
    TCP    (port 20004) -- real-time state feedback

Observation and action dicts use the flat-key convention
expected by the LeRobot framework:
    observation keys : "joint1.pos" .. "joint6.pos", camera images
    action keys      : "joint1.pos" .. "joint6.pos"

All joint values are in **degrees** (Fairino's native unit).

ServoJ commands are issued synchronously from the calling
thread (typically the teleoperation main loop) because the
Fairino V3.9.1 firmware rejects ServoJ calls made from
non-main threads.
"""

import contextlib
import logging
import threading
import time
import xmlrpc.client
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation

from .config_fairino_follower import FairinoFollowerConfig
from .fairino.Robot import RPC as FairinoRPC  # noqa: N811

logger = logging.getLogger(__name__)


# Timing constants for the hardware initialisation sequence.
_SETTLE_SHORT_S = 0.2
_SETTLE_MID_S = 0.3
_SETTLE_LONG_S = 0.5

# Joint-read retry parameters for transient failures.
_READ_RETRIES = 3
_READ_RETRY_DELAY_S = 0.05

# Error code: servo session terminated by controller.
_SERVO_SESSION_LOST = 14


class FairinoFollower(Robot):
    """Control a Fairino FR5 6-DOF arm through the LeRobot API.

    Uses ServoJ (real-time servo) for joint-position commands,
    which provides the low latency required for VLA inference
    and keyboard teleoperation.

    ServoJ is called **synchronously** in ``send_action()``
    from the caller's thread.  The Fairino V3.9.1 firmware
    rejects ServoJ issued from background threads.

    Attributes:
        config: Frozen dataclass with IP, joint limits, etc.
        cameras: Connected camera instances (name -> Camera).
    """

    config_class = FairinoFollowerConfig
    name = "fairino_follower"

    def __init__(self, config: FairinoFollowerConfig):
        super().__init__(config)
        self.config = config
        self._rpc = None
        self._is_connected = False
        self._use_xmlrpc_reads = False
        self.cameras = {}

        # XMLRPC proxy used exclusively for ServoJ.
        self._servo_proxy = None

        # Separate XMLRPC proxy for the gripper worker thread.
        # xmlrpc.client.ServerProxy shares a single Transport,
        # so we keep main-thread (ServoJ) and worker-thread
        # (MoveGripper / ServoMoveEnd / ServoMoveStart) on
        # distinct proxies to avoid transport races.
        self._gripper_proxy = None

        # Interpolation state for velocity-limited ramp.
        self._commanded: list[float] = [0.0] * 6

        # Gripper state.
        self._gripper_pos: float = 0.0
        self._last_gripper_cmd: float = -1.0

        # Gripper worker thread -- offloads the blocking
        # ServoMoveEnd / MoveGripper / ServoMoveStart sequence
        # off the control-loop thread so send_action() keeps
        # running at the configured control_hz even while the
        # gripper is actively moving.
        self._gripper_worker: threading.Thread | None = None
        self._gripper_stop_event = threading.Event()
        self._gripper_wake_event = threading.Event()
        self._gripper_lock = threading.Lock()
        # Latest-wins slot -- older targets are overwritten
        # before the worker consumes them.
        self._gripper_target_pending: float | None = None

    # ---- properties (required by Robot base class) ---------

    @property
    def observation_features(self) -> dict:
        """Return observation schema (works when disconnected)."""
        features = {}
        for jname in self.config.joint_names:
            features[f"{jname}.pos"] = float
        if self.config.gripper_enabled:
            features["gripper.pos"] = float
        for cam_name, cam_cfg in self.config.cameras.items():
            features[cam_name] = (
                cam_cfg.height,
                cam_cfg.width,
                3,
            )
        return features

    @property
    def action_features(self) -> dict:
        """Return action schema (works when disconnected)."""
        features = {f"{jname}.pos": float for jname in self.config.joint_names}
        if self.config.gripper_enabled:
            features["gripper.pos"] = float
        return features

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        """Fairino uses absolute encoders; always calibrated."""
        return True

    # ---- lifecycle ------------------------------------------

    def connect(self, calibrate: bool = True) -> None:
        """Open communication and initialise servo mode.

        Performs a full reset sequence to handle stale sessions:
            ServoMoveEnd -> RobotEnable(0) -> ResetAllError
            -> RobotEnable(1) -> Mode(0) -> ServoMoveStart

        Args:
            calibrate: Ignored (absolute encoders).

        Raises:
            RuntimeError: If already connected.
            ConnectionError: If the controller is unreachable.
        """
        if self._is_connected:
            raise RuntimeError("[Fairino] Already connected.")

        logger.info(
            "[Fairino] Connecting to %s ...",
            self.config.ip_address,
        )

        tcp_ok = self._probe_tcp(
            self.config.ip_address,
            20004,
            timeout_s=2.0,
        )

        if tcp_ok:
            self._rpc = FairinoRPC(self.config.ip_address)
            try:
                self._wait_for_state_data(timeout_s=3.0)
            except ConnectionError:
                tcp_ok = False
        if not tcp_ok:
            logger.warning("[Fairino] TCP 20004 unavailable; using XMLRPC-only mode.")
            self._init_xmlrpc_only()

        self._use_xmlrpc_reads = not tcp_ok

        ret, joints = self._read_joints()
        if ret != 0:
            self._rpc = None
            raise ConnectionError(f"[Fairino] Connection failed (err {ret})")

        self._initialise_servo_mode()
        self._commanded = list(joints)

        # Create the servo proxy used by send_action(), plus
        # a second proxy dedicated to the gripper worker.
        link = f"http://{self.config.ip_address}:20003"
        self._servo_proxy = xmlrpc.client.ServerProxy(link)
        self._gripper_proxy = xmlrpc.client.ServerProxy(link)

        self._is_connected = True
        logger.info(
            "[Fairino] Connected. Joints (deg): %s",
            joints,
        )

        if self.config.cameras:
            self.cameras = make_cameras_from_configs(self.config.cameras)
            for cam_name, cam in self.cameras.items():
                cam.connect()
                logger.info(
                    "[Fairino] Camera '%s' connected.",
                    cam_name,
                )

        if self.config.gripper_enabled:
            self._initialise_gripper()
            self._start_gripper_worker()

    def disconnect(self) -> None:
        """End servo mode and release RPC resources."""
        # Stop the gripper worker first so it cannot race the
        # RPC teardown below.
        self._stop_gripper_worker()

        for cam_name, cam in self.cameras.items():
            cam.disconnect()
            logger.info(
                "[Fairino] Camera '%s' disconnected.",
                cam_name,
            )

        if self._rpc is not None:
            if self.config.gripper_enabled:
                with contextlib.suppress(Exception):
                    self._rpc.ActGripper(
                        self.config.gripper_index,
                        0,
                    )
            with contextlib.suppress(Exception):
                self._rpc.ServoMoveEnd()
            with contextlib.suppress(Exception):
                self._rpc.CloseRPC()
            self._rpc = None

        self._servo_proxy = None
        self._gripper_proxy = None
        self._is_connected = False
        logger.info("[Fairino] Disconnected.")

    def calibrate(self) -> None:
        """No-op: Fairino uses absolute encoders."""

    def configure(self) -> None:
        """No-op: no runtime configuration needed."""

    # ---- observation / action -------------------------------

    def get_observation(self) -> RobotObservation:
        """Read joint positions [deg] and camera images.

        Returns:
            Flat dict matching ``observation_features`` keys.

        Raises:
            RuntimeError: If the joint query fails.
        """
        self._assert_connected()

        obs: dict[str, Any] = {}

        ret, joints_deg = self._read_joints()
        if ret != 0:
            raise RuntimeError(f"[Fairino] Joint read failed (err {ret})")

        for i, jname in enumerate(self.config.joint_names):
            obs[f"{jname}.pos"] = joints_deg[i]

        if self.config.gripper_enabled:
            obs["gripper.pos"] = self._read_gripper_pos()

        for cam_name, cam in self.cameras.items():
            obs[cam_name] = cam.async_read()

        return obs

    def send_action(self, action: RobotAction) -> RobotAction:
        """Send a ServoJ command toward the target [deg].

        Interpolates one step toward the target at
        ``max_servo_speed`` deg/s, then issues a single
        ServoJ call.  Must be called at the loop frequency
        (``control_hz``) for smooth motion.

        Args:
            action: Dict with "joint{1..6}.pos" in degrees.

        Returns:
            Dict of the clamped values that will be sent.
        """
        self._assert_connected()

        target_deg = [float(action[f"{jname}.pos"]) for jname in self.config.joint_names]

        lower = self.config.joint_limits_lower
        upper = self.config.joint_limits_upper
        clamped_deg = [
            max(lo, min(hi, val))
            for val, lo, hi in zip(
                target_deg,
                lower,
                upper,
                strict=True,
            )
        ]

        # Ramp one step toward the target.  ``ramp_in_progress``
        # is set whenever any joint took only a partial step;
        # while it is True, gripper commands are dropped so the
        # worker's ServoMoveEnd / MoveGripper / ServoMoveStart
        # sequence cannot race the in-flight ServoJ ramp and
        # tear down the servo session (error 14).
        period = 1.0 / self.config.servo_hz
        max_step = self.config.max_servo_speed * period
        ramp_in_progress = False
        for i in range(len(self._commanded)):
            delta = clamped_deg[i] - self._commanded[i]
            if abs(delta) > max_step:
                self._commanded[i] += max_step if delta > 0 else -max_step
                ramp_in_progress = True
            else:
                self._commanded[i] = clamped_deg[i]

        # Send ServoJ synchronously.
        axis_pos = [0.0, 0.0, 0.0, 0.0]
        try:
            ret = self._servo_proxy.ServoJ(
                self._commanded,
                axis_pos,
                0.0,
                0.0,
                period,
                0.0,
                0.0,
            )
            if ret == _SERVO_SESSION_LOST:
                self._recover_servo_session()
            elif ret != 0:
                logger.debug(
                    "[Fairino] ServoJ err: %d",
                    ret,
                )
        except Exception:
            logger.debug(
                "[Fairino] ServoJ call failed.",
                exc_info=True,
            )

        result = {f"{jname}.pos": clamped_deg[i] for i, jname in enumerate(self.config.joint_names)}

        if self.config.gripper_enabled and "gripper.pos" in action and not ramp_in_progress:
            grip = max(0.0, min(100.0, float(action["gripper.pos"])))
            result["gripper.pos"] = grip
            self._enqueue_gripper_cmd(grip)

        return result

    # ---- convenience helpers --------------------------------

    def get_joint_positions_deg(self) -> list[float]:
        """Return current joint angles as a plain list [deg]."""
        self._assert_connected()
        ret, joints = self._read_joints()
        if ret != 0:
            raise RuntimeError(f"[Fairino] Joint read failed (err {ret})")
        return list(joints)

    def get_tcp_pose(self) -> list[float]:
        """Return TCP pose [x, y, z, rx, ry, rz] in mm/deg."""
        self._assert_connected()
        ret, pose = self._rpc.GetActualTCPPose()
        if ret != 0:
            raise RuntimeError(f"[Fairino] TCP read failed (err {ret})")
        return list(pose)

    def stop_motion(self) -> None:
        """Issue an immediate motion stop command."""
        if self._rpc is not None:
            self._rpc.StopMotion()

    def reset_errors(self) -> None:
        """Clear errors, re-enable, and restart servo mode."""
        if self._rpc is not None:
            with contextlib.suppress(Exception):
                self._rpc.ServoMoveEnd()
            self._initialise_servo_mode()
            logger.info("[Fairino] Errors reset; servo restarted.")

    # ---- private helpers ------------------------------------

    def _assert_connected(self) -> None:
        if not self._is_connected or self._rpc is None:
            raise RuntimeError("[Fairino] Not connected. Call connect().")

    def _recover_servo_session(self) -> None:
        """Re-establish the servo session after error 14."""
        logger.info("[Fairino] Servo session lost; recovering ...")
        try:
            self._servo_proxy.ServoMoveEnd()
            time.sleep(_SETTLE_SHORT_S)
            self._servo_proxy.ServoMoveStart()
            time.sleep(_SETTLE_SHORT_S)
            logger.info("[Fairino] Servo session recovered.")
        except Exception as exc:
            logger.warning(
                "[Fairino] Servo recovery failed: %s",
                exc,
            )

    def _wait_for_state_data(
        self,
        timeout_s: float = 5.0,
    ) -> None:
        """Block until the TCP thread populates state."""
        from ctypes import Structure

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            pkg = self._rpc.robot_state_pkg
            if isinstance(pkg, Structure):
                return
            time.sleep(0.1)
        raise ConnectionError(
            f"[Fairino] Timed out waiting for state data ({timeout_s}s). Check TCP port 20004."
        )

    @staticmethod
    def _probe_tcp(
        ip: str,
        port: int,
        timeout_s: float = 2.0,
    ) -> bool:
        """Return True if a TCP connection can be opened."""
        import socket as _socket

        sock = _socket.socket(
            _socket.AF_INET,
            _socket.SOCK_STREAM,
        )
        sock.settimeout(timeout_s)
        try:
            sock.connect((ip, port))
            sock.close()
            return True
        except Exception:
            return False

    def _init_xmlrpc_only(self) -> None:
        """Create a minimal RPC-like namespace for XMLRPC-only."""
        import types

        link = f"http://{self.config.ip_address}:20003"
        proxy = xmlrpc.client.ServerProxy(link)

        rpc = types.SimpleNamespace()
        rpc.robot = proxy
        rpc.RobotEnable = proxy.RobotEnable
        rpc.ResetAllError = proxy.ResetAllError
        rpc.Mode = proxy.Mode
        rpc.ServoMoveStart = proxy.ServoMoveStart
        rpc.ServoMoveEnd = proxy.ServoMoveEnd
        rpc.StopMotion = proxy.StopMotion
        rpc.GetActualTCPPose = proxy.GetActualTCPPose
        rpc.SetGripperConfig = proxy.SetGripperConfig
        rpc.ActGripper = proxy.ActGripper
        rpc.MoveGripper = proxy.MoveGripper
        rpc.CloseRPC = lambda: None
        self._rpc = rpc

    def _read_joints(self) -> tuple[int, list[float]]:
        """Read joint positions with retries.

        Returns:
            (error_code, [j1..j6]) in degrees.
        """
        for attempt in range(_READ_RETRIES):
            ret, joints = self._read_joints_once()
            if ret == 0:
                return 0, joints
            if attempt < _READ_RETRIES - 1:
                logger.debug(
                    "[Fairino] Joint read retry %d/%d (err %d)",
                    attempt + 1,
                    _READ_RETRIES,
                    ret,
                )
                time.sleep(_READ_RETRY_DELAY_S)
        return ret, joints

    def _read_joints_once(
        self,
    ) -> tuple[int, list[float]]:
        """Single attempt to read joint positions."""
        if not self._use_xmlrpc_reads:
            try:
                result = self._rpc.GetActualJointPosDegree()
            except Exception as exc:
                logger.warning(
                    "[Fairino] SDK joint read error: %s",
                    exc,
                )
                return -1, [0.0] * 6

            if isinstance(result, tuple):
                return result[0], list(result[1])
            return int(result), [0.0] * 6

        try:
            result = self._rpc.robot.GetActualJointPosDegree(1)
            if result[0] == 0:
                return 0, list(result[1:7])
            return result[0], [0.0] * 6
        except Exception as exc:
            logger.warning(
                "[Fairino] XMLRPC joint read error: %s",
                exc,
            )
            return -1, [0.0] * 6

    # ---- gripper helpers ------------------------------------

    def _initialise_gripper(self) -> None:
        """Configure and activate the gripper."""
        cfg = self.config
        ret = self._rpc.SetGripperConfig(
            cfg.gripper_company,
            cfg.gripper_device,
            0,
            0,
        )
        if ret != 0:
            logger.warning(
                "[Fairino] SetGripperConfig err: %d",
                ret,
            )
        time.sleep(_SETTLE_MID_S)

        self._rpc.ActGripper(cfg.gripper_index, 0)
        time.sleep(_SETTLE_LONG_S)
        ret = self._rpc.ActGripper(cfg.gripper_index, 1)
        if ret != 0:
            logger.warning(
                "[Fairino] ActGripper err: %d",
                ret,
            )
        time.sleep(1.0)

        self._gripper_pos = self._read_gripper_pos()
        self._last_gripper_cmd = self._gripper_pos
        logger.info(
            "[Fairino] Gripper activated (pos=%.0f%%).",
            self._gripper_pos,
        )

    def _read_gripper_pos(self) -> float:
        """Read the current gripper position [0-100 %]."""
        if not self._use_xmlrpc_reads:
            try:
                result = self._rpc.GetGripperCurPosition()
                if isinstance(result, tuple) and len(result) >= 3 and result[0] == 0:
                    self._gripper_pos = float(result[2])
                    return self._gripper_pos
            except Exception as exc:
                logger.debug(
                    "[Fairino] Gripper read err: %s",
                    exc,
                )
        return self._gripper_pos

    def _enqueue_gripper_cmd(self, pos: float) -> None:
        """Queue a gripper target for the worker thread.

        Runs on the main control-loop thread.  Drops commands
        smaller than 1 % away from the last request to avoid
        waking the worker on micro-motions, then stores the
        latest target in a size-1 slot.  Previous un-consumed
        targets are overwritten (latest-wins), so the worker
        never builds a backlog.

        Args:
            pos: Target position [0-100 %].
        """
        if abs(pos - self._last_gripper_cmd) < 1.0:
            return
        self._last_gripper_cmd = pos
        self._gripper_pos = pos
        with self._gripper_lock:
            self._gripper_target_pending = pos
        self._gripper_wake_event.set()

    def _start_gripper_worker(self) -> None:
        """Spawn the background thread that drives the gripper."""
        self._gripper_stop_event.clear()
        self._gripper_wake_event.clear()
        self._gripper_target_pending = None
        self._gripper_worker = threading.Thread(
            target=self._gripper_worker_loop,
            name="fairino-gripper",
            daemon=True,
        )
        self._gripper_worker.start()

    def _stop_gripper_worker(self) -> None:
        """Signal the worker to exit and join it."""
        worker = self._gripper_worker
        if worker is None:
            return
        self._gripper_stop_event.set()
        self._gripper_wake_event.set()
        worker.join(timeout=2.0)
        self._gripper_worker = None

    def _gripper_worker_loop(self) -> None:
        """Worker loop: apply the latest queued target.

        The XMLRPC sequence (ServoMoveEnd -> MoveGripper ->
        ServoMoveStart) used to run inline in send_action(),
        which stalled the main control loop at ~1 Hz while
        the gripper was moving.  Running it here keeps the
        main loop at the configured control_hz.  ServoJ
        issued from the main thread during the pause window
        may return _SERVO_SESSION_LOST; _recover_servo_session
        already handles that case.
        """
        while not self._gripper_stop_event.is_set():
            self._gripper_wake_event.wait(timeout=0.5)
            if self._gripper_stop_event.is_set():
                break
            self._gripper_wake_event.clear()

            with self._gripper_lock:
                target = self._gripper_target_pending
                self._gripper_target_pending = None
            if target is None:
                continue

            try:
                self._run_gripper_cmd(float(target))
            except Exception as exc:
                logger.warning(
                    "[Fairino] Gripper worker err: %s",
                    exc,
                )

    def _run_gripper_cmd(self, pos: float) -> None:
        """Pause servo mode, command the gripper, resume servo.

        The Fairino controller blocks ``MoveGripper`` while
        servo mode is active, so we pause/resume around it.
        Called from the gripper worker thread; the main
        control thread keeps issuing ServoJ during the pause
        window and recovers via _recover_servo_session when
        the controller reports the session was torn down.

        Args:
            pos: Target position [0-100 %].
        """
        pos_int = int(pos)
        cfg = self.config
        proxy = self._gripper_proxy
        if proxy is None:
            return
        try:
            proxy.ServoMoveEnd()
            time.sleep(0.02)
            ret = proxy.MoveGripper(
                int(cfg.gripper_index),
                pos_int,
                int(cfg.gripper_vel),
                int(cfg.gripper_force),
                30000,
                1,
                0,
                0.0,
                0,
                0,
            )
            if ret != 0:
                logger.warning(
                    "[Fairino] MoveGripper FAILED (err %d)",
                    ret,
                )
            else:
                logger.info(
                    "[Fairino] Gripper -> %d%%",
                    pos_int,
                )
            time.sleep(0.02)
            proxy.ServoMoveStart()
            time.sleep(0.02)
        except Exception as exc:
            logger.warning(
                "[Fairino] Gripper err: %s",
                exc,
            )
            with contextlib.suppress(Exception):
                proxy.ServoMoveStart()

    # ---- servo helpers ------------------------------------

    def _initialise_servo_mode(self) -> None:
        """Run the full enable-and-servo startup sequence.

        The settle times must be generous (>=0.5 s) because
        the controller needs time to transition between
        states.  Shorter pauses cause ServoJ to return
        error 101 persistently.
        """
        with contextlib.suppress(Exception):
            self._rpc.ServoMoveEnd()
        time.sleep(_SETTLE_LONG_S)

        with contextlib.suppress(Exception):
            self._rpc.StopMotion()
        time.sleep(_SETTLE_LONG_S)

        self._rpc.RobotEnable(0)
        time.sleep(1.0)
        self._rpc.ResetAllError()
        time.sleep(1.0)
        self._rpc.RobotEnable(1)
        time.sleep(1.0)
        self._rpc.Mode(0)
        time.sleep(1.0)

        ret = self._rpc.ServoMoveStart()
        if ret != 0:
            logger.warning("[Fairino] ServoMoveStart error: %d", ret)
        time.sleep(_SETTLE_LONG_S)
