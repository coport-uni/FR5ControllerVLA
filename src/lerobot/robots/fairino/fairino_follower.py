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
# Each pause gives the controller time to process the command.
_SETTLE_SHORT_S = 0.2   # [s] after ServoMoveEnd cleanup
_SETTLE_MID_S = 0.3     # [s] between enable / reset / mode
_SETTLE_LONG_S = 0.5    # [s] after Mode(0) before ServoMoveStart

# Joint-read retry parameters for transient failures.
_READ_RETRIES = 3
_READ_RETRY_DELAY_S = 0.05



class FairinoFollower(Robot):
    """Control a Fairino FR5 6-DOF arm through the LeRobot API.

    Uses ServoJ (real-time servo) for joint-position commands,
    which provides the low latency required for VLA inference
    and keyboard teleoperation.

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

        # Internal servo thread state.
        self._servo_thread: threading.Thread | None = None
        self._servo_target: list[float] = [0.0] * 6
        self._servo_lock = threading.Lock()
        self._servo_running = False
        # Dedicated XMLRPC proxy for the servo thread.
        # ServerProxy is NOT thread-safe, so the servo loop
        # must not share a proxy with the main thread.
        self._servo_proxy = None

        # Gripper state.
        self._gripper_pos: float = 0.0
        self._last_gripper_cmd: float = -1.0
        # Pending gripper command consumed by the servo loop.
        self._pending_gripper: int | None = None

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
            key = f"observation.images.{cam_name}"
            features[key] = (
                cam_cfg.height, cam_cfg.width, 3
            )
        return features

    @property
    def action_features(self) -> dict:
        """Return action schema (works when disconnected)."""
        features = {
            f"{jname}.pos": float
            for jname in self.config.joint_names
        }
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
            raise RuntimeError(
                "[Fairino] Already connected."
            )

        logger.info(
            "[Fairino] Connecting to %s ...",
            self.config.ip_address,
        )

        # Probe TCP 20004 before creating the SDK object.
        # The SDK constructor blocks on TCP connect, so we
        # check reachability first with a short timeout.
        tcp_ok = self._probe_tcp(
            self.config.ip_address, 20004, timeout_s=2.0,
        )

        if tcp_ok:
            self._rpc = FairinoRPC(self.config.ip_address)
            try:
                self._wait_for_state_data(timeout_s=3.0)
            except ConnectionError:
                tcp_ok = False
        if not tcp_ok:
            logger.warning(
                "[Fairino] TCP 20004 unavailable; "
                "using XMLRPC-only mode."
            )
            self._init_xmlrpc_only()

        self._use_xmlrpc_reads = not tcp_ok

        # Verify communication by reading joint positions.
        ret, joints = self._read_joints()
        if ret != 0:
            self._rpc = None
            raise ConnectionError(
                f"[Fairino] Connection failed (err {ret})"
            )

        self._initialise_servo_mode()

        # Initialise the servo target to the current position
        # so the robot does not jump on the first command.
        self._servo_target = list(joints)
        self._create_servo_proxy()
        self._start_servo_thread()

        self._is_connected = True
        logger.info(
            "[Fairino] Connected. Joints (deg): %s",
            joints,
        )

        # Connect cameras if configured.
        if self.config.cameras:
            self.cameras = make_cameras_from_configs(
                self.config.cameras
            )
            for cam_name, cam in self.cameras.items():
                cam.connect()
                logger.info(
                    "[Fairino] Camera '%s' connected.",
                    cam_name,
                )

        # Activate gripper if configured.
        if self.config.gripper_enabled:
            self._initialise_gripper()

    def disconnect(self) -> None:
        """Stop servo thread, end servo mode, and release RPC."""
        self._stop_servo_thread()

        for cam_name, cam in self.cameras.items():
            cam.disconnect()
            logger.info(
                "[Fairino] Camera '%s' disconnected.",
                cam_name,
            )

        if self._rpc is not None:
            # Deactivate gripper before closing.
            if self.config.gripper_enabled:
                with contextlib.suppress(Exception):
                    self._rpc.ActGripper(
                        self.config.gripper_index, 0,
                    )
            with contextlib.suppress(Exception):
                self._rpc.ServoMoveEnd()
            with contextlib.suppress(Exception):
                self._rpc.CloseRPC()
            self._rpc = None

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
            raise RuntimeError(
                f"[Fairino] Joint read failed (err {ret})"
            )

        for i, jname in enumerate(self.config.joint_names):
            obs[f"{jname}.pos"] = joints_deg[i]

        if self.config.gripper_enabled:
            obs["gripper.pos"] = self._read_gripper_pos()

        for cam_name, cam in self.cameras.items():
            key = f"observation.images.{cam_name}"
            obs[key] = cam.async_read()

        return obs

    def send_action(self, action: RobotAction) -> RobotAction:
        """Update the servo target [deg].

        The internal servo thread continuously sends the
        latest target to the controller at ``servo_hz``.
        This method only updates the shared target variable.

        Args:
            action: Dict with "joint{1..6}.pos" in degrees.

        Returns:
            Dict of the clamped values that will be sent.
        """
        self._assert_connected()

        # Collect targets in joint order.
        target_deg = [
            float(action[f"{jname}.pos"])
            for jname in self.config.joint_names
        ]

        # Clamp to safe joint limits.
        lower = self.config.joint_limits_lower
        upper = self.config.joint_limits_upper
        clamped_deg = [
            max(lo, min(hi, val))
            for val, lo, hi in zip(
                target_deg, lower, upper, strict=True,
            )
        ]

        with self._servo_lock:
            self._servo_target = list(clamped_deg)

        result = {
            f"{jname}.pos": clamped_deg[i]
            for i, jname in enumerate(self.config.joint_names)
        }

        # Send gripper command if present and changed.
        if (
            self.config.gripper_enabled
            and "gripper.pos" in action
        ):
            grip = max(0.0, min(100.0, float(
                action["gripper.pos"]
            )))
            result["gripper.pos"] = grip
            self._send_gripper_cmd(grip)

        return result

    # ---- convenience helpers --------------------------------

    def get_joint_positions_deg(self) -> list[float]:
        """Return current joint angles as a plain list [deg]."""
        self._assert_connected()
        ret, joints = self._read_joints()
        if ret != 0:
            raise RuntimeError(
                f"[Fairino] Joint read failed (err {ret})"
            )
        return list(joints)

    def get_tcp_pose(self) -> list[float]:
        """Return TCP pose [x, y, z, rx, ry, rz] in mm/deg."""
        self._assert_connected()
        ret, pose = self._rpc.GetActualTCPPose()
        if ret != 0:
            raise RuntimeError(
                f"[Fairino] TCP read failed (err {ret})"
            )
        return list(pose)

    def stop_motion(self) -> None:
        """Issue an immediate motion stop command."""
        if self._rpc is not None:
            self._rpc.StopMotion()

    def reset_errors(self) -> None:
        """Clear errors, re-enable, and restart servo mode."""
        if self._rpc is not None:
            self._stop_servo_thread()
            with contextlib.suppress(Exception):
                self._rpc.ServoMoveEnd()
            self._initialise_servo_mode()
            self._start_servo_thread()
            logger.info(
                "[Fairino] Errors reset; servo restarted."
            )

    # ---- private helpers ------------------------------------

    def _assert_connected(self) -> None:
        """Raise if the robot is not connected."""
        if not self._is_connected or self._rpc is None:
            raise RuntimeError(
                "[Fairino] Not connected. Call connect()."
            )

    def _wait_for_state_data(
        self, timeout_s: float = 5.0,
    ) -> None:
        """Block until the TCP thread populates state.

        The SDK sets ``robot_state_pkg`` to the *class*
        (not an instance) on init.  The background thread
        replaces it with ``from_buffer_copy()`` once the
        first TCP packet arrives.  We poll until that
        happens so that subsequent reads are safe.

        Args:
            timeout_s: Maximum wait time [s].

        Raises:
            ConnectionError: If state data never arrives.
        """
        from ctypes import Structure

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            pkg = self._rpc.robot_state_pkg
            if isinstance(pkg, Structure):
                return
            time.sleep(0.1)
        raise ConnectionError(
            "[Fairino] Timed out waiting for state data "
            f"({timeout_s}s). Check TCP port 20004."
        )

    @staticmethod
    def _probe_tcp(
        ip: str, port: int, timeout_s: float = 2.0,
    ) -> bool:
        """Return True if a TCP connection can be opened."""
        import socket as _socket
        sock = _socket.socket(
            _socket.AF_INET, _socket.SOCK_STREAM,
        )
        sock.settimeout(timeout_s)
        try:
            sock.connect((ip, port))
            sock.close()
            return True
        except Exception:
            return False

    def _init_xmlrpc_only(self) -> None:
        """Create a minimal RPC-like object for XMLRPC-only.

        When TCP 20004 is occupied we cannot use the full SDK
        (its constructor blocks).  Instead we create a thin
        namespace that exposes ``.robot`` (the ServerProxy)
        and the helper methods we need.
        """
        import types
        import xmlrpc.client

        link = (
            f"http://{self.config.ip_address}:20003"
        )
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
        """Read joint positions, using the best available method.

        When TCP 20004 is connected, the SDK's
        ``GetActualJointPosDegree`` reads from the fast
        state-packet cache.  When TCP is unavailable we
        fall back to a direct XMLRPC call.

        Retries up to ``_READ_RETRIES`` times on transient
        failures before returning an error code.

        Returns:
            (error_code, [j1..j6]) in degrees.
        """
        for attempt in range(_READ_RETRIES):
            ret, joints = self._read_joints_once()
            if ret == 0:
                return 0, joints
            if attempt < _READ_RETRIES - 1:
                logger.debug(
                    "[Fairino] Joint read retry %d/%d "
                    "(err %d)",
                    attempt + 1, _READ_RETRIES, ret,
                )
                time.sleep(_READ_RETRY_DELAY_S)
        return ret, joints

    def _read_joints_once(
        self,
    ) -> tuple[int, list[float]]:
        """Single attempt to read joint positions.

        Returns:
            (error_code, [j1..j6]) in degrees.
        """
        if not self._use_xmlrpc_reads:
            try:
                result = self._rpc.GetActualJointPosDegree()
            except Exception as exc:
                logger.warning(
                    "[Fairino] SDK joint read error: %s",
                    exc,
                )
                return -1, [0.0] * 6

            # The SDK normally returns (0, [j1..j6]).
            # The @xmlrpc_timeout decorator returns a bare
            # int (-4) when RPC.is_conect is False.
            if isinstance(result, tuple):
                return result[0], list(result[1])
            return int(result), [0.0] * 6

        # XMLRPC fallback.
        try:
            result = (
                self._rpc.robot.GetActualJointPosDegree(1)
            )
            if result[0] == 0:
                return 0, list(result[1:7])
            return result[0], [0.0] * 6
        except Exception as exc:
            logger.warning(
                "[Fairino] XMLRPC joint read error: %s",
                exc,
            )
            return -1, [0.0] * 6

    def _create_servo_proxy(self) -> None:
        """Create a dedicated XMLRPC proxy for the servo thread.

        ``xmlrpc.client.ServerProxy`` is **not** thread-safe.
        The servo loop runs at 100 Hz in a background thread,
        so it needs its own proxy to avoid ``CannotSendRequest``
        errors when the main thread issues concurrent calls.

        The servo thread also handles gripper commands: it
        pauses ServoJ, sends MoveGripper, then resumes,
        because the Fairino controller blocks MoveGripper
        while servo mode is active.
        """
        link = (
            f"http://{self.config.ip_address}:20003"
        )
        self._servo_proxy = xmlrpc.client.ServerProxy(link)

    def _start_servo_thread(self) -> None:
        """Launch the background thread that streams ServoJ."""
        self._servo_running = True
        self._servo_thread = threading.Thread(
            target=self._servo_loop, daemon=True,
        )
        self._servo_thread.start()
        logger.info(
            "[Fairino] Servo thread started at %d Hz.",
            int(self.config.servo_hz),
        )

    def _stop_servo_thread(self) -> None:
        """Signal the servo thread to stop and wait for it."""
        self._servo_running = False
        if self._servo_thread is not None:
            self._servo_thread.join(timeout=2.0)
            self._servo_thread = None
        self._servo_proxy = None

    def _servo_loop(self) -> None:
        """Send ServoJ commands at ``servo_hz`` in a tight loop.

        Also handles gripper commands: the Fairino controller
        blocks ``MoveGripper`` while servo mode is active, so
        the loop temporarily pauses servo (``ServoMoveEnd``),
        sends the gripper command, then resumes
        (``ServoMoveStart``).  This keeps everything on a
        single XMLRPC proxy (``_servo_proxy``), avoiding
        thread-safety issues entirely.

        To avoid ServoJ rejecting large position jumps, the
        loop interpolates toward the target at a maximum
        velocity of ``config.max_servo_speed`` deg/s.
        """
        period = 1.0 / self.config.servo_hz
        cmd_t = period
        axis_pos = [0.0, 0.0, 0.0, 0.0]
        max_step = (
            self.config.max_servo_speed * period
        )
        proxy = self._servo_proxy

        # Start from the current servo target (set to the
        # actual joint position during connect()).
        with self._servo_lock:
            commanded = list(self._servo_target)

        while self._servo_running:
            t0 = time.perf_counter()

            # ---- handle pending gripper command --------
            grip_cmd = self._pending_gripper
            if grip_cmd is not None:
                self._pending_gripper = None
                self._exec_gripper_in_servo(
                    proxy, grip_cmd,
                )

            # ---- normal ServoJ -------------------------
            with self._servo_lock:
                target = list(self._servo_target)

            # Ramp each joint toward the target.
            for i in range(len(commanded)):
                delta = target[i] - commanded[i]
                if abs(delta) > max_step:
                    commanded[i] += (
                        max_step if delta > 0 else -max_step
                    )
                else:
                    commanded[i] = target[i]

            try:
                ret = proxy.ServoJ(
                    commanded, axis_pos,
                    0.0, 0.0, cmd_t, 0.0, 0.0,
                )
                if ret != 0:
                    logger.debug(
                        "[Fairino] ServoJ err: %d", ret,
                    )
            except Exception:
                if not self._servo_running:
                    break
                logger.debug(
                    "[Fairino] ServoJ call failed.",
                    exc_info=True,
                )

            elapsed = time.perf_counter() - t0
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def _exec_gripper_in_servo(
        self,
        proxy: xmlrpc.client.ServerProxy,
        pos_int: int,
    ) -> None:
        """Pause servo, send MoveGripper, resume servo.

        The Fairino controller rejects ``MoveGripper`` while
        servo mode is active.  This helper runs entirely on
        the servo thread's proxy so there are no thread-safety
        issues.

        The pause is brief (~100 ms total) and the robot holds
        its last commanded position during the gap.
        """
        cfg = self.config
        try:
            proxy.ServoMoveEnd()
            time.sleep(0.02)

            ret = proxy.MoveGripper(
                int(cfg.gripper_index),
                pos_int,
                int(cfg.gripper_vel),
                int(cfg.gripper_force),
                30000,  # maxtime [ms]
                1,      # block: 1=non-blocking
                0,      # type=parallel
                0.0, 0, 0,
            )
            if ret != 0:
                logger.warning(
                    "[Fairino] MoveGripper -> %d%% "
                    "FAILED (err %d)",
                    pos_int, ret,
                )
            else:
                logger.info(
                    "[Fairino] Gripper -> %d%%", pos_int,
                )

            time.sleep(0.02)
            proxy.ServoMoveStart()
            time.sleep(0.02)
        except Exception as exc:
            logger.warning(
                "[Fairino] Gripper servo-pause err: %s",
                exc,
            )
            # Try to recover servo mode.
            with contextlib.suppress(Exception):
                proxy.ServoMoveStart()

    # ---- gripper helpers ------------------------------------

    def _initialise_gripper(self) -> None:
        """Configure and activate the gripper.

        Calls SetGripperConfig, then resets and activates
        the gripper at the configured index.
        """
        cfg = self.config
        # All 4 args are required by the XMLRPC protocol;
        # the SDK wrapper has Python defaults but the raw
        # proxy does not.
        ret = self._rpc.SetGripperConfig(
            cfg.gripper_company, cfg.gripper_device, 0, 0,
        )
        if ret != 0:
            logger.warning(
                "[Fairino] SetGripperConfig err: %d", ret,
            )
        time.sleep(_SETTLE_MID_S)

        # Reset then activate.
        self._rpc.ActGripper(cfg.gripper_index, 0)
        time.sleep(_SETTLE_LONG_S)
        ret = self._rpc.ActGripper(cfg.gripper_index, 1)
        if ret != 0:
            logger.warning(
                "[Fairino] ActGripper err: %d", ret,
            )
        time.sleep(1.0)

        # Read initial position.
        self._gripper_pos = self._read_gripper_pos()
        self._last_gripper_cmd = self._gripper_pos
        logger.info(
            "[Fairino] Gripper activated (pos=%.0f%%).",
            self._gripper_pos,
        )

    def _read_gripper_pos(self) -> float:
        """Read the current gripper position [0-100 %].

        Uses the state-packet cache when TCP is connected,
        otherwise falls back to the last commanded position.
        """
        if not self._use_xmlrpc_reads:
            try:
                result = (
                    self._rpc.GetGripperCurPosition()
                )
                if isinstance(result, tuple) and len(result) >= 3:
                    if result[0] == 0:
                        self._gripper_pos = float(
                            result[2]
                        )
                        return self._gripper_pos
            except Exception as exc:
                logger.debug(
                    "[Fairino] Gripper read err: %s", exc,
                )
        # Fallback: return last known position.
        return self._gripper_pos

    def _send_gripper_cmd(self, pos: float) -> None:
        """Queue a gripper command for the servo loop.

        The Fairino controller blocks ``MoveGripper`` while
        servo mode is active.  Instead of calling XMLRPC from
        the main thread, we set ``_pending_gripper`` which the
        servo loop picks up on its next iteration: it pauses
        servo, sends MoveGripper, and resumes.

        Skips when the new position is within 1% of the last
        command to avoid unnecessary servo pauses.

        Args:
            pos: Target position [0-100 %].
        """
        if abs(pos - self._last_gripper_cmd) < 1.0:
            return
        self._last_gripper_cmd = pos
        self._gripper_pos = pos
        self._pending_gripper = int(pos)

    # ---- servo helpers ------------------------------------

    def _initialise_servo_mode(self) -> None:
        """Run the full enable-and-servo startup sequence.

        The sequence clears residual servo state from a
        previous (possibly crashed) session, then transitions
        the controller to auto-mode servo control:
            ServoMoveEnd → RobotEnable(0) → ResetAllError
            → RobotEnable(1) → Mode(0) → ServoMoveStart
        """
        # Clean up any stale servo session left by a
        # previous run that did not disconnect cleanly.
        with contextlib.suppress(Exception):
            self._rpc.ServoMoveEnd()
        time.sleep(_SETTLE_SHORT_S)

        self._rpc.RobotEnable(0)
        time.sleep(_SETTLE_MID_S)
        self._rpc.ResetAllError()
        time.sleep(_SETTLE_MID_S)
        self._rpc.RobotEnable(1)
        time.sleep(_SETTLE_MID_S)
        self._rpc.Mode(0)
        time.sleep(_SETTLE_LONG_S)

        ret = self._rpc.ServoMoveStart()
        if ret != 0:
            logger.warning(
                "[Fairino] ServoMoveStart error: %d", ret
            )
        time.sleep(_SETTLE_MID_S)
