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
import time
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

    # ---- properties (required by Robot base class) ---------

    @property
    def observation_features(self) -> dict:
        """Return observation schema (works when disconnected)."""
        features = {}
        for jname in self.config.joint_names:
            features[f"{jname}.pos"] = float
        for cam_name, cam_cfg in self.config.cameras.items():
            key = f"observation.images.{cam_name}"
            features[key] = (
                cam_cfg.height, cam_cfg.width, 3
            )
        return features

    @property
    def action_features(self) -> dict:
        """Return action schema (works when disconnected)."""
        return {
            f"{jname}.pos": float
            for jname in self.config.joint_names
        }

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

    def disconnect(self) -> None:
        """End servo mode, close cameras, and release RPC."""
        for cam_name, cam in self.cameras.items():
            cam.disconnect()
            logger.info(
                "[Fairino] Camera '%s' disconnected.",
                cam_name,
            )

        if self._rpc is not None:
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

        for cam_name, cam in self.cameras.items():
            key = f"observation.images.{cam_name}"
            obs[key] = cam.async_read()

        return obs

    def send_action(self, action: RobotAction) -> RobotAction:
        """Send joint targets [deg] via ServoJ.

        Targets are clamped to the configured joint limits
        before transmission.

        Args:
            action: Dict with "joint{1..6}.pos" in degrees.

        Returns:
            Dict of the clamped values actually sent.
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
            for val, lo, hi in zip(target_deg, lower, upper, strict=True)
        ]

        # The Fairino controller expects cmdT in [0.001 .. 0.016]s
        # (i.e. 62–1000 Hz internal servo rate).  We use the SDK
        # default 0.008s regardless of the teleop loop rate.
        _SERVO_CMD_T = 0.008
        axis_pos = [0.0, 0.0, 0.0, 0.0]

        if self._use_xmlrpc_reads:
            # XMLRPC-only path — call raw XMLRPC.
            ret = self._rpc.robot.ServoJ(
                clamped_deg, axis_pos,
                0.0, 0.0, _SERVO_CMD_T, 0.0, 0.0,
            )
        else:
            # Full SDK path — use the wrapper (handles safety).
            ret = self._rpc.ServoJ(
                clamped_deg, axis_pos,
                acc=0.0, vel=0.0, cmdT=_SERVO_CMD_T,
                filterT=0.0, gain=0.0,
            )

        if ret != 0:
            logger.warning(
                "[Fairino] ServoJ error code: %d", ret
            )

        return {
            f"{jname}.pos": clamped_deg[i]
            for i, jname in enumerate(self.config.joint_names)
        }

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
            with contextlib.suppress(Exception):
                self._rpc.ServoMoveEnd()
            self._initialise_servo_mode()
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
        rpc.CloseRPC = lambda: None
        self._rpc = rpc

    def _read_joints(self) -> tuple[int, list[float]]:
        """Read joint positions, using the best available method.

        When TCP 20004 is connected, the SDK's
        ``GetActualJointPosDegree`` reads from the fast
        state-packet cache.  When TCP is unavailable we
        fall back to a direct XMLRPC call.

        Returns:
            (error_code, [j1..j6]) in degrees.
        """
        if not self._use_xmlrpc_reads:
            return self._rpc.GetActualJointPosDegree()

        # XMLRPC fallback.
        try:
            result = self._rpc.robot.GetActualJointPosDegree(1)
            if result[0] == 0:
                return 0, list(result[1:7])
            return result[0], [0.0] * 6
        except Exception as exc:
            logger.warning(
                "[Fairino] XMLRPC joint read error: %s", exc
            )
            return -1, [0.0] * 6

    def _initialise_servo_mode(self) -> None:
        """Run the full enable-and-servo startup sequence.

        The sequence clears residual errors from a previous
        session and transitions the controller to auto-mode
        servo control.
        """
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
