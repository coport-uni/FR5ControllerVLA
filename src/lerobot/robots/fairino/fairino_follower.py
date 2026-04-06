#!/usr/bin/env python

# Copyright APPEAL Automation team. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation

from .config_fairino_follower import FairinoFollowerConfig
from .fairino.Robot import RPC as FairinoRPC

logger = logging.getLogger(__name__)


class FairinoFollower(Robot):
    """
    LeRobot-compatible wrapper for the Fairino FR5 6-DOF collaborative robot.

    Communicates with the Fairino controller via XMLRPC (port 20003) and
    receives real-time state feedback via TCP socket (port 20004).

    Actions and observations use the flat-dict contract expected by LeRobot:
        observation keys: "joint1.pos" .. "joint6.pos", plus camera images
        action keys:      "joint1.pos" .. "joint6.pos"
    All joint values are in **degrees** (the native Fairino unit).
    """

    config_class = FairinoFollowerConfig
    name = "fairino_follower"

    def __init__(self, config: FairinoFollowerConfig):
        super().__init__(config)
        self.config = config
        self._rpc = None  # Fairino SDK RPC handle
        self._is_connected = False
        self.cameras = {}

    # ------------------------------------------------------------------
    # Abstract property implementations
    # ------------------------------------------------------------------

    @property
    def observation_features(self) -> dict:
        features = {}
        for jname in self.config.joint_names:
            features[f"{jname}.pos"] = float
        for cam_name, cam_cfg in self.config.cameras.items():
            features[f"observation.images.{cam_name}"] = (cam_cfg.height, cam_cfg.width, 3)
        return features

    @property
    def action_features(self) -> dict:
        return {f"{jname}.pos": float for jname in self.config.joint_names}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        # Fairino has absolute encoders; no calibration needed.
        return True

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise RuntimeError("[Fairino] Already connected.")

        logger.info(f"[Fairino] Connecting to {self.config.ip_address} ...")
        self._rpc = FairinoRPC(self.config.ip_address)

        # Verify communication by reading joint positions
        ret, joints = self._rpc.GetActualJointPosDegree()
        if ret != 0:
            self._rpc = None
            raise ConnectionError(
                f"[Fairino] Failed to connect (error code: {ret})"
            )

        # Clean up any leftover servo session, then initialise into a
        # known-good state:
        #   ServoMoveEnd (ignore errors) → RobotEnable(0) → ResetAllError
        #   → RobotEnable(1) → Mode(0) → ServoMoveStart

        try:
            self._rpc.ServoMoveEnd()
        except Exception:
            pass
        time.sleep(0.2)
        self._rpc.RobotEnable(0)
        time.sleep(0.3)
        self._rpc.ResetAllError()
        time.sleep(0.3)
        self._rpc.RobotEnable(1)
        time.sleep(0.3)
        self._rpc.Mode(0)
        time.sleep(0.5)

        # Start servo mode for real-time joint control
        ret_servo = self._rpc.ServoMoveStart()
        if ret_servo != 0:
            logger.warning(f"[Fairino] ServoMoveStart returned {ret_servo}")
        time.sleep(0.3)
        logger.info("[Fairino] Robot enabled in auto mode (servo).")

        self._is_connected = True
        logger.info(f"[Fairino] Connected. Current joints (deg): {joints}")

        # Connect cameras
        if self.config.cameras:
            self.cameras = make_cameras_from_configs(self.config.cameras)
            for cam_name, cam in self.cameras.items():
                cam.connect()
                logger.info(f"[Fairino] Camera '{cam_name}' connected.")

    def disconnect(self) -> None:
        # Disconnect cameras first
        for cam_name, cam in self.cameras.items():
            cam.disconnect()
            logger.info(f"[Fairino] Camera '{cam_name}' disconnected.")

        if self._rpc is not None:
            try:
                self._rpc.ServoMoveEnd()
            except Exception:
                pass
            try:
                self._rpc.CloseRPC()
            except Exception:
                pass
            self._rpc = None

        self._is_connected = False
        logger.info("[Fairino] Disconnected.")

    def calibrate(self) -> None:
        # Fairino uses absolute encoders — nothing to calibrate.
        pass

    def configure(self) -> None:
        pass

    # ------------------------------------------------------------------
    # Observation / Action
    # ------------------------------------------------------------------

    def get_observation(self) -> RobotObservation:
        self._assert_connected()

        obs: dict[str, Any] = {}

        # Read joint positions in degrees
        ret, joints_deg = self._rpc.GetActualJointPosDegree()
        if ret != 0:
            raise RuntimeError(f"[Fairino] Failed to read joints (error code: {ret})")

        for i, jname in enumerate(self.config.joint_names):
            obs[f"{jname}.pos"] = joints_deg[i]

        # Read camera images
        for cam_name, cam in self.cameras.items():
            obs[f"observation.images.{cam_name}"] = cam.async_read()

        return obs

    def send_action(self, action: RobotAction) -> RobotAction:
        """
        Send joint-position targets (degrees) to the Fairino robot via ServoJ.

        Uses real-time servo mode (ServoJ) for low-latency control, which is
        required for VLA inference and keyboard teleoperation.

        Args:
            action: dict with keys "joint1.pos" .. "joint6.pos", values in degrees.

        Returns:
            The clamped action that was actually sent (degrees).
        """
        self._assert_connected()

        # Build target joint list (degrees)
        target_deg = []
        for jname in self.config.joint_names:
            key = f"{jname}.pos"
            target_deg.append(float(action[key]))

        # Clamp to joint limits
        lower = self.config.joint_limits_lower
        upper = self.config.joint_limits_upper
        clamped_deg = [
            max(lo, min(hi, val))
            for val, lo, hi in zip(target_deg, lower, upper)
        ]

        # Send via ServoJ (direct XMLRPC, 7 params — no 'id' arg for this FW)
        cmd_t = 1.0 / self.config.control_hz
        axis_pos = [0.0, 0.0, 0.0, 0.0]
        ret = self._rpc.robot.ServoJ(
            clamped_deg, axis_pos, 0.0, 0.0, cmd_t, 0.0, 0.0,
        )

        if ret != 0:
            logger.warning(f"[Fairino] ServoJ returned error code: {ret}")

        # Return the action that was actually sent
        sent_action = {}
        for i, jname in enumerate(self.config.joint_names):
            sent_action[f"{jname}.pos"] = clamped_deg[i]
        return sent_action

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _assert_connected(self) -> None:
        if not self._is_connected or self._rpc is None:
            raise RuntimeError("[Fairino] Robot is not connected. Call connect() first.")

    def get_joint_positions_deg(self) -> list[float]:
        """Convenience: return current joint positions in degrees as a plain list."""
        self._assert_connected()
        ret, joints = self._rpc.GetActualJointPosDegree()
        if ret != 0:
            raise RuntimeError(f"[Fairino] Failed to read joints (error code: {ret})")
        return list(joints)

    def get_tcp_pose(self) -> list[float]:
        """Convenience: return current TCP pose [x,y,z,rx,ry,rz] (mm/deg)."""
        self._assert_connected()
        ret, pose = self._rpc.GetActualTCPPose()
        if ret != 0:
            raise RuntimeError(f"[Fairino] Failed to read TCP pose (error code: {ret})")
        return list(pose)

    def stop_motion(self) -> None:
        """Emergency stop the robot motion."""
        if self._rpc is not None:
            self._rpc.StopMotion()

    def reset_errors(self) -> None:
        """Clear robot error state, re-enable, set auto mode, and restart servo."""
        if self._rpc is not None:
    
            try:
                self._rpc.ServoMoveEnd()
            except Exception:
                pass
            self._rpc.RobotEnable(0)
            time.sleep(0.3)
            self._rpc.ResetAllError()
            time.sleep(0.3)
            self._rpc.RobotEnable(1)
            time.sleep(0.3)
            self._rpc.Mode(0)
            time.sleep(0.5)
            self._rpc.ServoMoveStart()
            time.sleep(0.3)
            logger.info("[Fairino] Errors reset, robot re-enabled in auto mode (servo).")
