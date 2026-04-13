#!/usr/bin/env python

# Copyright 2026 APPEAL Automation team. All rights reserved.
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

"""Fairino FR5 leader teleoperator for drag-teach teleoperation.

The leader arm enters freedrive (drag-teach) mode so an operator
can physically guide it.  Each ``get_action()`` call reads the
current joint positions and returns them as an absolute-position
action dict, which the follower robot mirrors via ServoJ.

Communication uses the same dual-channel SDK as the follower:
    XMLRPC (port 20003) -- command interface
    TCP    (port 20004) -- real-time state feedback (~50 Hz)
"""

import contextlib
import logging
import time
from typing import Any

from lerobot.types import RobotAction

from ..teleoperator import Teleoperator
from .config_fairino_leader import FairinoLeaderConfig

logger = logging.getLogger(__name__)

# Timing constants for the hardware init sequence.
_SETTLE_SHORT_S = 0.2
_SETTLE_MID_S = 0.3

# Joint-read retry parameters.
_READ_RETRIES = 3
_READ_RETRY_DELAY_S = 0.05


class FairinoLeader(Teleoperator):
    """Read joint positions from an FR5 in drag-teach mode.

    The robot is put into freedrive so the operator can move
    it by hand.  ``get_action()`` returns the current joint
    angles as an absolute-position dict compatible with
    ``FairinoFollower.send_action()``.

    Attributes:
        config: Frozen dataclass with IP, joint names.
    """

    config_class = FairinoLeaderConfig
    name = "fairino_leader"

    def __init__(self, config: FairinoLeaderConfig):
        super().__init__(config)
        self.config = config
        self._rpc = None
        self._is_connected = False
        self._use_xmlrpc_reads = False

    # ---- properties (required by Teleoperator) ----------------

    @property
    def action_features(self) -> dict:
        """Action schema: six joint positions [deg]."""
        return {f"{jname}.pos": float for jname in self.config.joint_names}

    @property
    def feedback_features(self) -> dict:
        """Feedback schema (unused -- leader needs no feedback)."""
        return {f"{jname}.pos": float for jname in self.config.joint_names}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        """Fairino uses absolute encoders; always calibrated."""
        return True

    # ---- lifecycle --------------------------------------------

    def connect(self, calibrate: bool = True) -> None:
        """Connect to the FR5 and enter drag-teach mode.

        Sequence:
            1. Probe TCP 20004, create SDK RPC object
            2. RobotEnable(0) -> ResetAllError -> RobotEnable(1)
            3. DragTeachSwitch(1)  -- freedrive mode

        Args:
            calibrate: Ignored (absolute encoders).

        Raises:
            RuntimeError: If already connected.
            ConnectionError: If the controller is unreachable.
        """
        if self._is_connected:
            raise RuntimeError("[FairinoLeader] Already connected.")

        logger.info(
            "[FairinoLeader] Connecting to %s ...",
            self.config.ip_address,
        )

        tcp_ok = self._probe_tcp(
            self.config.ip_address,
            20004,
            timeout_s=2.0,
        )

        if tcp_ok:
            from lerobot.robots.fairino.fairino.Robot import (
                RPC as FairinoRPC,  # noqa: N811
            )

            self._rpc = FairinoRPC(self.config.ip_address)
            try:
                self._wait_for_state_data(timeout_s=3.0)
            except ConnectionError:
                tcp_ok = False

        if not tcp_ok:
            logger.warning("[FairinoLeader] TCP 20004 unavailable; using XMLRPC-only mode.")
            self._init_xmlrpc_only()

        self._use_xmlrpc_reads = not tcp_ok

        # Verify communication.
        ret, joints = self._read_joints()
        if ret != 0:
            self._rpc = None
            raise ConnectionError(f"[FairinoLeader] Connection failed (err {ret})")

        # Enable robot and enter drag-teach mode.
        self._enter_drag_teach()

        self._is_connected = True
        logger.info(
            "[FairinoLeader] Connected in drag-teach mode. Joints (deg): %s",
            joints,
        )

    def disconnect(self) -> None:
        """Exit drag-teach mode and release RPC resources."""
        if self._rpc is not None:
            with contextlib.suppress(Exception):
                self._rpc.DragTeachSwitch(0)
            logger.info("[FairinoLeader] Drag-teach mode exited.")
            with contextlib.suppress(Exception):
                self._rpc.CloseRPC()
            self._rpc = None

        self._is_connected = False
        logger.info("[FairinoLeader] Disconnected.")

    def calibrate(self) -> None:
        """No-op: Fairino uses absolute encoders."""

    def configure(self) -> None:
        """No-op: no runtime configuration needed."""

    # ---- action / feedback ------------------------------------

    def get_action(self) -> RobotAction:
        """Read current joint positions from the leader arm.

        Returns:
            Dict with "joint{1..6}.pos" angles [deg].

        Raises:
            RuntimeError: If the joint query fails.
        """
        self._assert_connected()

        ret, joints_deg = self._read_joints()
        if ret != 0:
            raise RuntimeError(f"[FairinoLeader] Joint read failed (err {ret})")

        return {f"{jname}.pos": joints_deg[i] for i, jname in enumerate(self.config.joint_names)}

    def send_feedback(
        self,
        feedback: dict[str, Any],
    ) -> None:
        """No-op: the leader does not need follower feedback."""

    # ---- private helpers --------------------------------------

    def _assert_connected(self) -> None:
        """Raise if not connected."""
        if not self._is_connected or self._rpc is None:
            raise RuntimeError("[FairinoLeader] Not connected. Call connect().")

    def _enter_drag_teach(self) -> None:
        """Enable the robot and activate drag-teach mode.

        Sequence: RobotEnable(0) -> ResetAllError
        -> RobotEnable(1) -> DragTeachSwitch(1).
        """
        self._rpc.RobotEnable(0)
        time.sleep(_SETTLE_MID_S)
        self._rpc.ResetAllError()
        time.sleep(_SETTLE_MID_S)
        self._rpc.RobotEnable(1)
        time.sleep(_SETTLE_MID_S)

        ret = self._rpc.DragTeachSwitch(1)
        if ret != 0:
            logger.warning(
                "[FairinoLeader] DragTeachSwitch(1) error: %d",
                ret,
            )
        time.sleep(_SETTLE_SHORT_S)

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

    def _wait_for_state_data(
        self,
        timeout_s: float = 5.0,
    ) -> None:
        """Block until the TCP thread populates state data.

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
        raise ConnectionError(f"[FairinoLeader] Timed out waiting for state data ({timeout_s}s).")

    def _init_xmlrpc_only(self) -> None:
        """Create a minimal RPC-like namespace for XMLRPC-only.

        Used when TCP 20004 is unavailable.
        """
        import types
        import xmlrpc.client

        link = f"http://{self.config.ip_address}:20003"
        proxy = xmlrpc.client.ServerProxy(link)

        rpc = types.SimpleNamespace()
        rpc.robot = proxy
        rpc.RobotEnable = proxy.RobotEnable
        rpc.ResetAllError = proxy.ResetAllError
        rpc.DragTeachSwitch = proxy.DragTeachSwitch
        rpc.IsInDragTeach = proxy.IsInDragTeach
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
                    "[FairinoLeader] Joint read retry %d/%d (err %d)",
                    attempt + 1,
                    _READ_RETRIES,
                    ret,
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
                    "[FairinoLeader] SDK joint read error: %s",
                    exc,
                )
                return -1, [0.0] * 6

            if isinstance(result, tuple):
                return result[0], list(result[1])
            return int(result), [0.0] * 6

        # XMLRPC fallback.
        try:
            result = self._rpc.robot.GetActualJointPosDegree(1)
            if result[0] == 0:
                return 0, list(result[1:7])
            return result[0], [0.0] * 6
        except Exception as exc:
            logger.warning(
                "[FairinoLeader] XMLRPC joint read error: %s",
                exc,
            )
            return -1, [0.0] * 6
