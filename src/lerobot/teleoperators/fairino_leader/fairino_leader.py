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
import threading
import time
import xmlrpc.client
from typing import Any

from lerobot.types import RobotAction

from ..teleoperator import Teleoperator
from .config_fairino_leader import FairinoLeaderConfig

logger = logging.getLogger(__name__)

# Timing constants for the hardware init sequence.
_SETTLE_SHORT_S = 0.2
_SETTLE_MID_S = 0.3
_SETTLE_LONG_S = 0.5

# Joint-read retry parameters.
_READ_RETRIES = 3
_READ_RETRY_DELAY_S = 0.05

# Minimum seconds between controller-fault recovery attempts.
# A joint dragged into its limit faults the controller; this
# keeps the re-enter-drag-teach sequence from hammering it.
_RECOVERY_MIN_INTERVAL_S = 2.0

# MoveGripper maxtime [ms] -- int32 max (~24.8 days).  The
# SDK rejects -1, 600000 (10 min) expired mid-session
# (issue #24), and even int32 max was observed to expire in
# practice -- so we also run a periodic keepalive thread
# that re-issues MoveGripper while the leader is idle.
# The controller stops when ActGripper(0) is issued on disconnect.
_GRIPPER_MAXTIME_MS = 2147483647

# Keepalive worker timings.
_KEEPALIVE_POLL_S = 1.0
_KEEPALIVE_JOIN_TIMEOUT_S = 2.0


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

        # Last-known gripper position [0-100 %].  Updated from
        # hardware in ``get_action()``; only used as a fallback
        # value if the read ever fails.
        self._gripper_pos: float = 0.0
        self._gripper_initialised: bool = False

        # Gripper keepalive worker -- a separate ServerProxy is
        # used because xmlrpc.client transports aren't
        # thread-safe; the follower's gripper worker follows the
        # same pattern.  Activity tracking lets the worker tell
        # idle vs. active periods apart.
        self._keepalive_rpc: xmlrpc.client.ServerProxy | None = None
        self._keepalive_thread: threading.Thread | None = None
        self._keepalive_stop: threading.Event = threading.Event()
        self._keepalive_active: threading.Event = threading.Event()
        self._last_activity_time: float = 0.0
        self._last_joint_pos: list[float] | None = None
        self._last_gripper_pos_for_activity: float = 0.0

        # Gripper position reported to the follower.  While a
        # keepalive nudge is in progress, this value stays
        # frozen at the pre-nudge read so the follower does not
        # mirror the ~1 % wobble.  Only the leader hardware
        # moves during keepalive; the follower stays put.
        self._reported_gripper_pos: float = 0.0

        # Last good joint reading.  get_action() serves this
        # when a read fails so the teleop loop keeps running
        # instead of crashing on a RuntimeError.
        self._last_joints: list[float] | None = None
        # Edge-trigger flag for read-fallback logging.
        self._joint_read_ok: bool = True
        # Monotonic timestamp of the last drag-teach recovery,
        # used to rate-limit recovery attempts.
        self._last_recovery_time: float = 0.0

    # ---- properties (required by Teleoperator) ----------------

    @property
    def action_features(self) -> dict:
        """Action schema: joint positions [deg] + optional gripper [%]."""
        features = {f"{jname}.pos": float for jname in self.config.joint_names}
        if self.config.gripper_enabled:
            features["gripper.pos"] = float
        return features

    @property
    def feedback_features(self) -> dict:
        """Feedback schema (unused -- leader needs no feedback)."""
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
            from lerobot.robots.fairino_follower.fairino.Robot import (
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

        # Activate the gripper in low-force compliance mode so
        # the operator can drag the jaws by hand.
        if self.config.gripper_enabled:
            self._initialise_gripper()
            if self.config.gripper_keepalive_enabled:
                self._start_keepalive()

        self._is_connected = True
        logger.info(
            "[FairinoLeader] Connected in drag-teach mode. Joints (deg): %s",
            joints,
        )

    def disconnect(self) -> None:
        """Exit drag-teach mode and release RPC resources."""
        # Tear down the keepalive worker first so it cannot race
        # with ActGripper(0) or CloseRPC() below.
        self._stop_keepalive()

        if self._rpc is not None:
            if self.config.gripper_enabled:
                with contextlib.suppress(Exception):
                    self._rpc.ActGripper(
                        self.config.gripper_index,
                        0,
                    )
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
        """Read live joint and gripper positions.

        Joints come from drag-teach; the gripper position is
        read from the controller each loop so the follower
        can mirror the operator's hand motion of the jaws.

        Returns:
            Dict with "joint{1..6}.pos" [deg] and optionally
            "gripper.pos" [0-100 %].

        Raises:
            RuntimeError: If the joint query fails.
        """
        self._assert_connected()

        # If the operator dragged a joint into its limit, the
        # controller faults and drops out of drag-teach.  Detect
        # it from the state package and re-enter drag-teach so
        # the arm stays back-drivable.
        if self._detect_fault():
            self._recover_from_fault()

        ret, joints_deg = self._read_joints()
        if ret != 0:
            # The read can stall for several ticks while a fault
            # is active.  Recover and serve the last good reading
            # instead of raising so the teleop loop survives.
            self._recover_from_fault()
            if self._last_joints is None:
                raise RuntimeError(f"[FairinoLeader] Joint read failed (err {ret})")
            if self._joint_read_ok:
                logger.warning(
                    "[FairinoLeader] Joint read failed (err %d); serving last-known position.",
                    ret,
                )
                self._joint_read_ok = False
            joints_deg = self._last_joints
        else:
            if not self._joint_read_ok:
                logger.info("[FairinoLeader] Joint read recovered.")
                self._joint_read_ok = True
            self._last_joints = list(joints_deg)

        action = {f"{jname}.pos": joints_deg[i] for i, jname in enumerate(self.config.joint_names)}

        if self.config.gripper_enabled:
            live_pos = self._read_gripper_pos()
            # Mask keepalive-induced wobble from the follower:
            # only refresh the reported position when the
            # keepalive worker is not currently nudging the
            # leader's jaws.  Otherwise the follower would
            # mirror a jitter that is only meant to refresh
            # the leader's compliance timer.
            if not self._keepalive_active.is_set():
                self._reported_gripper_pos = live_pos
            action["gripper.pos"] = self._reported_gripper_pos

        # Activity tracker for the keepalive worker.  Skip while
        # the worker is nudging so our own motions don't reset
        # the idle clock.
        if (
            self.config.gripper_enabled
            and self.config.gripper_keepalive_enabled
            and not self._keepalive_active.is_set()
        ):
            self._update_activity(
                joints_deg,
                float(action.get("gripper.pos", self._gripper_pos)),
            )

        return action

    def send_feedback(
        self,
        feedback: dict[str, Any],
    ) -> None:
        """Initialise gripper target from the follower's state.

        Only the first call initialises ``_gripper_pos`` from
        ``feedback["gripper.pos"]``.  Subsequent calls are
        ignored so the keyboard retains authority over the
        gripper target.  Without this, the leader would send
        ``gripper.pos=0`` on the first iteration and force the
        physical gripper to fully close.

        Args:
            feedback: Follower observation dict; may contain
                ``"gripper.pos"`` in [0, 100] %.
        """
        if self._gripper_initialised:
            return
        if not self.config.gripper_enabled:
            return
        if "gripper.pos" not in feedback:
            return
        self._gripper_pos = float(feedback["gripper.pos"])
        self._gripper_initialised = True
        logger.info(
            "[FairinoLeader] Gripper target initialised: %.0f%%",
            self._gripper_pos,
        )

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

    def _detect_fault(self) -> bool:
        """Return True if the controller reports an active fault.

        Reads the main error code and both safety-stop flags from
        the real-time state package, so it adds no RPC round-trip.
        XMLRPC-only mode cannot see these fields and reports no
        fault.

        Returns:
            True if a main fault code is set or either safety
            stop is asserted; False otherwise.
        """
        if self._use_xmlrpc_reads:
            return False
        try:
            pkg = self._rpc.robot_state_pkg
            return pkg.main_code != 0 or pkg.safety_stop0_state == 1 or pkg.safety_stop1_state == 1
        except Exception:
            return False

    def _recover_from_fault(self) -> None:
        """Clear a fault and re-enter drag-teach mode.

        On the leader a fault usually means a joint was dragged
        into its limit.  Re-running ResetAllError, RobotEnable(1)
        and DragTeachSwitch(1) restores back-drive.  Rate-limited
        so a persistent fault does not hammer the controller.
        """
        now = time.monotonic()
        if now - self._last_recovery_time < _RECOVERY_MIN_INTERVAL_S:
            return
        self._last_recovery_time = now
        logger.warning("[FairinoLeader] Controller fault; re-entering drag-teach ...")
        try:
            self._rpc.ResetAllError()
            time.sleep(_SETTLE_MID_S)
            self._rpc.RobotEnable(1)
            time.sleep(_SETTLE_MID_S)
            ret = self._rpc.DragTeachSwitch(1)
            if ret != 0:
                logger.warning(
                    "[FairinoLeader] DragTeachSwitch(1) err: %d",
                    ret,
                )
            logger.info("[FairinoLeader] Drag-teach recovered.")
        except Exception as exc:
            logger.warning(
                "[FairinoLeader] Recovery failed: %s",
                exc,
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

        link = f"http://{self.config.ip_address}:20003"
        proxy = xmlrpc.client.ServerProxy(link)

        rpc = types.SimpleNamespace()
        rpc.robot = proxy
        rpc.RobotEnable = proxy.RobotEnable
        rpc.ResetAllError = proxy.ResetAllError
        rpc.DragTeachSwitch = proxy.DragTeachSwitch
        rpc.IsInDragTeach = proxy.IsInDragTeach
        rpc.SetGripperConfig = proxy.SetGripperConfig
        rpc.ActGripper = proxy.ActGripper
        rpc.MoveGripper = proxy.MoveGripper
        rpc.GetGripperCurPosition = proxy.GetGripperCurPosition
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

    # ---- gripper helpers --------------------------------------

    def _initialise_gripper(self) -> None:
        """Activate the gripper in low-force compliance mode.

        SetGripperConfig + ActGripper(1) turn the gripper on,
        then a non-blocking MoveGripper at the current position
        with ``gripper_force`` as the holding force lets the
        operator back-drive the jaws by hand.  Targeting the
        current position (instead of a hardcoded 0) prevents
        the jaws from slowly drifting closed when the operator
        isn't touching them -- that drift would otherwise be
        mirrored by the follower.
        """
        cfg = self.config
        ret = self._rpc.SetGripperConfig(
            cfg.gripper_company,
            cfg.gripper_device,
            0,
            0,
        )
        if ret != 0:
            logger.warning(
                "[FairinoLeader] SetGripperConfig err: %d",
                ret,
            )
        time.sleep(_SETTLE_MID_S)

        self._rpc.ActGripper(cfg.gripper_index, 0)
        time.sleep(_SETTLE_LONG_S)
        ret = self._rpc.ActGripper(cfg.gripper_index, 1)
        if ret != 0:
            logger.warning(
                "[FairinoLeader] ActGripper err: %d",
                ret,
            )
        time.sleep(1.0)

        # Read the current jaw position before issuing the
        # compliance command so we hold in place rather than
        # drifting toward any particular target.
        current_pos = self._read_gripper_pos()
        hold_target = int(round(current_pos))

        with contextlib.suppress(Exception):
            self._rpc.MoveGripper(
                int(cfg.gripper_index),
                hold_target,
                int(cfg.gripper_vel),
                int(cfg.gripper_force),
                _GRIPPER_MAXTIME_MS,
                0,
                0,
                0.0,
                0,
                0,
            )

        self._gripper_pos = self._read_gripper_pos()
        logger.info(
            "[FairinoLeader] Gripper compliant (pos=%.0f%%, force=%d).",
            self._gripper_pos,
            cfg.gripper_force,
        )

    def _read_gripper_pos(self) -> float:
        """Read the current gripper position [0-100 %].

        Falls back to the last-known value if the controller
        returns an error or unexpected payload.
        """
        try:
            result = self._rpc.GetGripperCurPosition()
            if isinstance(result, tuple) and len(result) >= 3 and result[0] == 0:
                self._gripper_pos = float(result[2])
        except Exception as exc:
            logger.debug(
                "[FairinoLeader] Gripper read err: %s",
                exc,
            )
        return self._gripper_pos

    # ---- gripper keepalive ------------------------------------

    def _update_activity(
        self,
        joints_deg: list[float],
        gripper_pct: float,
    ) -> None:
        """Refresh activity timestamp if joints or gripper moved.

        Compares the current joint angles and gripper position to
        the last snapshot.  A change in any joint greater than
        ``gripper_idle_joint_deg`` or a gripper delta greater than
        ``gripper_idle_pos_pct`` counts as activity.  The first
        call (``_last_joint_pos is None``) is also treated as
        activity so the idle timer starts counting from a known
        baseline rather than t=0.
        """
        cfg = self.config
        moved = False
        if self._last_joint_pos is None:
            moved = True
        else:
            for cur, prev in zip(joints_deg, self._last_joint_pos, strict=True):
                if abs(cur - prev) > cfg.gripper_idle_joint_deg:
                    moved = True
                    break
            if (
                not moved
                and abs(gripper_pct - self._last_gripper_pos_for_activity) > cfg.gripper_idle_pos_pct
            ):
                moved = True

        self._last_joint_pos = list(joints_deg)
        self._last_gripper_pos_for_activity = float(gripper_pct)
        if moved:
            self._last_activity_time = time.monotonic()

    def _start_keepalive(self) -> None:
        """Spin up the gripper-compliance refresh worker."""
        link = f"http://{self.config.ip_address}:20003"
        self._keepalive_rpc = xmlrpc.client.ServerProxy(link)
        self._keepalive_stop.clear()
        self._keepalive_active.clear()
        self._last_activity_time = time.monotonic()
        self._last_joint_pos = None
        self._last_gripper_pos_for_activity = float(self._gripper_pos)
        self._reported_gripper_pos = float(self._gripper_pos)
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_loop,
            name="FairinoLeaderKeepalive",
            daemon=True,
        )
        self._keepalive_thread.start()
        logger.info(
            "[FairinoLeader] Gripper keepalive started (interval=%.1fs, nudge=%.1f%%).",
            self.config.gripper_keepalive_interval_s,
            self.config.gripper_nudge_pct,
        )

    def _stop_keepalive(self) -> None:
        """Signal the keepalive worker to exit and join it."""
        if self._keepalive_thread is not None:
            self._keepalive_stop.set()
            self._keepalive_thread.join(
                timeout=_KEEPALIVE_JOIN_TIMEOUT_S,
            )
            self._keepalive_thread = None
        # ServerProxy has no explicit close method.
        self._keepalive_rpc = None

    def _keepalive_loop(self) -> None:
        """Re-issue MoveGripper during idle periods.

        When neither the joints nor the gripper have moved for at
        least ``gripper_keepalive_interval_s`` seconds, nudge the
        gripper by ``gripper_nudge_pct`` % and then return to the
        original position.  Each MoveGripper call restarts the
        controller's compliance timer, which is what keeps the
        jaws back-drivable after long idle periods.
        """
        cfg = self.config
        interval_s = float(cfg.gripper_keepalive_interval_s)
        settle_s = float(cfg.gripper_nudge_settle_s)
        nudge_pct = float(cfg.gripper_nudge_pct)

        while not self._keepalive_stop.is_set():
            if self._keepalive_stop.wait(_KEEPALIVE_POLL_S):
                break
            idle_s = time.monotonic() - self._last_activity_time
            if idle_s < interval_s:
                continue

            current = float(self._gripper_pos)
            # Nudge in whichever direction stays within [0, 100].
            if current >= nudge_pct:
                nudge_target = int(round(current - nudge_pct))
            else:
                nudge_target = int(round(current + nudge_pct))
            return_target = int(round(current))

            self._keepalive_active.set()
            try:
                logger.debug(
                    "[FairinoLeader] Keepalive nudge %d%% -> %d%%",
                    return_target,
                    nudge_target,
                )
                self._issue_move_gripper(nudge_target)
                if self._keepalive_stop.wait(settle_s):
                    break
                self._issue_move_gripper(return_target)
                if self._keepalive_stop.wait(settle_s):
                    break
            except Exception as exc:
                logger.warning(
                    "[FairinoLeader] Keepalive err: %s",
                    exc,
                )
            finally:
                self._keepalive_active.clear()
                # Reset idle clock so the next nudge waits a full
                # interval; also rebaseline activity tracking so
                # our own movement isn't re-detected next loop.
                self._last_activity_time = time.monotonic()
                self._last_gripper_pos_for_activity = float(self._gripper_pos)

    def _issue_move_gripper(self, pos_int: int) -> None:
        """Issue a non-blocking MoveGripper via the keepalive proxy.

        Uses the separate ServerProxy to avoid contention with the
        main thread's ``self._rpc``.  Parameters mirror the initial
        compliance command issued in ``_initialise_gripper()``.
        """
        cfg = self.config
        proxy = self._keepalive_rpc
        if proxy is None:
            return
        proxy.MoveGripper(
            int(cfg.gripper_index),
            int(pos_int),
            int(cfg.gripper_vel),
            int(cfg.gripper_force),
            _GRIPPER_MAXTIME_MS,
            0,
            0,
            0.0,
            0,
            0,
        )
