"""Exercise FR5 limit-fault detection and recovery without hardware.

Issue #93: a joint hitting its motor limit faulted the controller and
crashed `lerobot-teleoperate` (surfacing as a Rerun gRPC transport
error).  Layer B adds fault detection + auto-recovery and replaces the
RuntimeError-raising joint reads with a last-known-joints fallback.

This script injects fake RPC objects so the new logic can be checked
off-robot.  Run with the lerobot env python:

    /home/inno-controller/anaconda3/envs/lerobot/bin/python \
        claude_test/debug_limit_fault_recovery.py

Expected lifetime: keep while issue #93 is open; promote the asserts
into tests/ if they prove stable.
"""

# The fake RPC classes deliberately mirror the Fairino SDK's
# CamelCase method names so the production code calls resolve.
# ruff: noqa: N802

import sys

import lerobot.robots.fairino_follower.fairino_follower as follower_mod
import lerobot.teleoperators.fairino_leader.fairino_leader as leader_mod
from lerobot.robots.fairino_follower.config_fairino_follower import FairinoFollowerConfig
from lerobot.robots.fairino_follower.fairino_follower import FairinoFollower
from lerobot.teleoperators.fairino_leader.config_fairino_leader import FairinoLeaderConfig
from lerobot.teleoperators.fairino_leader.fairino_leader import FairinoLeader

# Recovery runs the full enable sequence with multi-second settles.
# Neutralise the sleeps so the off-robot checks finish instantly; the
# rate-limit logic uses time.monotonic(), which is left untouched.
follower_mod.time.sleep = lambda *_a, **_k: None
leader_mod.time.sleep = lambda *_a, **_k: None

_results = []


def check(name, ok):
    _results.append((name, ok))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")


class FakePkg:
    """Stand-in for the SDK real-time state package."""

    def __init__(self, main_code=0, s0=0, s1=0):
        self.main_code = main_code
        self.sub_code = 0
        self.safety_stop0_state = s0
        self.safety_stop1_state = s1


class FakeRpc:
    """Records control calls and returns a scripted joint read."""

    def __init__(self, joints):
        self.robot_state_pkg = FakePkg()
        self._joints = list(joints)
        self.read_ret = 0
        self.calls = []

    def GetActualJointPosDegree(self):
        return (self.read_ret, list(self._joints))

    def _record(self, name):
        self.calls.append(name)
        return 0

    def ServoMoveEnd(self):
        return self._record("ServoMoveEnd")

    def ServoMoveStart(self):
        return self._record("ServoMoveStart")

    def StopMotion(self):
        return self._record("StopMotion")

    def ResetAllError(self):
        return self._record("ResetAllError")

    def RobotEnable(self, state):
        return self._record(f"RobotEnable({state})")

    def Mode(self, state):
        return self._record(f"Mode({state})")

    def DragTeachSwitch(self, state):
        return self._record(f"DragTeachSwitch({state})")


class FakeServoProxy:
    """Counts ServoJ commands actually issued to the controller."""

    def __init__(self):
        self.servoj_calls = 0

    def ServoJ(self, *_a):
        self.servoj_calls += 1
        return 0

    def ServoMoveEnd(self):
        return 0

    def ServoMoveStart(self):
        return 0


def make_follower():
    f = FairinoFollower(FairinoFollowerConfig())
    f._is_connected = True
    f._use_xmlrpc_reads = False
    f._rpc = FakeRpc([10, 20, 30, 40, 50, 60])
    f._servo_proxy = FakeServoProxy()
    f._commanded = [10, 20, 30, 40, 50, 60]
    return f


def make_leader():
    cfg = FairinoLeaderConfig()
    cfg.gripper_enabled = False
    cfg.gripper_keepalive_enabled = False
    ldr = FairinoLeader(cfg)
    ldr._is_connected = True
    ldr._use_xmlrpc_reads = False
    ldr._rpc = FakeRpc([1, 2, 3, 4, 5, 6])
    return ldr


def action6(vals):
    return {f"joint{i + 1}.pos": vals[i] for i in range(6)}


# ---- 1. _detect_fault truth table -------------------------------
f = make_follower()
f._rpc.robot_state_pkg = FakePkg(main_code=0)
check("follower: no fault when main_code=0", f._detect_fault() is False)
f._rpc.robot_state_pkg = FakePkg(main_code=7)
check("follower: fault when main_code!=0", f._detect_fault() is True)
f._rpc.robot_state_pkg = FakePkg(s0=1)
check("follower: fault when safety_stop0 set", f._detect_fault() is True)
f._rpc.robot_state_pkg = FakePkg(s1=1)
check("follower: fault when safety_stop1 set", f._detect_fault() is True)
f._use_xmlrpc_reads = True
f._rpc.robot_state_pkg = FakePkg(main_code=7)
check("follower: xmlrpc-only reports no fault", f._detect_fault() is False)

# ---- 2. get_observation serves last-known on read failure -------
f = make_follower()
obs = f.get_observation()
check("follower: first observation reads live joints", obs["joint1.pos"] == 10)
f._rpc.read_ret = -1  # next read fails
try:
    obs2 = f.get_observation()
    served_stale = obs2["joint1.pos"] == 10
    raised = False
except RuntimeError:
    served_stale = False
    raised = True
check("follower: read failure does NOT raise", raised is False)
check("follower: read failure serves last-known joints", served_stale)

# ---- 3. send_action recovers on fault and rate-limits -----------
f = make_follower()
f._rpc.robot_state_pkg = FakePkg(main_code=7)  # active fault
f.send_action(action6([11, 21, 31, 41, 51, 61]))
first_resets = f._rpc.calls.count("ResetAllError")
first_servoj = f._servo_proxy.servoj_calls
check("follower: fault triggers recovery (ResetAllError)", first_resets == 1)
check("follower: ServoJ skipped during recovery tick", first_servoj == 0)
# Second immediate faulted tick is within the rate-limit window.
f.send_action(action6([12, 22, 32, 42, 52, 62]))
check(
    "follower: recovery rate-limited (no 2nd ResetAllError)",
    f._rpc.calls.count("ResetAllError") == 1,
)

# ---- 4. leader get_action fallback + drag-teach recovery --------
ldr = make_leader()
act = ldr.get_action()
check("leader: first action reads live joints", act["joint1.pos"] == 1)
ldr._rpc.read_ret = -1
try:
    act2 = ldr.get_action()
    leader_stale = act2["joint1.pos"] == 1
    leader_raised = False
except RuntimeError:
    leader_stale = False
    leader_raised = True
check("leader: read failure does NOT raise", leader_raised is False)
check("leader: read failure serves last-known joints", leader_stale)
check(
    "leader: read failure re-enters drag-teach",
    ldr._rpc.calls.count("DragTeachSwitch(1)") >= 1,
)
# Immediate second failure stays within the rate-limit window.
before = ldr._rpc.calls.count("DragTeachSwitch(1)")
ldr.get_action()
check(
    "leader: drag-teach recovery rate-limited",
    ldr._rpc.calls.count("DragTeachSwitch(1)") == before,
)

# ---- summary ----------------------------------------------------
passed = sum(1 for _n, ok in _results if ok)
total = len(_results)
print(f"\n{passed}/{total} checks passed")
sys.exit(0 if passed == total else 1)
