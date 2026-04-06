#!/usr/bin/env python
"""Read PiPER arm joint values via the LeRobot framework.

Connects to the PiPER arm in slave mode and prints joint
positions at a configurable rate. This validates that the
PiPER leader teleoperator correctly reads joint angles.

Usage::

    python claude_test/test_piper_joint_read.py
    python claude_test/test_piper_joint_read.py --can can0
    python claude_test/test_piper_joint_read.py --can can0 --hz 10 --duration 5
"""

import argparse
import sys
import time

JOINT_NAMES = [
    "joint1", "joint2", "joint3",
    "joint4", "joint5", "joint6",
]
MOTOR_NAMES = JOINT_NAMES + ["gripper"]

# ANSI helpers
_CLR = "\033[2K"
_UP = "\033[A"
_BOLD = "\033[1m"
_RST = "\033[0m"
_GRN = "\033[92m"
_CYN = "\033[96m"

_DISPLAY_LINES = len(MOTOR_NAMES) + 3


def _format_joints(
    values: dict[str, float],
    hz_actual: float,
) -> str:
    """Build a multi-line joint state display."""
    lines = [
        f"  {_CYN}Hz: {hz_actual:.1f}{_RST}",
        "",
    ]
    for motor in MOTOR_NAMES:
        key = f"{motor}.pos"
        val = values.get(key, 0.0)
        lines.append(f"  {motor:<8}: {val:>10.3f}")
    return "\n".join(lines)


def main():
    """Connect to PiPER and read joint positions."""
    ap = argparse.ArgumentParser(
        description="Read PiPER joint values via LeRobot",
    )
    ap.add_argument(
        "--can", default="can0",
        help="CAN interface name (default: can0)",
    )
    ap.add_argument(
        "--hz", type=float, default=10.0,
        help="Read rate in Hz (default: 10.0)",
    )
    ap.add_argument(
        "--duration", type=float, default=0.0,
        help="Duration in seconds (0 = infinite, default: 0)",
    )
    args = ap.parse_args()

    from lerobot.teleoperators.piper_leader.config_piper_leader import (
        PiperLeaderConfig,
    )
    from lerobot.teleoperators.piper_leader.piper_leader import (
        PiperLeader,
    )

    cfg = PiperLeaderConfig(id="piper_test", port=args.can)

    print(
        f"{_BOLD}Connecting to PiPER on '{args.can}' "
        f"...{_RST}"
    )
    try:
        leader = PiperLeader(cfg)
        leader.connect()
    except Exception as exc:
        print(f"Connection failed: {exc}")
        sys.exit(1)

    print(f"{_GRN}Connected (slave mode).{_RST}")
    print(
        f"\n{_BOLD}Reading joint positions at "
        f"{args.hz:.0f} Hz.  Ctrl+C to stop.{_RST}\n"
    )

    # Initial read + display
    action = leader.get_action()
    print(_format_joints(action, 0.0))

    period = 1.0 / args.hz
    t_start_global = time.perf_counter()
    read_count = 0

    try:
        while True:
            t_start = time.perf_counter()

            action = leader.get_action()
            read_count += 1

            dt = time.perf_counter() - t_start
            hz_actual = 1.0 / dt if dt > 0 else 0.0

            for _ in range(_DISPLAY_LINES):
                sys.stdout.write(_UP + _CLR)
            print(_format_joints(action, hz_actual))

            # Check duration limit.
            if args.duration > 0:
                elapsed = (
                    time.perf_counter() - t_start_global
                )
                if elapsed >= args.duration:
                    break

            remaining = period - dt
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        pass
    finally:
        leader.disconnect()
        elapsed = time.perf_counter() - t_start_global
        print(
            f"\n{_GRN}Done. "
            f"{read_count} reads in {elapsed:.1f}s "
            f"({read_count / elapsed:.1f} Hz avg).{_RST}"
        )


if __name__ == "__main__":
    main()
