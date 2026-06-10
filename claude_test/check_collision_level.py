"""Probe the FR5's collision-level setting and set it to the least sensitive grade (10).

Fairino collision grade is per-joint 1-10 (1 = most sensitive, 10 = least
sensitive). The Python SDK has no getter, so this script first introspects the
controller's XMLRPC surface for any Get-style collision method, reports what it
finds, then calls SetAnticollision(0, [level]*6, 1) to apply and persist the
requested grade. One-off diagnostic for gh issue #78.
"""

import argparse
import xmlrpc.client

XMLRPC_PORT = 20003
JOINT_COUNT = 6
MODE_GRADE = 0  # SetAnticollision mode: 0 = grade 1-10, 1 = percentage
PERSIST_CONFIG = 1  # 1 = write the value into the controller config file


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="192.168.58.2", help="Robot controller IP")
    parser.add_argument(
        "--level",
        type=float,
        default=10.0,
        help="Collision grade for all joints, 1 (most sensitive) - 10 (least)",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Only probe the current setting; do not change anything",
    )
    args = parser.parse_args()

    proxy = xmlrpc.client.ServerProxy(f"http://{args.ip}:{XMLRPC_PORT}")

    # 1. Discover any collision-related methods the controller exposes.
    collision_methods = []
    try:
        methods = proxy.system.listMethods()
        collision_methods = [m for m in methods if "ollision" in m or "nticollision" in m]
        print(f"[introspection] collision-related XMLRPC methods: {collision_methods}")
    except Exception as exc:
        print(f"[introspection] system.listMethods not supported: {exc}")

    # 2. Try every Get-style candidate so we can record the current setting.
    for name in collision_methods:
        if not name.startswith("Get"):
            continue
        try:
            result = getattr(proxy, name)()
            print(f"[current] {name}() -> {result}")
        except Exception as exc:
            print(f"[current] {name}() failed: {exc}")

    if args.check_only:
        return

    # 3. Apply the requested grade to all six joints and persist it.
    level = [float(args.level)] * JOINT_COUNT
    errcode = proxy.SetAnticollision(MODE_GRADE, level, PERSIST_CONFIG)
    print(f"[set] SetAnticollision(mode={MODE_GRADE}, level={level}, config={PERSIST_CONFIG}) -> {errcode}")
    if errcode == 0:
        print(f"[set] OK: all joints now at collision grade {args.level} (least sensitive = 10)")
    else:
        print(f"[set] FAILED with controller errcode {errcode}")

    # 4. Re-read if a getter exists, to confirm the new value.
    for name in collision_methods:
        if name.startswith("Get"):
            try:
                print(f"[verify] {name}() -> {getattr(proxy, name)()}")
            except Exception as exc:
                print(f"[verify] {name}() failed: {exc}")


if __name__ == "__main__":
    main()
