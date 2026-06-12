"""Set the FR5 per-joint collision grade to the least sensitive value via one SDK call.

Fairino collision grade is per-joint 1-10 (1 = most sensitive, 10 = least
sensitive). This script makes exactly one documented XMLRPC call --
SetAnticollision(0, [level]*6, 1) -- and nothing else: probing the controller
with system.listMethods or bare TCP connects crashed the control service
(ports 20003/20004 went down), so all introspection was removed. There is no
SDK getter for the current grade; read it from the WebApp safety page instead.
One-off diagnostic for gh issue #78.
"""

import argparse
import socket
import xmlrpc.client

XMLRPC_PORT = 20003
JOINT_COUNT = 6
MODE_GRADE = 0  # SetAnticollision mode: 0 = grade 1-10, 1 = percentage
PERSIST_CONFIG = 1  # 1 = write the value into the controller config file
CONNECT_TIMEOUT_S = 5  # fail fast instead of hanging when the service is down


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="192.168.58.2", help="Robot controller IP")
    parser.add_argument(
        "--level",
        type=float,
        default=10.0,
        help="Collision grade for all joints, 1 (most sensitive) - 10 (least)",
    )
    args = parser.parse_args()

    socket.setdefaulttimeout(CONNECT_TIMEOUT_S)
    proxy = xmlrpc.client.ServerProxy(f"http://{args.ip}:{XMLRPC_PORT}")

    level = [float(args.level)] * JOINT_COUNT
    errcode = proxy.SetAnticollision(MODE_GRADE, level, PERSIST_CONFIG)
    print(f"SetAnticollision(mode={MODE_GRADE}, level={level}, config={PERSIST_CONFIG}) -> {errcode}")
    if errcode == 0:
        print(f"OK: all joints now at collision grade {args.level} (least sensitive = 10)")
    else:
        print(f"FAILED with controller errcode {errcode}")


if __name__ == "__main__":
    main()
