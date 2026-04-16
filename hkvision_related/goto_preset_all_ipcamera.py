"""Move every configured HikVision camera to a named PTZ preset.

Thin runner on top of ``onvif_controller.ONVIFController``. Camera
credentials are hardcoded below to match the existing style of this
repository; each camera is handled independently so a single failed
connection does not block the others.
"""

import logging

from onvif_controller import ONVIFController

camera_configs = [
    {
        "ip": "192.168.1.218",
        "port": 218,
        "user": "admin",
        "password": "peal2024",
    },
    {
        "ip": "192.168.1.219",
        "port": 219,
        "user": "admin",
        "password": "peal2024",
    },
    {
        "ip": "192.168.1.220",
        "port": 220,
        "user": "admin",
        "password": "peal2024",
    },
]


def goto_preset_all(preset_name: str) -> None:
    """Move every configured camera to the named preset, in order.

    Exceptions from any single camera are logged and swallowed so the
    remaining cameras still run. This matches the physical intent of
    "send all cameras to preset X" even when one device is offline.

    Args:
        preset_name: Target preset name, e.g. ``"Preset 1"``. Must
            already exist on each camera; unknown names are logged as
            a warning by :class:`ONVIFController` and have no effect.
    """
    for cfg in camera_configs:
        ip = cfg["ip"]
        try:
            cam = ONVIFController(
                ip,
                cfg["port"],
                cfg["user"],
                cfg["password"],
            )
            cam.camera_start()
            cam.go_to_preset(preset_name)
        except Exception as exc:
            logging.error("Camera %s failed: %s", ip, exc)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    goto_preset_all("Preset 1")
