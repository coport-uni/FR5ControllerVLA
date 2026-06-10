# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Keyboard shortcut listener that reads Linux evdev devices.

pynput's X11 backend cannot capture global key presses under Wayland
and fails to import entirely when no display is available (e.g. over
SSH). This module reads key events straight from ``/dev/input/event*``
instead, which works regardless of display server and window focus.
Reading those device nodes requires membership in the ``input`` group
(or an equivalent ACL on the device files).
"""

import contextlib
import logging
import select
import threading
from collections.abc import Callable

import evdev
from evdev import ecodes

# Seconds select() waits per loop; bounds the stop() latency.
_select_timeout_s = 0.2

# evdev reports value 1 for a key-press transition (0 is release,
# 2 is auto-repeat).
_key_down_value = 1


class EvdevKeyboardListener:
    """Reads key-down events from all keyboard-like evdev devices.

    The listener scans ``/dev/input/event*`` for devices that expose
    the arrow and escape keys, then reads them on a daemon thread and
    forwards every key-down event to the callback.

    Args:
        on_press: Callback invoked with the evdev key code (an
            ``ecodes.KEY_*`` integer) for every key-down event.
    """

    def __init__(self, on_press: Callable[[int], None]):
        self._on_press = on_press
        self._devices: list[evdev.InputDevice] = []
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @staticmethod
    def find_keyboard_devices() -> list[evdev.InputDevice]:
        """Returns readable evdev devices exposing arrow and Esc keys.

        Devices the current user cannot open (typically because the
        user is not in the ``input`` group) are silently skipped.
        """
        required_keys = {
            ecodes.KEY_LEFT,
            ecodes.KEY_RIGHT,
            ecodes.KEY_ESC,
        }
        devices = []
        for path in evdev.list_devices():
            try:
                device = evdev.InputDevice(path)
            except OSError:
                continue
            key_capabilities = device.capabilities().get(ecodes.EV_KEY, [])
            if required_keys.issubset(key_capabilities):
                devices.append(device)
            else:
                device.close()
        return devices

    def start(self) -> None:
        """Opens keyboard devices and starts the reader thread.

        Raises:
            RuntimeError: If no readable keyboard device is found,
                e.g. when the user is not in the ``input`` group.
        """
        self._devices = self.find_keyboard_devices()
        if not self._devices:
            raise RuntimeError(
                "no readable keyboard device under /dev/input (is the user in the 'input' group?)"
            )
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._read_events,
            name="evdev_keyboard_listener",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """Stops the reader thread and closes all devices."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        for device in self._devices:
            with contextlib.suppress(OSError):
                device.close()
        self._devices = []

    def _read_events(self) -> None:
        """Forwards key-down events until stop() or device loss."""
        while not self._stop_event.is_set():
            readable, _, _ = select.select(self._devices, [], [], _select_timeout_s)
            for device in readable:
                try:
                    events = list(device.read())
                except OSError:
                    # Device unplugged; drop it and keep the rest.
                    self._devices.remove(device)
                    continue
                for event in events:
                    if event.type == ecodes.EV_KEY and event.value == _key_down_value:
                        self._on_press(event.code)
            if not self._devices:
                logging.warning("All keyboard devices disappeared; stopping evdev listener.")
                return
