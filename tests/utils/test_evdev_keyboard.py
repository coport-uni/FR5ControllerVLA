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

"""Unit tests for the evdev-based record shortcut listener.

The real `/dev/input/event*` devices need `input`-group access and a
physical keyboard, so these tests drive the listener thread through a
pipe-backed fake device instead.
"""

import os
import threading
import time

import pytest

pytest.importorskip("evdev")

from evdev import ecodes  # noqa: E402

from lerobot.utils.evdev_keyboard import EvdevKeyboardListener  # noqa: E402

# evdev key event values: 1 is key-down, 0 is key-up.
_key_down_value = 1
_key_up_value = 0

# Upper bound for the reader thread to dispatch an emitted event.
_dispatch_timeout_s = 2.0

# Polling period while waiting for the dispatch.
_poll_period_s = 0.01


class FakeKeyEvent:
    """Minimal stand-in for an `evdev.InputEvent`."""

    def __init__(self, code, value, event_type=ecodes.EV_KEY):
        self.type = event_type
        self.code = code
        self.value = value


class FakeKeyboardDevice:
    """Pipe-backed stand-in for `evdev.InputDevice`.

    `select()` inside the listener watches the pipe's read end; one
    byte is written per emitted event to wake the reader thread.
    """

    def __init__(self):
        self._read_fd, self._write_fd = os.pipe()
        self._pending = []
        self._lock = threading.Lock()

    def fileno(self):
        return self._read_fd

    def emit(self, event):
        """Queues one event and wakes the listener's select() call."""
        with self._lock:
            self._pending.append(event)
        os.write(self._write_fd, b"x")

    def read(self):
        os.read(self._read_fd, 1)
        with self._lock:
            events = self._pending
            self._pending = []
        return events

    def close(self):
        os.close(self._read_fd)
        os.close(self._write_fd)


def _wait_for_dispatch(pressed):
    """Blocks until the listener delivers a key or the timeout hits."""
    deadline = time.monotonic() + _dispatch_timeout_s
    while not pressed and time.monotonic() < deadline:
        time.sleep(_poll_period_s)


def test_listener_dispatches_key_down_only(monkeypatch):
    """Key-down events reach the callback; releases are filtered."""
    device = FakeKeyboardDevice()
    monkeypatch.setattr(
        EvdevKeyboardListener,
        "find_keyboard_devices",
        staticmethod(lambda: [device]),
    )

    pressed = []
    listener = EvdevKeyboardListener(pressed.append)
    listener.start()
    try:
        device.emit(FakeKeyEvent(ecodes.KEY_LEFT, _key_up_value))
        device.emit(FakeKeyEvent(ecodes.KEY_RIGHT, _key_down_value))
        _wait_for_dispatch(pressed)
    finally:
        listener.stop()

    assert pressed == [ecodes.KEY_RIGHT]


def test_start_raises_without_readable_devices(monkeypatch):
    """start() must fail loudly when no keyboard device is readable."""
    monkeypatch.setattr(
        EvdevKeyboardListener,
        "find_keyboard_devices",
        staticmethod(list),
    )

    listener = EvdevKeyboardListener(lambda key_code: None)
    with pytest.raises(RuntimeError, match="input"):
        listener.start()


def test_stop_is_idempotent_and_closes_devices(monkeypatch):
    """stop() joins the thread and survives being called twice."""
    device = FakeKeyboardDevice()
    monkeypatch.setattr(
        EvdevKeyboardListener,
        "find_keyboard_devices",
        staticmethod(lambda: [device]),
    )

    listener = EvdevKeyboardListener(lambda key_code: None)
    listener.start()
    listener.stop()
    listener.stop()

    with pytest.raises(OSError):
        os.fstat(device.fileno())
