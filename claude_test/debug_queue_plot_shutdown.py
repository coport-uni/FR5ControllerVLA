"""Reproduce the --debug_visualize_queue_size no-show without the robot.

Mimics async_client()'s exact shutdown path (robot_client.py:505-512):
a daemon receiver thread, a blocking "control loop", and the same
finally-block that calls visualize_action_queue_size only after the
interrupt. The script sends SIGINT to itself after 3 s, so running it
is equivalent to pressing Ctrl+C once and then waiting.

Expected on this box (TkAgg + DISPLAY): "calling visualize" prints,
then a Tk window with the queue plot opens and the process blocks in
plt.show() until the window is closed. If the window never appears,
the environment (backend/DISPLAY) is at fault; if it appears, a field
no-show means the finally block was killed early (second Ctrl+C, or
an exception in RobotClient.stop() before the visualize call).

Usage:
    /home/inno-controller/anaconda3/envs/lerobot/bin/python \
        claude_test/debug_queue_plot_shutdown.py
"""

import os
import signal
import threading
import time

from lerobot.async_inference.helpers import visualize_action_queue_size


def receiver():
    while True:
        time.sleep(1)


def send_sigint_later():
    time.sleep(3)
    print("sending SIGINT to self (simulated single Ctrl+C)", flush=True)
    os.kill(os.getpid(), signal.SIGINT)


threading.Thread(target=receiver, daemon=True).start()
threading.Thread(target=send_sigint_later, daemon=True).start()

try:
    print("control loop running (SIGINT arrives in 3 s)", flush=True)
    time.sleep(60)
finally:
    time.sleep(1)  # stand-in for client.stop() + receiver join
    print("calling visualize", flush=True)
    import matplotlib

    print("backend:", matplotlib.get_backend(), flush=True)
    visualize_action_queue_size([25, 20, 15, 22, 18, 12, 25])
    print("Client stopped (visualize returned)", flush=True)
