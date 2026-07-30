"""One-off: trigger real ACT inference on the remote policy server with
synthetic observations (no robot, FR5 never moves) to verify that the
new [server] log stream in 9__run_client_act.sh shows the server-side
inference timings locally, and to measure actual inference latency.

Expected lifetime: this debug session only (gh #113).
"""

import argparse
import pickle
import time

import grpc
import numpy as np

from lerobot.async_inference.helpers import RemotePolicyConfig, TimedObservation
from lerobot.datasets.feature_utils import hw_to_dataset_features
from lerobot.transport import services_pb2, services_pb2_grpc
from lerobot.transport.utils import grpc_channel_options, send_bytes_in_chunks

SERVER_ADDRESS = "127.0.0.1:17044"
PRETRAINED = (
    "coport-uni/FR5_task2_transfer_the_gray_tablets_from_the_brown_bottle"
    "_into_another_brown_bottle_50_act_h200"
)
TASK = "transfer the gray tablets from the brown bottle into another brown bottle"
ACTIONS_PER_CHUNK = 100
NUM_OBSERVATIONS = 5

# Mirrors FairinoFollower.observation_features with gripper_enabled=true
# and the three cameras from 9__run_client_act.sh (all 640x480 RGB).
hw_features = {f"joint{i}.pos": float for i in range(1, 7)}
hw_features["gripper.pos"] = float
for cam in ("top_left", "top_right", "hand"):
    hw_features[cam] = (480, 640, 3)
lerobot_features = hw_to_dataset_features(hw_features, "observation", use_video=False)


def make_raw_observation(rng, step):
    obs = {f"joint{i}.pos": float(rng.uniform(-30, 30)) for i in range(1, 7)}
    obs["gripper.pos"] = float(rng.uniform(0, 100))
    for cam in ("top_left", "top_right", "hand"):
        obs[cam] = rng.integers(0, 255, size=(480, 640, 3), dtype=np.uint8)
    obs["task"] = TASK
    return obs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", default=SERVER_ADDRESS)
    args = parser.parse_args()

    channel = grpc.insecure_channel(args.server, grpc_channel_options())
    stub = services_pb2_grpc.AsyncInferenceStub(channel)

    stub.Ready(services_pb2.Empty())
    print(f"[fake-client] handshake OK with {args.server}")

    policy_config = RemotePolicyConfig(
        policy_type="act",
        pretrained_name_or_path=PRETRAINED,
        lerobot_features=lerobot_features,
        actions_per_chunk=ACTIONS_PER_CHUNK,
        device="cuda",
    )
    t0 = time.perf_counter()
    stub.SendPolicyInstructions(services_pb2.PolicySetup(data=pickle.dumps(policy_config)))
    print(f"[fake-client] policy loaded on server in {time.perf_counter() - t0:.1f}s")

    rng = np.random.default_rng(0)
    round_trips_ms = []
    for step in range(NUM_OBSERVATIONS):
        obs = TimedObservation(
            timestamp=time.time(),
            timestep=step,
            observation=make_raw_observation(rng, step),
            must_go=True,
        )
        obs_bytes = pickle.dumps(obs)
        stub.SendObservations(send_bytes_in_chunks(obs_bytes, services_pb2.Observation, silent=True))
        t0 = time.perf_counter()
        actions = stub.GetActions(services_pb2.Empty())
        rt_ms = (time.perf_counter() - t0) * 1000
        n_actions = len(pickle.loads(actions.data)) if len(actions.data) else 0
        round_trips_ms.append(rt_ms)
        print(
            f"[fake-client] obs #{step}: GetActions round-trip {rt_ms:.1f} ms, {n_actions} actions received"
        )

    print(
        f"[fake-client] round-trip over {len(round_trips_ms)} chunks: "
        f"min {min(round_trips_ms):.1f} / median {sorted(round_trips_ms)[len(round_trips_ms) // 2]:.1f} "
        f"/ max {max(round_trips_ms):.1f} ms"
    )


if __name__ == "__main__":
    main()
