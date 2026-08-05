"""Check that the meta-device Pi0 load produces the exact same model as the old eager load (gh #123).

Intent: one-off verification for the load-time fix. Loads one checkpoint twice -- once the way
from_pretrained used to work (full random init on CPU, then copy the weights in) and once the way
it works now (meta skeleton, assign=True) -- and compares every parameter and buffer bit for bit,
including dtype and device. Also prints the wall-clock cost of each path. Expected lifetime: until
the fix is verified on hardware.

    python claude_test/debug_pi0_meta_load.py [--repo REPO_ID] [--device cuda]
"""

import argparse
import time
from itertools import chain

import torch
from safetensors.torch import load_file
from transformers.utils import cached_file

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.pi0.modeling_pi0 import PI0Policy
from lerobot.policies.pi05.modeling_pi05 import PI05Policy

DEFAULT_REPO = (
    "coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_50_pi0_b200_model"
)
POLICY_CLASSES = {"pi0": PI0Policy, "pi05": PI05Policy}


def load_the_old_way(policy_class, config, repo):
    """Reproduce from_pretrained as it was before the meta-init change."""
    model = policy_class(config)  # full float32 random init on CPU, then .to(device)
    state_dict = load_file(cached_file(repo, "model.safetensors"))  # host memory
    fixed = model._fix_pytorch_state_dict_keys(state_dict, model.config)
    remapped = {k if k.startswith("model.") else f"model.{k}": v for k, v in fixed.items()}
    missing, unexpected = model.load_state_dict(remapped, strict=True)
    assert not missing and not unexpected, (missing, unexpected)
    return model


def compare(reference, candidate):
    """Return a list of human-readable differences between two models."""
    differences = []
    ref_tensors = dict(chain(reference.named_parameters(), reference.named_buffers()))
    new_tensors = dict(chain(candidate.named_parameters(), candidate.named_buffers()))

    only_ref = sorted(set(ref_tensors) - set(new_tensors))
    only_new = sorted(set(new_tensors) - set(ref_tensors))
    differences += [f"missing from the new path: {k}" for k in only_ref]
    differences += [f"unexpected in the new path: {k}" for k in only_new]

    for name in sorted(set(ref_tensors) & set(new_tensors)):
        ref, new = ref_tensors[name], new_tensors[name]
        if ref.dtype != new.dtype:
            differences.append(f"{name}: dtype {ref.dtype} -> {new.dtype}")
        elif ref.device != new.device:
            differences.append(f"{name}: device {ref.device} -> {new.device}")
        elif not torch.equal(ref, new):
            differences.append(f"{name}: values differ")
    return differences


def run_one_chunk(model, config, repo, task):
    """Push one synthetic observation through the full inference path."""
    from lerobot.policies.factory import make_pre_post_processors

    preprocessor, postprocessor = make_pre_post_processors(
        config,
        pretrained_path=repo,
        preprocessor_overrides={"device_processor": {"device": config.device}},
        postprocessor_overrides={"device_processor": {"device": config.device}},
    )
    observation = {"task": task, "observation.state": torch.zeros(7)}
    for name, feature in config.input_features.items():
        if "image" in name:
            observation[name] = torch.rand(feature.shape)

    start = time.perf_counter()
    with torch.inference_mode():
        chunk = model.predict_action_chunk(preprocessor(observation))
    seconds = time.perf_counter() - start

    finite = bool(torch.isfinite(chunk).all())
    print(f"\nforward pass: {tuple(chunk.shape)} in {seconds:.2f} s, all finite: {finite}")
    postprocessor(chunk[:, 0])
    return finite


def benchmark(model, config, repo, task, repeats):
    """Return the median action-chunk latency in seconds."""
    from lerobot.policies.factory import make_pre_post_processors

    preprocessor, _ = make_pre_post_processors(
        config,
        pretrained_path=repo,
        preprocessor_overrides={"device_processor": {"device": config.device}},
        postprocessor_overrides={"device_processor": {"device": config.device}},
    )
    observation = {"task": task, "observation.state": torch.zeros(7)}
    for name, feature in config.input_features.items():
        if "image" in name:
            observation[name] = torch.rand(feature.shape)
    batch = preprocessor(observation)

    latencies = []
    with torch.inference_mode():
        for index in range(repeats + 3):  # first calls warm the kernels up
            torch.cuda.synchronize()
            start = time.perf_counter()
            model.predict_action_chunk(batch)
            torch.cuda.synchronize()
            if index >= 3:
                latencies.append(time.perf_counter() - start)
    latencies.sort()
    return latencies[len(latencies) // 2], latencies[0], latencies[-1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default=DEFAULT_REPO)
    parser.add_argument("--policy", default="pi0", choices=sorted(POLICY_CLASSES))
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--dtype", default=None, help="override policy.dtype, as the train scripts do")
    parser.add_argument("--smoke", action="store_true", help="also run one synthetic action chunk")
    parser.add_argument("--skip-old", action="store_true", help="skip the slow pre-fix load path")
    parser.add_argument("--bench", type=int, default=0, help="A/B the inference latency of both paths")
    args = parser.parse_args()

    config = PreTrainedConfig.from_pretrained(args.repo)
    config.compile_model = False  # compilation is orthogonal and slow (LP Q14)
    config.device = args.device
    if args.dtype is not None:
        config.dtype = args.dtype

    policy_class = POLICY_CLASSES[args.policy]

    start = time.perf_counter()
    new_model = policy_class.from_pretrained(args.repo, config=config)
    new_seconds = time.perf_counter() - start
    print(f"\nnew meta path:  {new_seconds:6.1f} s")

    if args.smoke:
        run_one_chunk(
            new_model,
            config,
            args.repo,
            "move the brown colored glass bottle to the designated location",
        )

    if args.skip_old:
        return

    start = time.perf_counter()
    old_model = load_the_old_way(policy_class, config, args.repo)
    old_seconds = time.perf_counter() - start

    print(f"\nold eager path: {old_seconds:6.1f} s")
    print(f"new meta path:  {new_seconds:6.1f} s")
    print(f"speedup:        {old_seconds / new_seconds:6.1f}x")

    if args.bench:
        task = "move the brown colored glass bottle to the designated location"
        new_median, new_min, new_max = benchmark(new_model, config, args.repo, task, args.bench)
        old_median, old_min, old_max = benchmark(old_model, config, args.repo, task, args.bench)
        print(f"\ninference latency over {args.bench} calls (median [min-max]):")
        print(f"  old eager path: {old_median * 1000:6.1f} ms [{old_min * 1000:.1f}-{old_max * 1000:.1f}]")
        print(f"  new meta path:  {new_median * 1000:6.1f} ms [{new_min * 1000:.1f}-{new_max * 1000:.1f}]")

    differences = compare(old_model, new_model)
    n_tensors = len(dict(chain(old_model.named_parameters(), old_model.named_buffers())))
    if differences:
        print(f"\nFAIL: {len(differences)} of {n_tensors} tensors differ")
        for line in differences[:20]:
            print(f"  - {line}")
    else:
        print(f"\nPASS: all {n_tensors} parameters and buffers match bit for bit")


if __name__ == "__main__":
    main()
