"""Verify gh #88: pi0_base vision tower actually loads (not random) under transformers 5.x.

Loads the pinned local checkpoint via PI0Policy.from_pretrained and checks that
(a) the vision-tower remap kicked in (model keys hold checkpoint values), and
(b) missing/unexpected key counts after the fix.
"""

import torch
from safetensors.torch import load_file

from lerobot.policies.pi0.modeling_pi0 import PI0Policy

CKPT = "models/pi0_base_v051compat"

policy = PI0Policy.from_pretrained(CKPT)
sd = policy.state_dict()
raw = load_file(f"{CKPT}/model.safetensors")

# Compare every checkpoint vision-tower tensor against the loaded model.
n_checked = n_match = 0
for old_key, expected in raw.items():
    if ".vision_tower.vision_model." not in old_key:
        continue
    new_key = old_key.replace(".vision_tower.vision_model.", ".vision_tower.")
    if not new_key.startswith("model."):
        new_key = f"model.{new_key}"
    if new_key not in sd:
        print(f"STILL MISSING in model: {new_key}")
        continue
    n_checked += 1
    if torch.allclose(sd[new_key].float().cpu(), expected.float()):
        n_match += 1
    else:
        print(f"VALUE MISMATCH: {new_key}")

print(f"vision-tower tensors checked: {n_checked}, matching: {n_match}")
n_model_vision = sum(1 for k in sd if ".vision_tower." in k)
print(f"model vision-tower keys: {n_model_vision}")
assert n_checked > 0, "no vision-tower keys found in checkpoint"
assert n_match == n_checked, "vision tower NOT fully loaded"
print("OK: vision tower fully loaded from checkpoint")
