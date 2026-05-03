# Pi0.5 VRAM Probe Summary (issue #58)

Goal: find the largest per-GPU `--batch_size` for `7__train_pi05_adv.sh` whose peak VRAM stays at ~80% (~115 GB) of an H200 NVL (143771 MiB / 143.77 GB).

Probe configuration:
- 2× H200 NVL, `accelerate launch --num_processes=2 --mixed_precision=bf16`.
- `--policy.type=pi05 --policy.pretrained_path=lerobot/pi05_base`.
- `--policy.compile_model=true --policy.gradient_checkpointing=true --policy.dtype=bfloat16`.
- `--policy.freeze_vision_encoder=false --policy.train_expert_only=false`.
- `--policy.normalization_mapping='{"ACTION":"MEAN_STD","STATE":"MEAN_STD","VISUAL":"IDENTITY"}'`.
- `--policy.optimizer_weight_decay=1e-10`.
- `--steps=50` per probe, 1 Hz `nvidia-smi memory.used` sampler.
- Probe script: [probe_pi05_vram.sh](../probe_pi05_vram.sh). Logs in this directory.

## Results

| per-GPU batch | effective batch | GPU0 peak | GPU0 % | GPU1 peak | GPU1 % | status |
|---:|---:|---:|---:|---:|---:|---|
|  96 | 192 |  79.59 GB | 55 % |  85.85 GB | 60 % | ok |
| 112 | 224 |  93.74 GB | 65 % |  90.39 GB | 63 % | ok |
| 128 | 256 | 101.36 GB | 70 % | 102.43 GB | 71 % | ok |
| 132 | 264 | 104.56 GB | 73 % | 101.84 GB | 71 % | ok |
| **136** | **272** | **108.78 GB** | **76 %** | **106.88 GB** | **74 %** | **ok ← chosen** |
| 140 | 280 | — | — | — | — | OOM |
| 144 | 288 | — | — | — | — | OOM |
| 152 | 304 | — | — | — | — | OOM |
| 160 | 320 | — | — | — | — | OOM (initial sweep, aborted) |

(Per-GPU % = peak / 143.77 GB.)

## Choice

**Per-GPU `--batch_size = 136`** (effective 272). Peak max(GPU0, GPU1) = 108.78 GB → 76 % of an H200 NVL. Safety margin ~7 GB before the next probed step (140 OOMs). Stays under the 80 % (~115 GB) target.

Pi0.5 is heavier than pi0 here: pi0 fits 160 per GPU at ~80 %, pi05 fits 136. The gap is consistent with pi05's larger gemma backbone driving more activation memory under `gradient_checkpointing=true`.

## Hyperparameter recommendation

Baseline (openpi pi0.5 reference): effective_batch = 32, `lr = 2.5e-5`, `warmup = 1000`.

For new effective batch = 272 (= 136 × 2) the sample budget is 8.5× the openpi baseline.

| rule | formula | new lr |
|---|---|---|
| SQRT (preferred for Transformer + AdamW) | `2.5e-5 * sqrt(272/32)` | **`7.3e-5`** |
| Linear (CV/SGD style) | `2.5e-5 * (272/32)` | `2.1e-4` |

`scheduler_warmup_steps` stays at the openpi default 1000. `optimizer_weight_decay=1e-10` and `normalization_mapping=MEAN_STD` are unchanged.

`--steps=3500` keeps the openpi sample budget at this larger batch: 3500 × 272 ≈ 952 k samples ≈ 7.97 epoch on the 119,356-frame `FR5_pick_red_colored_marker_to_box` (vs openpi reference 30000 × 32 = 960 k samples ≈ 8.04 epoch). `--steps=3000` would only see ~6.84 epoch at the new batch.

## Notes

- All probes used `--steps=50` post-compile; peak VRAM stabilizes well before step 30 in every successful run.
- OOM probes report a misleadingly-low peak in [probe_pi05_vram.sh](../probe_pi05_vram.sh) because the failure happens before steady-state allocation is reached; ignore those numbers.
- Initial sweep started at [160, 168, 172, 176, 184] targeting 90 % VRAM; pi05 OOMed at 160 (vs pi0's 79.6 % fit at the same value), so the target was lowered to 80 % and the sweep restarted at smaller batches.
