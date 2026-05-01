# Pi0 fine-tune VRAM probe summary (issue #55)

Date: 2026-05-01
Hardware: 2 x H200 NVL (143771 MiB / 143.77 GB per GPU)
Settings: bf16 + `compile_model=true` + `gradient_checkpointing=true`,
`freeze_vision_encoder=false` (full fine-tune), `num_workers=10`,
50 steps per probe, 1 Hz `nvidia-smi` sampling.
Probe script: [claude_test/probe_pi0_vram.sh](probe_pi0_vram.sh).

## Results

| per-GPU batch | effective batch | peak GPU0 (GB) | peak GPU1 (GB) | GPU0 % | result |
|--------------:|----------------:|---------------:|---------------:|-------:|:-------|
|            16 |              32 |          48.69 |          48.69 |  33.9% | OK (production) |
|            24 |              48 |          50.54 |          50.53 |  35.2% | OK     |
|            32 |              64 |          54.20 |          54.20 |  37.7% | OK     |
|            48 |              96 |          60.09 |          60.66 |  42.2% | OK     |
|            64 |             128 |          60.06 |          61.82 |  43.0% | OK     |
|            96 |             192 |          74.07 |          75.07 |  52.2% | OK     |
|           128 |             256 |          91.89 |          95.07 |  66.1% | OK     |
|           144 |             288 |         102.33 |         100.02 |  71.2% | OK     |
|       **160** |         **320** |     **114.42** |     **108.92** | **79.6%** | **OK (~80% target)** |
|           176 |             352 |            --- |            --- |      — | OOM    |
|           192 |             384 |            --- |            --- |      — | OOM    |
|           256 |             512 |            --- |            --- |      — | OOM    |

## Decision

`per-GPU batch_size = 160` (effective 320 with 2 GPUs).
GPU0 peak 114.42 GB (79.6%), GPU1 peak 108.92 GB (75.8%).

## Hyperparameter recommendation

Baseline (openpi pi0 fine-tune): effective_batch=32, `lr=2.5e-5`, warmup=1000 steps.
Effective batch ratio: B'/B = 320/32 = 10.

| Rule          | Formula                       | New `lr`         | Note                                            |
|---------------|-------------------------------|------------------|-------------------------------------------------|
| **SQRT**      | `2.5e-5 * sqrt(10)`           | **`7.9e-5`**     | **Recommended.** Safer for Transformer + AdamW. |
| Linear        | `2.5e-5 * 10`                 | `2.5e-4`         | Aggressive; only if SQRT underfits.             |

`scheduler_warmup_steps`: keep at 1000 (warmup length is not directly
proportional to batch size). Keep `optimizer_weight_decay`,
`betas=(0.9, 0.95)`, decay schedule unchanged.

Note: increasing per-GPU batch from 16 to 160 also reduces gradient noise
by ~sqrt(10), so optimization may need more steps to reach the same loss
even at the new lr. If using `--steps=30000` produces an underfit model,
either raise lr toward the linear bound or extend `--steps`.

## OOM behavior

Peak shown for OOM rows (~50 GB) is the last value sampled before the
crash, not a steady-state peak. The OOM happens during the first
forward/backward at the new batch shape, before activation memory
stabilises.
