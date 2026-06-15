# Pi0.5-adv VRAM + compile probe summary -- 4 x B200 (issue #89)

Date: 2026-06-15
Hardware: 4 x NVIDIA B200, 183359 MiB (179.06 GB) per GPU.
Env: conda `lerobot` at `/NHNHOME/workspace/sungwoo/miniforge3`,
torch 2.10.0+cu128 (CUDA 12.8, Blackwell sm_100), transformers 5.11.0.
Settings: bf16 + `compile_model=true` + `compile_mode=default` +
`gradient_checkpointing=true`, `freeze_vision_encoder=false` (full
fine-tune), `optimizer_weight_decay=1e-10`,
`normalization_mapping=MEAN_STD`, `num_workers=10`, `num_processes=4`,
1 Hz `nvidia-smi` sampling.
Probe driver: [claude_test/probe_pi05_vram_b200.sh](../probe_pi05_vram_b200.sh).
Follows the pi0 B200 probe in [SUMMARY_b200.md](SUMMARY_b200.md) (#87)
and the H200 pi05 probe in [SUMMARY_pi05.md](SUMMARY_pi05.md) (#58).

## Blockers found during setup (all fixed)

### 1. pi05_base HEAD ships processor steps unknown to this fork
`lerobot/pi05_base` HEAD (revision `7de66397`) ships a
`relative_actions_processor` step in `policy_preprocessor.json` and an
`absolute_actions_processor` step in `policy_postprocessor.json` (both
`enabled: false` no-ops). This v0.5.1 fork's `ProcessorStepRegistry`
has neither, so the pipeline load raises
`ImportError: ... not found in registry` before training starts -- the
exact pi0 blocker from SUMMARY_b200.md, now for pi05.

Fix: snapshot the HEAD to `models/pi05_base_v051compat/` and drop just
those two disabled steps from the two processor JSONs (weights are
byte-identical -- only the processor JSON changed). Used as
`--policy.pretrained_path`. Model weights load cleanly (812 keys
remapped, including the PaliGemma vision-tower key remap from gh #88,
which already covers pi05).

### 2. pi05 default compile_mode="max-autotune" crashes on sm_100
With the processor fixed, the run died at step 0 with
`CUDA error: an illegal memory access was encountered`, raised inside
Triton GEMM autotuning (`select_algorithm.py`). pi05 compiles with
`compile_mode="max-autotune"` by default; one autotune candidate
performs an illegal access on Blackwell (sm_100) under
torch 2.10.0+cu128 -- identical to the pi0 B200 blocker.

Fix: pass `--policy.compile_mode=default` (GEMMs fall back to
cuBLAS/ATen; pointwise fusion kept).

### 3. NCCL P2P + noexec /tmp (inherited from the pi0 B200 probe)
`NCCL_P2P_DISABLE=0` (P2P must stay on; `=1` hangs the first allreduce
here) and `TORCHINDUCTOR_CACHE_DIR` / `TRITON_CACHE_DIR` redirected off
noexec `/tmp`. See SUMMARY_b200.md blockers #3 and #5.

## VRAM + compile ladder results (per-GPU batch, 4 x B200)

50-step rungs at 128 and 152; 12-step boundary-search rungs at
136/144/160/168 (peak VRAM stabilizes well before step 12). Peaks from
1 Hz `nvidia-smi` sampling across all 4 GPUs:

| per-GPU batch | effective (x4) | peak GPU (MiB) | peak % of 183359 | result |
|--------------:|---------------:|---------------:|-----------------:|:-------|
| 128           |            512 |         109300 |           59.6 % | OK |
| 136           |            544 |         115900 |           63.2 % | OK |
| 144           |            576 |         114028 |           62.2 % | OK |
| **152**       |        **608** |     **131024** |       **71.5 %** | **OK, 50/50 steps, 5.30 s/step** |
| 160           |            640 | 53844 (pre-fail) |             -- | FAIL (Triton shared-mem) |
| 168           |            672 | 55448 (pre-fail) |             -- | FAIL (Triton shared-mem) |

The failure at >= 160 is NOT VRAM (152 leaves ~28 % headroom). pi05's
gemma RMSNorm compiles to a Triton persistent-reduction kernel
(`triton_per_fused_..._mean_mul_pow_rsqrt_sum_...`) whose per-block
shared memory grows with batch: at batch 168 it needs 294976 B vs the
B200 per-SM limit of 232448 B, so torch.compile reports
"No valid triton configs. OutOfMemoryError: out of resource" and the
build fails before the first step. (Pre-fail peaks are mid-compile
samples, not steady state.)

## Decision (batch)

`per-GPU batch_size = 152` (effective 608 with 4 GPUs): the largest
rung that compiles, peaking at 71.5 % VRAM. The binding constraint is
the compile shared-memory limit, not VRAM -- the next rung (160) fails
to compile. Same accept-below-failure rule as the pi0 B200 probe.

## Hyperparameter recommendation (LR + steps)

Baseline (openpi pi0.5 reference): effective_batch=32, `lr=2.5e-5`,
warmup=1000. SQRT rule: `lr = 2.5e-5 * sqrt(608/32) = 1.09e-4` ->
**`1.1e-4`**. `scheduler_warmup_steps` stays at 1000. Linear bound for
reference: `2.5e-5 * 19 = 4.75e-4` (aggressive; only if SQRT underfits).

`--steps=1570`: 1570 * 608 = 954,560 samples ~= 8.00 epoch on the
119,356-frame FR5_pick_red_colored_marker_to_box dataset (holds the
openpi sample budget; cf. 30000 * 32 = 960 k and the H200 adv run's
3500 * 272 = 952 k). `save_freq=157` (CHECKPOINT_NUMBER=10).

Production script: [7__train_pi05_adv_b200.sh](../../7__train_pi05_adv_b200.sh).
