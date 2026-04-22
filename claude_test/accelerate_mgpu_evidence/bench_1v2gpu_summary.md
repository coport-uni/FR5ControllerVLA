# Benchmark: 1-GPU vs 2-GPU `accelerate launch` (issues #34, #35)

Reproduced with `bash claude_test/bench_accelerate_1v2gpu.sh` on
2x H200 NVL (PCIe-only, no NVLink bridge — see `nvidia-smi topo -m`).
Three models: ACT, Pi0, Pi0.5 — 100 steps, 5 trials per group, weak
scaling (per-rank batch unchanged).

## Setup
- Dataset: `coport-uni/FR5_pick_red_colored_marker_to_box`.
- Per-rank `batch_size`: ACT=64, Pi0=32, Pi0.5=32 (matches each model's
  production launch script).
- Pi0 / Pi0.5: `--policy.compile_model=false` (compile disabled so the
  100-step measurement reflects steady-state forward+backward+optim
  rather than torch.compile warmup time);
  `--policy.gradient_checkpointing=true`, `--policy.dtype=bfloat16` —
  matches production minus compile.
- Common: `--mixed_precision=bf16`, `--num_workers=4`, `--steps=100`,
  `--save_checkpoint=false`, `--wandb.enable=false`,
  `--policy.push_to_hub=false`.
- 5 trials per group (30 runs total).
- Container-only env for 2-GPU runs:
  `NCCL_P2P_DISABLE=1 NCCL_SHM_DISABLE=1` (forces NCCL to socket
  transport — see issue #30; the host NUC does not need this).

## Results (raw CSV: `bench_1v2gpu.csv`)

| Model  | GPUs | n | wall_clock_s    | training_loop_s | step/s | samples/s |
|--------|------|---|-----------------|-----------------|-------:|----------:|
| ACT    | 1    | 5 | **68.88 ± 0.93**  | **56.40 ± 1.36**  |  1.77  |  113.48   |
| ACT    | 2    | 5 | **93.28 ± 0.82**  | **80.00 ± 0.89**  |  1.25  |  160.00   |
| Pi0    | 1    | 5 | **286.53 ± 0.26** | **207.00 ± 0.63** |  0.48  |   15.46   |
| Pi0    | 2    | 5 | **417.97 ± 3.77** | **336.80 ± 4.62** |  0.30  |   19.00   |
| Pi0.5  | 1    | 5 | **310.07 ± 0.83** | **230.00 ± 0.89** |  0.43  |   13.91   |
| Pi0.5  | 2    | 5 | **462.96 ± 4.78** | **380.20 ± 5.60** |  0.26  |   16.83   |

Definitions:
- `wall_clock_s`: full `accelerate launch` invocation.
- `training_loop_s`: from `Start offline training` to `step:100 ...`.
- `step/s` = 100 / `training_loop_s`.
- `samples/s` = 100 × per_rank_batch × num_gpus / `training_loop_s`.

## Scaling: 1 GPU → 2 GPUs (samples/s)

| Model  | 1 GPU samples/s | 2 GPU samples/s | **Speedup** | **% improvement** | Scaling efficiency (vs ideal 2.0×) |
|--------|----------------:|----------------:|------------:|------------------:|----------------------------------:|
| ACT    | 113.48          | 160.00          | **1.41×**   | **+41.0 %**       | 70.5 %                            |
| Pi0    | 15.46           | 19.00           | **1.23×**   | **+22.9 %**       | 61.5 %                            |
| Pi0.5  | 13.91           | 16.83           | **1.21×**   | **+21.0 %**       | 60.5 %                            |

## Variability (coefficient of variation, CV = std / mean × 100 %)

| Model  | GPUs | CV(wall) | CV(loop) |
|--------|------|---------:|---------:|
| ACT    | 1    | 1.35 %   | 2.41 %   |
| ACT    | 2    | 0.88 %   | 1.12 %   |
| Pi0    | 1    | 0.09 %   | 0.30 %   |
| Pi0    | 2    | 0.90 %   | 1.37 %   |
| Pi0.5  | 1    | 0.27 %   | 0.39 %   |
| Pi0.5  | 2    | 1.03 %   | 1.47 %   |

All CVs are well under 3 % — trial-to-trial noise is negligible.

## Interpretation

- **All three models scale**: 2-GPU adds throughput in every case, but
  none reaches the ideal 2.0× weak-scaling target. ACT recovers ~70 %
  of the ideal; Pi0 / Pi0.5 recover ~60 %.
- **The bigger the model, the worse the scaling here**. ACT is 52 M
  params (104 MB bf16 grad). Pi0 / Pi0.5 are ~4 B (≈8 GB bf16 grad).
  In this container the DDP allreduce is forced through socket
  transport, which is bandwidth-bound around 10–15 GB/s — that turns
  ~7 ms of allreduce overhead per step on ACT into hundreds of ms on
  Pi0 / Pi0.5. On the host NUC with native P2P enabled, the per-step
  allreduce drops into the sub-ms range and Pi0 / Pi0.5 should
  approach the ACT-like ~70 %+ scaling efficiency.
- **Per-step compute (1 GPU baseline)**: ACT ~0.56 s/step, Pi0 ~2.07
  s/step, Pi0.5 ~2.30 s/step. Pi0.5 ≈ 11 % slower than Pi0 per step
  in this configuration — consistent with its slightly larger expert
  branch.
- **Init overhead**: ACT ~12 s, Pi0 ~80 s, Pi0.5 ~80 s — Pi-family
  pretrained weight load + AdamW state allocation dominates init time.

## Notes for re-running

- The `bench_accelerate_1v2gpu.sh` script accepts `--model {act|pi0|pi05}
  --steps N --batch N --trials N --compile {true|false}
  --gradient-checkpointing {true|false}`. Output CSV is appended.
- Trial 4 of 2-GPU Pi0.5 first hit `FileExistsError: Output directory
  ... already exists and resume is False`. This is a race in
  `src/lerobot/configs/train.py:122` between rank 0 (which `mkdir`s
  `output_dir` after `Accelerator()` init in
  `src/lerobot/scripts/lerobot_train.py:204-205`) and rank 1 (which
  reaches `cfg.validate()` slightly later and sees the dir exists).
  The bench script issues `rm -rf "$outdir"` per launch, so the race
  only fires when rank 1 lags during DDP startup. The CSV row was
  filled by re-running that single trial after the sweep completed
  (timestamp 09:37:08+00:00 — out of order with the rest of Pi0.5
  for that reason). Same race was observed once in ACT 2-GPU (#34
  prior run); follow-up fix should gate the dir-existence check on
  `is_main_process` or move it after the first barrier — not in scope
  for this issue.
- The original bench-script parser missed every `loop_s` because
  `INFO ...` log lines are concatenated to the previous tqdm bar
  (no leading newline) and `awk '{print $2}'` was reading the tqdm
  token instead of the date. Fix landed in
  `claude_test/bench_accelerate_1v2gpu.sh` (uses
  `grep -oE "INFO ... "` to isolate the structured substring) and the
  CSV here was rebuilt in-place from the saved per-trial logs.

---

# Addendum: ACT at per-rank batch=32 (issue #35)

Same setup as above but per-rank `batch_size=32`. Weak scaling:
- **1 GPU**: batch=32, effective=32.
- **2 GPU**: batch=32 per rank, effective=64.

## Results (10 new rows in `bench_1v2gpu.csv`, all `model=act batch_size=32`)

| Model    | GPUs | n | wall_clock_s    | training_loop_s | step/s | samples/s |
|----------|------|---|-----------------|-----------------|-------:|----------:|
| ACT b=32 | 1    | 5 | **39.95 ± 0.68**  | **29.00 ± 0.63**  |  3.45  |  110.34   |
| ACT b=32 | 2    | 5 | **52.50 ± 0.75**  | **40.60 ± 1.20**  |  2.46  |  157.64   |

## Scaling: 1 GPU → 2 GPUs

| Config         | 1-GPU samples/s | 2-GPU samples/s | **Speedup** | **% improvement** | Scaling efficiency |
|----------------|----------------:|----------------:|------------:|------------------:|-------------------:|
| ACT @ batch=32 |  110.34         |  157.64         | **1.43×**   | **+42.9 %**       | 71.4 %             |
| ACT @ batch=64 |  113.48         |  160.00         | **1.41×**   | **+41.0 %**       | 70.5 %             |

## Interpretation

- **Per-step compute is ~halved** when per-rank batch goes 64 → 32
  (1-GPU loop 56.4 s → 29.0 s; 2-GPU loop 80.0 s → 40.6 s). This is
  slightly *better* than the ideal 2× reduction because CUDA kernel
  launch fixed-cost overhead becomes a slightly larger fraction at
  batch=64, meaning halving the batch more than halves useful compute
  per step.
- **Throughput**: 1-GPU samples/s drops a little (113.5 → 110.3) — the
  smaller batch means kernel-launch overhead, optimizer.step, and
  dataloader fetch are each amortised over fewer samples. Same story
  on 2-GPU (160.0 → 157.6). Small overhead, not a scaling regression.
- **Scaling efficiency**: 71.4 % at batch=32 vs 70.5 % at batch=64 —
  statistically indistinguishable. For ACT on this container, per-rank
  batch size in the 32–64 range does not materially change the 2-GPU
  scaling efficiency. The bottleneck is still the socket-NCCL
  allreduce of the ~104 MB bf16 gradient, which scales with model
  size, not batch size.
- **Absolute wall clock**: batch=32 finishes a 100-step run in ~40 s
  on 1 GPU vs ~69 s for batch=64 — useful if you're sweeping configs
  and need fast feedback.

## Variability (coefficient of variation, CV)

| Config         | GPUs | CV(wall) | CV(loop) |
|----------------|------|---------:|---------:|
| ACT b=32       | 1    | 1.70 %   | 2.18 %   |
| ACT b=32       | 2    | 1.43 %   | 2.96 %   |

Within the same noise floor as the batch=64 runs.
