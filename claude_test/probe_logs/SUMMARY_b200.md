# Pi0 fine-tune VRAM + NCCL probe summary -- 4 x B200 (issue #87)

Date: 2026-06-13
Hardware: 4 x NVIDIA B200, 183359 MiB (179.06 GB) per GPU.
Env: conda `lerobot` at `/NHNHOME/workspace/sungwoo/miniforge3`,
torch 2.10.0+cu128 (CUDA 12.8, Blackwell sm_100), transformers 5.11.0.
Settings: bf16 + `compile_model=true` + `gradient_checkpointing=true`,
`freeze_vision_encoder=false` (full fine-tune), `num_workers=10`,
`num_processes=4`, 50 steps per probe, 1 Hz `nvidia-smi` sampling.
Probe driver: [claude_test/probe_pi0_vram_b200.sh](../probe_pi0_vram_b200.sh).
Follows the H200 probe in [SUMMARY.md](SUMMARY.md) (issue #55).

## Blocker found during setup: pi0_base no longer loads on this env

Two independent dependency drifts surfaced; both must be understood
before trusting a real training run on this box.

### 1. Processor registry (fixed)
`lerobot/pi0_base` HEAD (commit `25c379b`, 2026-06-03 "Add relative
action processor steps") ships `policy_preprocessor.json` with a step
`relative_actions_processor` (config `enabled: false`). This v0.5.1
fork's `ProcessorStepRegistry` has no such step, so the pipeline load
raises `ImportError` before training starts -- every probe in the first
ladder run died here, not on VRAM.

Fix: pin the checkpoint to the prior revision `26b99b9439ac`
(2026-01-22), whose preprocessor lacks that step (weights are
byte-identical -- only the processor JSON changed). Snapshotted to
`models/pi0_base_v051compat/` and used as `--policy.pretrained_path`.

### 2. transformers 5.11.0 PaliGemma keys (FIXED -- key remap, gh #88)
With the processor fixed, weight loading warns:
`Could not load state dict ... Missing/Unexpected key(s)`. The
checkpoint stores the vision tower as
`...paligemma.model.vision_tower.vision_model.embeddings...`, but the
model built under transformers 5.11.0 expects
`...paligemma.model.vision_tower.embeddings...` (the inner
`vision_model` module was flattened). No size mismatches -- purely a
key-path rename. `PreTrainedPolicy.from_pretrained` loads with
`strict=False` and does NOT route through
`PI0Policy._fix_pytorch_state_dict_keys` (which only handles adaRMS /
time_mlp / lm_head, not the `vision_model` nesting), so the entire
vision tower is left randomly initialized.

Impact: VRAM/NCCL probing was unaffected (memory and comm depend on
tensor shapes + batch, which are built correctly from config), but a
real fine-tune would have started from a random vision encoder.

Resolved (gh #88): a conditional bidirectional key remap in
`PI0Policy._fix_pytorch_state_dict_keys` rewrites a checkpoint key
only when it is absent from the model and the rewritten name is
present. Verified with `claude_test/verify_pi0_vision_load.py`:
"All keys loaded successfully", 437/437 vision-tower tensors match
the safetensors content.

### 3. NCCL_P2P_DISABLE=1 hangs DDP on this box (fixed -- inverted)
The production scripts inherit `NCCL_P2P_DISABLE=1` from issue #30
(P2P stalled the first allreduce inside a Docker image on the H200
box). On this bare-metal B200 NVLink machine the situation is exactly
inverted -- the NCCL sweep (per-GPU batch 96, 40 steps each) showed:

| config        | result                                | steady s/step |
|---------------|---------------------------------------|---------------|
| `p2p_off` (`NCCL_P2P_DISABLE=1`, inherited default) | HANG -- first allreduce never clears (480 s timeout) | n/a |
| `p2p_on` (P2P enabled)                | OK, completed 40 steps | **4.00** |
| `p2p_shm_off` (pure socket)           | OK, completed 40 steps | 6.67 (+67 %) |

Rule: never set `NCCL_P2P_DISABLE=1` on this B200 box; leave P2P at
its enabled default. (Pure socket works as a fallback but is 67 %
slower.) The first two ladder attempts ran with `p2p_off` and are
invalid.

### 4. pi0 default compile_mode="max-autotune" crashes on sm_100 (fixed)
With P2P fixed, the ladder still died at step 0 for per-GPU batch
176-240: `CUDA error: an illegal memory access was encountered`,
raised inside Triton GEMM autotuning
(`torch/_inductor/select_algorithm.py`, "Runtime error during
autotune"). LeRobot pi0 compiles with
`compile_mode="max-autotune"` by default
(`configuration_pi0.py:76`), which benchmark-autotunes `triton_mm`
candidate kernels; at large batch shapes one of those candidates
performs an illegal access on Blackwell (sm_100) under
torch 2.10.0+cu128. Runs at per-GPU batch 64 and 96 passed because
the failing shapes never arise.

Fix: pass `--policy.compile_mode=default` (GEMMs fall back to
cuBLAS/ATen; pointwise fusion is kept). The probe driver does this via
the `COMPILE_MODE` env knob, and the production script must carry the
same flag.

### 5. noexec /tmp breaks torch.compile (fixed)
Under `compile_model=true` the probe crashed at step 0/50 with
`ImportError: /tmp/torchinductor_*/...cuda_utils...so: failed to map
segment from shared object`. `/tmp` is mounted `noexec`
(`rw,nosuid,nodev,noexec`), so triton/inductor cannot `dlopen` the
kernels it compiles there. This would also crash the production
`7__train_pi0_adv_b200.sh` (compile_model=true).

Fix: redirect both caches to an exec-allowed dir before launch --
`TORCHINDUCTOR_CACHE_DIR` and `TRITON_CACHE_DIR` under the repo
`.cache/`. Baked into both probe drivers; must also be added to the
production training script.

## VRAM ladder results (per-GPU batch, 4 x B200)

Ladder v3 (the first valid run: P2P on + `compile_mode=default`; the
two earlier attempts died on blockers #3/#4 above). 50 steps per rung,
peaks from 1 Hz `nvidia-smi` sampling across all 4 GPUs:

| per-GPU batch | effective (x4) | peak GPU (MiB) | peak % of 183359 | result |
|--------------:|---------------:|---------------:|-----------------:|:-------|
| **176**       |        **704** |     **135458** |       **73.9 %** | **OK, 50/50 steps, 6.29 s/step** |
| 192           |            768 | 52652 (pre-OOM) |               -- | OOM at first forward/backward |
| 208           |            832 | 55432 (pre-OOM) |               -- | OOM |
| 224           |            896 | 58206 (pre-OOM) |               -- | OOM |
| 240           |            960 | 60980 (pre-OOM) |               -- | OOM |

(Pre-OOM peaks are the last sample before the crash, not steady
state -- same caveat as the H200 probe.)

## Decision (batch)

`per-GPU batch_size = 176` (effective 704 with 4 GPUs): the largest
rung that completes, peaking at 73.9 % VRAM. The next rung (192) OOMs
on the first-step transient even though arithmetic steady state would
fit -- mirroring the H200 probe, where 176 OOMed above the accepted
160. A midpoint probe (184) would buy at most ~3 % more VRAM for 32
extra effective samples; not worth the fragility margin.

## Hyperparameter recommendation (LR)

Baseline (openpi pi0 fine-tune): effective_batch=32, `lr=2.5e-5`,
warmup=1000. SQRT rule (recommended, per issue #55):
`lr = 2.5e-5 * sqrt(effective_batch / 32)`.

Effective batch 704 = 22x openpi's 32:
`lr = 2.5e-5 * sqrt(22) = 2.5e-5 * 4.690 = 1.17e-4` -> **`1.2e-4`**.
`scheduler_warmup_steps` stays at the LeRobot default 1000 (warmup
length is not proportional to batch size). Linear bound for reference:
`2.5e-5 * 22 = 5.5e-4` (aggressive; only if SQRT underfits).

## NCCL transport (4 x B200)

issue #30 forced `NCCL_P2P_DISABLE=1` because P2P stalled the first
allreduce inside a Docker image. This box is bare-metal NVLink/NVSwitch,
so P2P may work and be faster. Probe driver:
[claude_test/probe_nccl_b200.sh](../probe_nccl_b200.sh).

Per-GPU batch 96, 40 steps each, `timeout 480` to catch hangs:

| config        | result | steady s/step |
|---------------|--------|---------------|
| p2p_off (`NCCL_P2P_DISABLE=1`) | HANG/timeout -- first allreduce never cleared | n/a |
| p2p_on (P2P enabled, default)  | OK, completed 40 steps | **4.00** |
| p2p_shm_off (pure socket)      | OK, completed 40 steps | 6.67 |

Verdict: leave NCCL at its defaults on this box (P2P on). Do NOT
carry over `NCCL_P2P_DISABLE=1` from the H200/Docker scripts -- it
hangs DDP here (see blocker #3 above).
