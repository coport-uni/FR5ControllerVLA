# claude_test/

Debug and exploratory test scripts created during Fairino FR5 integration.
These are **not** production tests -- they are preserved here for reference.

## Files

| File | Purpose |
|------|---------|
| `test_fairino_debug.py`  | Basic connection and MoveJ test |
| `test_fairino_debug2.py` | Similar basic connection tests |
| `test_fairino_debug3.py` | Mode switch and StartJOG fallback |
| `test_fairino_debug4.py` | Comprehensive diagnostic: Mode, MoveJ, JOG, drag-teach |
| `test_fairino_debug5.py` | XMLRPC direct calls and forward kinematics |
| `test_fairino_debug6.py` | Error handling and enable/disable sequences |
| `test_fairino_debug7.py` | Edge cases and additional diagnostics |
| `test_fairino_joint1.py` | Single joint movement test |
| `teleop_fairino_basic.py`| Early teleop using LeRobot's basic KeyboardTeleop |
| `test_fr5_leader_follower.py` | Diagnose FR5-to-FR5 leader-follower: drag-teach read + ServoJ write |
| `show_image.sh` | Open an image with `feh` on the NUC's Xwayland :0 (default) or on a laptop via SSH X11-forwarded display. Usage: `claude_test/show_image.sh [--mode=ssh\|host] <png>`. In ssh mode reads `CLAUDE_SSH_DISPLAY` / `CLAUDE_SSH_XAUTH` (set by sourcing `Xserver.sh` at the repo root). |
| `debug_leader_follower.py` | Minimal XMLRPC-only leader-follower test: init drag-teach on leader, ServoMoveStart on follower, mirror joints for 10s via ServoJ. Reports per-step errors and final position diff. |
| `accelerate_mgpu_evidence/train.log` | ACT multi-GPU training log from the issue #30 verification run on 2x H200 (batch 64 per rank, effective 128, bf16). Reached step 460 in ~6 min with loss 24.25 → 1.69 and no errors — confirms `accelerate launch` wiring end-to-end. |
| `accelerate_mgpu_evidence/gpu_samples.log` | `nvidia-smi` snapshots (10 s interval) during the same run. Both GPUs hold ~44 GB and alternate 95-100% util, confirming both H200s are live. |
| `accelerate_mgpu_evidence/smoke_nccl_socket_ok.log` | Minimal 2-proc `accelerate launch` smoke test (no LeRobot) showing that with `NCCL_P2P_DISABLE=1 NCCL_SHM_DISABLE=1` the first allreduce completes in 0.65 s; without them the same collective hangs past any timeout inside this Docker container. |
| `bench_accelerate_1v2gpu.sh` | Issue #34 benchmark: time to step 100 for `accelerate launch --num_processes=1` vs `--multi_gpu --num_processes=2`. Weak scaling (per-rank batch=64), 5 trials each, writes `accelerate_mgpu_evidence/bench_1v2gpu.csv` with both wall-clock and training-loop time per trial, then prints a mean±std summary. |
| `accelerate_mgpu_evidence/bench_1v2gpu.csv` | Output of the above benchmark (`trial,num_gpus,wall_clock_s,training_loop_s,final_loss,timestamp`). |
| `accelerate_mgpu_evidence/bench_1v2gpu_summary.md` | Human-readable summary table + interpretation notes for the bench above (1-GPU vs 2-GPU mean ± std, samples/s, scaling efficiency, plus the FileExistsError race observed on one 2-GPU trial). |
| `probe_pi0_vram.sh` | Issue #55 probe: short Pi0 fine-tune at a given per-GPU `batch_size` with `compile_model=true` + `gradient_checkpointing=true` + bf16 (matching `7__train_pi0_adv.sh`). Spawns a 1 Hz `nvidia-smi` sampler and reports per-GPU peak VRAM. Used to find the largest batch that fits ~80% of an H200 NVL (~115 GB / GPU). Output isolated under `outputs/train/fr5_pi0_vram_probe_b<N>/`; logs in `claude_test/probe_logs/`. |
| `probe_logs/` | Per-batch `nvidia-smi` CSV samples and accelerate/lerobot stdout from the probes above. |
| `probe_logs/SUMMARY.md` | Issue #55 result table for `7__train_pi0_adv.sh`: probe results from per-GPU batch 16-256, the ~80% boundary lands at per-GPU batch=160 (effective 320, GPU0 peak 79.6% / GPU1 75.8%); per-GPU batch=176 OOMs. Includes the SQRT (recommended `lr=7.9e-5`) and linear (`lr=2.5e-4`) scaling options for the new effective batch. |
| `probe_vram_batch.sh` | Issue #56 probe: launches `7__train_act_adv.sh`-equivalent ACT training at a given per-GPU `--batch_size`, samples `nvidia-smi` every 2 s, and reports peak VRAM per GPU. Disables wandb / `push_to_hub`, writes throwaway output to `/tmp`. Usage: `claude_test/probe_vram_batch.sh <bs> [duration_s]`. |
| `probe_vram_smolvla.sh` | Issue #59 probe: SmolVLA counterpart of `probe_vram_batch.sh`. Launches the `7__train_smovla_adv.sh` recipe (smolvla_base, freeze vision encoder, train expert only, `compile_model=true`, the rename_map) at a given per-GPU `--batch_size`, samples `nvidia-smi` every 2 s, reports peak VRAM. Default duration 360 s (compile warmup is longer than ACT). Usage: `claude_test/probe_vram_smolvla.sh <bs> [duration_s]`. |
| `probe_pi05_vram.sh` | Issue #58 probe: same shape as `probe_pi0_vram.sh` but with pi05 policy options (`policy.type=pi05`, `pretrained_path=lerobot/pi05_base`, `normalization_mapping=MEAN_STD`, `optimizer_weight_decay=1e-10`). Used to find the largest per-GPU batch that fits ~80% of an H200 NVL (~115 GB / GPU). Output isolated under `outputs/train/fr5_pi05_vram_probe_b<N>/`; logs in `claude_test/probe_logs/{train,vram}_pi05_b<N>.{log,csv}`. |

## Key Findings

- **MoveJ** returns error 101 or 154 on firmware V3.9.1; use **ServoJ** instead.
- Init sequence: `RobotEnable(0)` -> `ResetAllError` -> `RobotEnable(1)` -> `Mode(0)` -> `ServoMoveStart`.
- ServoJ XMLRPC call takes **7 params** (no `id` arg); the SDK wrapper passes 8.
- `main_code=1` indicates a residual error state; must disable-then-enable to clear.
- **Init settle times must be ≥1 s** per step; 0.2–0.3 s causes persistent ServoJ error 101.
- **ServoJ must run from the main thread**; daemon threads always get error 101 on this firmware.
- Hold-position ServoJ (same position) returns 101; after ~40 consecutive 101s the controller terminates the session (error 14).
| `check_collision_level.py` | Issue #78: applies `SetAnticollision(0, [level]*6, 1)` to set and persist the FR5 per-joint collision grade (1 = most sensitive, 10 = least). Makes exactly one documented XMLRPC call — the original `system.listMethods` introspection probe crashed the controller's control service (ports 20003/20004 went down) and was removed; there is no SDK getter, so read the current grade from the WebApp safety page. Usage: `python claude_test/check_collision_level.py [--ip 192.168.58.2] [--level 10]`. |
