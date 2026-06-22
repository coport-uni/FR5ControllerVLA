# LearnedPatterns.md

> Patterns extracted from `ToDo.md` `[x]` items per the §10 Learned
> Patterns Bootstrap procedure in `CLAUDE.md`. Consult the relevant
> sections before drafting new `ToDo.md` entries. Append new patterns
> after each task completes (see CLAUDE.md "Learned Patterns
> Reference").
>
> Last updated: 2026-06-13
> Total patterns: 35
>
> Provenance format: `(from ToDo#N)` where N is the 1-based index of
> the top-level `##` heading in `ToDo.md` at the time of extraction.

---

## §1. Recurring Issues

### R1. `fairino` vs `fairino_follower` import path confusion

- **Problem**: `ImportError` at runtime because code imported
  `lerobot.robots.fairino` (the bundled SDK package) instead of the
  robot module `lerobot.robots.fairino_follower`.
- **Cause**: An older `robots/fairino/` copy coexists with the
  current `robots/fairino_follower/`; stale import paths leak
  through `utils.py`, entry scripts, and lazy imports.
- **Fix**: Standardised every site (factory, script imports, lazy
  imports inside `fairino_leader.py`) on `fairino_follower`. The
  nested SDK path is `lerobot.robots.fairino_follower.fairino.Robot`.
- **Rule**: Always import robots from `fairino_follower`; never from
  bare `fairino`, which is the nested SDK package, not a robot
  module. (from ToDo#7, ToDo#10, ToDo#11, ToDo#13, ToDo#28)

### R2. FairinoLeader gripper lock after MoveGripper `maxtime` expires

- **Problem**: Gripper compliance released and the jaw locked after a
  fixed duration (observed at 30 s, then 10 min, even with
  `maxtime=int32_max`); leader became un-back-drivable.
- **Cause**: `MoveGripper(..., maxtime=N, block=0)` only holds
  compliance for N ms. The SDK firmware actually in use rejects
  `maxtime=-1`.
- **Fix**: Replaced static-`maxtime` compliance with a keepalive
  thread that re-issues periodic nudge `MoveGripper` calls while
  the leader is idle; `get_action()` masks the nudge window so the
  follower does not mirror the micro-nudge.
- **Rule**: Never rely on `MoveGripper` `maxtime` for long-duration
  compliance; always pair it with a keepalive scheme when the leader
  must stay back-drivable. (from ToDo#21, ToDo#24, ToDo#25)

### R3. DDP `FileExistsError` race on `output_dir`

- **Problem**: Rank 0 creates `output_dir` before rank 1 reaches
  `TrainPipelineConfig.validate()`, so rank 1 raises
  `FileExistsError` and the run aborts.
- **Cause**: Both ranks run `cfg.validate()` without a main-process
  gate, so the directory check sees an already-created path on
  non-main ranks.
- **Fix**: Gated the existence check on
  `os.environ.get("LOCAL_RANK", "0") == "0"` in
  `src/lerobot/configs/train.py:TrainPipelineConfig.validate`.
  Single-GPU launches are unaffected (no `LOCAL_RANK` set).
- **Rule**: Always gate main-process-only filesystem checks with
  `LOCAL_RANK == "0"` (or an Accelerator `is_main_process` barrier)
  on DDP code paths. (from ToDo#34, ToDo#39)

### R4. Shell scripts hard-code FR5 control PC conda path

- **Problem**: Launch scripts source
  `/home/inno-controller/anaconda3/etc/profile.d/conda.sh` and fail
  on the H200 training box, where conda lives at `/opt/conda`.
- **Cause**: Conda install root differs per host; a single
  hard-coded `source` line silently fails elsewhere.
- **Fix**: Portable conda-source block tries
  `/home/inno-controller/anaconda3`, `/opt/conda`,
  `$HOME/anaconda3`, `$HOME/miniconda3` in order and errors clearly
  if none is present.
- **Rule**: Never hard-code a single conda install root in shared
  launch scripts; always probe known roots and error clearly on
  miss. (from ToDo#38; related hints in ToDo#28)

---

## §2. Solved Gotchas

### G1. FairinoFollower `KeyError: 'top_left'` on dataset build

- **Problem**: `lerobot-record` crashed with
  `KeyError: 'top_left'` when building a frame.
- **Cause**: `observation_features` and `get_observation()` returned
  full keys (`observation.images.top_left`) while `build_dataset_frame`
  expects short keys (`top_left`).
- **Fix**: Changed both sites to return short keys
  (`cam_name` only), matching the SO-follower reference.
- **Rule**: Always return short camera keys from
  `observation_features` and `get_observation()`; the recorder
  namespaces them downstream. (from ToDo#20)

### G2. ServoJ error 101 from daemon threads / insufficient settle

- **Problem**: `ServoJ` returned error 101 during init or when
  invoked from a background thread.
- **Cause**: Firmware V3.9.1 rejects `ServoJ` calls from non-main
  threads; state transitions during init need ≥1.0 s settle each.
- **Fix**: Removed the background servo thread and call `ServoJ`
  synchronously from the main control loop; bumped each
  `_initialise_servo_mode` settle to 1.0 s; added
  `_recover_servo_session` for error 14 (servo session expired).
- **Rule**: Never issue `ServoJ` from a daemon thread; always keep
  servo calls on the main control loop and allow ≥1 s settle
  between init state transitions. (from ToDo#3)

### G3. `git clone` returns LFS pointers instead of dataset content

- **Problem**: `5__fr5_record.sh` failed either with "destination
  path already exists" or, after clone, with
  `LeRobotDataset.load_hf_dataset()` errors because `.parquet` and
  `.mp4` files were LFS pointer stubs (~296 KB total).
- **Cause**: LeRobot dataset v3 stores media under Git LFS; plain
  `git clone` only materialises pointer files unless LFS is
  configured.
- **Fix**: Replaced `git clone` with
  `hf download --repo-type dataset coport-uni/<name> --local-dir <root>`.
- **Rule**: Always use `hf download` (never `git clone`) to pull
  LeRobot datasets from the HF Hub. (from ToDo#27)

### G4. Stale local LeRobot dataset cache silently hangs training

- **Problem**: Training launched but never stepped — both GPUs
  pinned at 100 % with `memory.util=0`; `LeRobotDataset` spun on
  parquet shard lock acquire/release instead of feeding batches.
- **Cause**: Local `~/.cache/huggingface/lerobot/<repo>/meta/info.json`
  reported an older `total_episodes`/`total_frames` than the current
  Hub revision; the parquet backend could not reconcile.
- **Fix**: Removed the four stale cache paths under
  `~/.cache/huggingface/{lerobot, datasets/parquet, hub/...}` plus
  the aborted `outputs/train/...` directory, then re-ran.
- **Rule**: When a training run loads data but never reaches
  `Start offline training`, always compare local `info.json`
  against the Hub revision before deeper debugging. (from ToDo#40)

### G5. pynput X11 listener silently no-ops on Wayland

- **Problem**: `init_keyboard_listener()` starts cleanly but
  Right/Left/Esc shortcuts in `lerobot-record` never fire on the
  FR5 control PC.
- **Cause**: pynput uses its `pynput.keyboard._xorg.Listener`
  backend; under `XDG_SESSION_TYPE=wayland`, X11 global key capture
  is blocked. Secondary: `--display_data=true` Rerun viewer can
  steal focus.
- **Fix**: Diagnostic only this pass. Options presented: switch
  login to Xorg, replace with an `evdev` reader on
  `/dev/input/event*`, or add a termios/stdin fallback when the
  terminal has focus.
- **Rule**: Never assume pynput captures keys on Wayland; always
  check `XDG_SESSION_TYPE` or pick an evdev/termios alternative
  before shipping keyboard shortcuts. (from ToDo#36)

### G6. v4l2loopback readers hang after re-running `1__setup_camera.sh`

- **Problem**: Second invocation of `1__setup_camera.sh` leaves
  old `ffmpeg` writers attached to `/dev/video18/19/20`; OpenCV
  readers then block on `cv2.VideoCapture.open`. VLC playing RTSP
  directly is unaffected.
- **Cause**: The setup script does not teardown existing writers
  before re-binding v4l2loopback.
- **Fix**: Workaround — run
  `hkvision_related/rtsp_to_v4l2_teardown.sh` before re-running
  `1__setup_camera.sh`. A real guard at the top of
  `setup_loopback` is still a follow-up item.
- **Rule**: Always teardown v4l2loopback writers
  (`rtsp_to_v4l2_teardown.sh`) before re-running
  `1__setup_camera.sh`. (from ToDo#26)

### G7. FairinoLeader gripper first-loop slam

- **Problem**: On first teleop loop, leader reported
  `gripper.pos=0.0` and the follower gripper slammed fully closed.
- **Cause**: `FairinoLeader._gripper_pos` was initialised to a
  hard-coded `0.0` instead of the current hardware position.
- **Fix**: Initialise `_gripper_pos` from `feedback["gripper.pos"]`
  on the first `send_feedback()` call, gated by a
  `_gripper_initialised` flag (mirrors `KeyboardFairinoTeleop`).
- **Rule**: Always initialise leader target state from the first
  feedback frame, never from a literal constant. (from ToDo#9)

### G8. Follower control loop drops from 60 Hz to 1 Hz on gripper commands

- **Problem**: Issuing a gripper command from the main control
  loop dropped the ServoJ rate from 60 Hz to ~1 Hz.
- **Cause**: `_send_gripper_cmd()` ran synchronously in the main
  loop with
  `ServoMoveEnd → sleep(0.02) → MoveGripper → sleep(0.02) → ServoMoveStart → sleep(0.02)`.
- **Fix**: Added a dedicated gripper worker thread; the main loop
  only writes to a latest-wins target slot. The worker uses its
  own `xmlrpc.client.ServerProxy` to avoid Transport races with
  the servo path.
- **Rule**: Never call `MoveGripper` synchronously from the main
  ServoJ loop; always dispatch it via a worker thread with its
  own RPC proxy. (from ToDo#22)

### G9. Gripper worker pause races main-thread ServoJ recovery

- **Problem**: With the gripper worker (G8) in place, pi0
  async-inference still stalled when the gripper moved — every
  ServoJ tick logged error 14 and the arm froze in a
  recover/retry loop.
- **Cause**: `MoveGripper(block=1, maxtime=30000ms)` blocks the
  worker for seconds. During that window the main thread's
  ServoJ saw the dropped servo session, called
  `_recover_servo_session()` → `ServoMoveEnd → ServoMoveStart`,
  and raced the worker's own `ServoMoveEnd / MoveGripper /
  ServoMoveStart` sequence — aborting MoveGripper and
  re-triggering error 14 on the next tick.
- **Fix**: Added `threading.Event _servo_paused`. The worker
  sets it before `ServoMoveEnd` and clears it after
  `ServoMoveStart`. While set, `send_action` skips ServoJ and
  the recovery path entirely. A `_resync_needed` flag set by
  the worker right before unpause forces the next tick to
  re-anchor `_commanded` from a live joint read.
- **Rule**: Whenever a worker thread issues `ServoMoveEnd`, gate
  the main-thread ServoJ loop (and its recovery path) behind a
  shared pause flag. A second proxy (G8) prevents transport
  races but not session-state races. (from ToDo#74)

### G10. Untagged dataset aborts training with a masked FileNotFoundError

- **Problem**: Training on a freshly pushed dataset aborted with
  `FileNotFoundError: .../lerobot/<repo>/meta/info.json`, even
  though the file existed on the Hub. A sibling dataset trained
  fine.
- **Cause**: LeRobot's `get_safe_version` (datasets/utils.py)
  resolves the dataset revision from a git tag matching
  `codebase_version` in `info.json`. The failing repo had **no
  tags** (the working one had `v3.0`). The `RevisionNotFoundError`
  it tries to raise crashes on a huggingface_hub API change
  (`HfHubHTTPError.__init__` now requires `response`), so the
  actionable "tag your dataset" message is replaced by a confusing
  `TypeError` and the local-load `FileNotFoundError`.
- **Fix**: Tag the dataset with its `codebase_version` via
  `HfApi().create_tag(repo, tag=version, repo_type="dataset")`.
  Added a pre-flight block to `7__train_act_task1_h200.sh` that
  reads `codebase_version` from the Hub `info.json`, checks repo
  tags, and creates the tag if absent.
- **Rule**: Always tag a LeRobot dataset repo with a git tag
  matching its `codebase_version` before training; an untagged
  repo fails with a misleading `FileNotFoundError`. (from ToDo#42)

---

### G11. Joint limit fault crashes teleop (surfaces as Rerun gRPC error)

- **Problem**: During FR5 leader-follower teleop, driving a joint
  to its motor limit "killed the connection"; the visible error
  was a Rerun `re_grpc_client::write ... transport error`, not a
  robot-comms failure.
- **Cause**: A joint at its limit faults/safety-stops the
  controller; the SDK TCP state thread resets (`reconnect_flag`)
  so `GetActualJointPosDegree` returns nonzero after its retries;
  `FairinoFollower.get_observation` / `FairinoLeader.get_action`
  then raise `RuntimeError`; `teleop_loop` has no per-iteration
  guard (outer try only catches `KeyboardInterrupt`), so the
  process exits and Rerun logs the transport error as it tears
  down. The Rerun error is a symptom of the crash, not the cause.
- **Fix**: (1) Both read paths now serve the last good joint
  reading (rate-limited warning) instead of raising, so the loop
  survives. (2) Added `_detect_fault()` (reads `main_code` /
  `safety_stop0/1_state` straight from the state package, no extra
  RPC) and `_recover_from_fault()` — follower re-runs the full
  enable sequence, leader re-enters drag-teach — gated by
  `_RECOVERY_MIN_INTERVAL_S` and (follower) `_servo_paused` so it
  cannot race the gripper worker (see G9). Issue #93.
- **Rule**: Never let a per-tick hardware read raise straight into
  the teleop loop; serve last-known state and recover. A Rerun
  `transport error` with no other traceback means the main process
  died — look upstream of Rerun. (from ToDo#51)

---

## §3. Library Quirks

### Q1. Fairino ServoJ 7-param signature (firmware V3.9.1)

- **Problem**: Calling `ServoJ` with 8 params (including `id`) raised.
- **Cause**: Firmware V3.9.1 drops the `id` argument.
- **Fix**: Call
  `robot.ServoJ(joint_pos, axis_pos, acc, vel, cmdT, filterT, gain)` —
  7 params, no `id`.
- **Rule**: Always use the 7-param `ServoJ` via direct XMLRPC on
  Fairino FR5 firmware V3.9.1. (from ToDo#1)

### Q2. MoveJ returns error 101/154 on Fairino FR5

- **Problem**: Standard `MoveJ` fails with error 101 or 154.
- **Cause**: Firmware V3.9.1's motion pipeline only supports
  real-time servo in the control mode used here.
- **Fix**: Use the ServoJ cycle
  (`ServoMoveStart → ServoJ loop → ServoMoveEnd`) instead.
- **Rule**: Always use ServoJ (never MoveJ) for commanded motion
  on Fairino FR5. (from ToDo#1, ToDo#3)

### Q3. RealSense D400-series needs ≥5 s warmup

- **Problem**: Default `warmup_s=1` missed D455 first frames;
  device opened but capture timed out.
- **Cause**: D400-series first-frame latency regularly exceeds 1 s
  in this environment.
- **Fix**: Bumped `warmup_s` to 5 in `find_cameras.py`.
- **Rule**: Always use `warmup_s=5` (not 1) for RealSense
  D400-series cameras. (from ToDo#12)

### Q4. RealSense V4L2 depth nodes pollute OpenCV scans

- **Problem**: OpenCV scan enumerates RealSense depth/metadata
  V4L2 nodes (`/dev/video2`, `/dev/video4`) and emits format
  warnings.
- **Cause**: V4L2 exposes every RealSense interface; `cv2.VideoCapture`
  cannot open these meaningfully.
- **Fix**: Filter them out by reading
  `/sys/class/video4linux/<dev>/name` before `cv2.VideoCapture`.
- **Rule**: Always filter RealSense-owned V4L2 nodes when scanning
  OpenCV cameras. (from ToDo#8, ToDo#12)

### Q5. NCCL P2P/CUMEM transport fails silently in `--privileged` Docker

- **Problem**: First allreduce on 2 × H200 stalls with both GPUs
  at 100 % util and zero memory traffic.
- **Cause**: NCCL P2P/CUMEM transport is blocked inside the
  container; the automatic fallback to socket does not engage.
- **Fix**: Set `NCCL_P2P_DISABLE=1 NCCL_SHM_DISABLE=1` for
  in-container training. The host NUC is unaffected; shell
  launchers are left clean.
- **Rule**: Always set `NCCL_P2P_DISABLE=1 NCCL_SHM_DISABLE=1`
  when running multi-GPU training from inside this Docker
  container. (from ToDo#32)

### Q6. Default NCCL watchdog (10 min) trips on slow first-step setup

- **Problem**: Long first-pass setup (dataset load + `torch.compile`)
  exceeded NCCL's default 10 min watchdog.
- **Cause**: First allreduce is delayed past 10 min on Pi0 / Pi0.5
  with large datasets.
- **Fix**: Pass `InitProcessGroupKwargs(timeout=1h)` in
  `lerobot_train.py`.
- **Rule**: Always extend NCCL process-group timeout to 1 h when
  launching long-setup multi-GPU training jobs. (from ToDo#32)

### Q7. `GR00TN15Config` dataclass ordering breaks on Python 3.12

- **Problem**: `lerobot-train --help` raised `TypeError` on
  `GR00TN15Config`.
- **Cause**: Four `init=False` fields without `default=` sat
  before fields with defaults; Python 3.12 rejects that ordering.
- **Fix**: Added `default=None` to `backbone_cfg`,
  `action_head_cfg`, `action_horizon`, `action_dim`.
- **Rule**: Always give `init=False` dataclass fields a `default=`
  when they sit before `init=True` fields on Python ≥3.12.
  (from ToDo#31)

### Q8. `spd-say` (speech-dispatcher) missing on base image

- **Problem**: `lerobot-record` raised `FileNotFoundError` from
  `src/lerobot/utils/utils.py:say()`.
- **Cause**: The base image does not ship `spd-say`; `Popen` fails
  before any TTS output is attempted.
- **Fix**: Installed `speech-dispatcher` via apt and hardened
  `say()` with `shutil.which` so missing TTS only logs a warning.
- **Rule**: Always guard `subprocess.Popen` on external binaries
  with `shutil.which` for graceful degradation. (from ToDo#17)

### Q9. HF Hub checkpoint pulls: use `hf auth` before downloads

- **Problem**: Silent 401 on private datasets during training
  pre-warm.
- **Cause**: Shell script assumed HF token propagation from the
  parent shell; inside activated conda env, the token was unset.
- **Fix**: The portable conda block also runs `hf auth whoami` to
  verify; explicit prompt if missing.
- **Rule**: Always verify `hf auth whoami` inside the activated
  environment before triggering Hub pulls. (from ToDo#38)

### Q10. PyPI `cmake` wrapper fails inside pip's build isolation

- **Problem**: `pip install -e ".[all]"` aborts during
  `egl_probe` / `hf-egl-probe` wheel build with
  `subprocess.CalledProcessError: Command '['cmake', '--version']'
  returned non-zero exit status 1`. Outside the build subprocess
  the same `cmake --version` works.
- **Cause**: The conda env's `cmake` is a Python wrapper script
  (`from cmake import cmake`) shipped by the PyPI `cmake`
  package. pip's build isolation overlays its own `sys.path` on
  the shebang-targeted interpreter, hiding the conda env's
  site-packages — so `import cmake` raises `ModuleNotFoundError`
  inside the build subprocess and the wrapper exits non-zero.
- **Fix**: Install a native cmake via apt
  (`apt-get install -y cmake build-essential` → `/usr/bin/cmake`)
  and `pip uninstall -y cmake` to drop the broken wrapper. The
  build subprocess then resolves `cmake` to the system binary
  with no Python dependency.
- **Rule**: Never rely on the PyPI `cmake` wrapper for source
  builds; always install a system cmake via apt before
  `pip install`-ing projects that build C/C++ extensions.
  (from ToDo#47)

### Q11. `evdev.list_devices()` silently returns [] without permissions

- **Problem**: While replacing the pynput record-shortcut listener
  (see G5), `evdev.list_devices()` returned an empty list on the
  FR5 control PC even though `/dev/input/event*` nodes exist.
- **Cause**: python-evdev filters the device list to nodes the
  current user can open; `/dev/input/event*` is `root:input 660`,
  and the user was not in the `input` group — no error is raised,
  the list is just empty.
- **Fix**: `sudo usermod -aG input <user>` plus re-login (or
  `sudo setfacl -m u:<user>:r /dev/input/event*` for the current
  boot). `EvdevKeyboardListener.start()` raises `RuntimeError`
  mentioning the `input` group so the failure is loud.
- **Rule**: Always treat an empty `evdev.list_devices()` result as
  a permission problem first; check `input`-group membership before
  debugging device detection. (from ToDo#76)

### Q12. HF Hub model repos are mutable — pin revisions for old forks

- **Problem**: `lerobot/pi0_base` suddenly failed to load with
  `ImportError: Processor step 'relative_actions_processor' not
  found in registry`; every probe died before training.
- **Cause**: The Hub repo HEAD moved (2026-06-03 commit added new
  processor steps for current LeRobot) while this fork is frozen at
  v0.5.1; `from_pretrained` without a revision always pulls HEAD.
- **Fix**: Pinned the prior revision `26b99b9439ac` (identical
  weights, older processor JSON) via `snapshot_download` to
  `models/pi0_base_v051compat/` and pointed
  `--policy.pretrained_path` at the local dir.
- **Rule**: Always pin a Hub revision (or keep a local snapshot)
  for pretrained checkpoints consumed by a version-frozen fork;
  never depend on repo HEAD. (from B200 probe, 2026-06-13, gh #87)

### Q13. `strict=False` weight loading silently leaves modules random

- **Problem**: pi0 fine-tune "loads" the checkpoint but the entire
  PaliGemma vision tower stays randomly initialized; the only trace
  is a `Warning: Could not load state dict` line.
- **Cause**: transformers 5.11.0 flattened the PaliGemma key path
  (`vision_tower.vision_model.*` → `vision_tower.*`);
  `PreTrainedPolicy.from_pretrained` calls
  `load_state_dict(strict=False)`, so every vision-tower tensor
  mismatches and is skipped without an error.
- **Fix**: Vision-tower key remap in the pi0 load path (gh #88);
  see Q12 for the companion revision pin.
- **Rule**: After any `from_pretrained` under `strict=False`,
  always inspect missing/unexpected keys before training; treat
  "Could not load state dict" warnings as fatal until explained.
  (from B200 probe, 2026-06-13, gh #87/#88)

### Q14. pi0 default `compile_mode="max-autotune"` crashes on Blackwell

- **Problem**: 4 x B200 training died at step 0 with
  `CUDA error: an illegal memory access was encountered` for
  per-GPU batch >= ~176, while batch 64/96 ran fine.
- **Cause**: pi0 compiles with `mode="max-autotune"`
  (`configuration_pi0.py:76`); inductor's Triton GEMM autotune
  benchmarks a `triton_mm` candidate that performs an illegal
  access on sm_100 (torch 2.10.0+cu128) at large batch shapes.
- **Fix**: Pass `--policy.compile_mode=default` (GEMMs fall back
  to cuBLAS/ATen, pointwise fusion kept); probe drivers expose a
  `COMPILE_MODE` env knob.
- **Rule**: Always set `--policy.compile_mode=default` for pi0 on
  the B200 box; when torch.compile dies with illegal-memory-access
  only above a batch threshold, suspect Triton GEMM autotune
  first. (from B200 probe, 2026-06-13, gh #87)

### Q15. pi05 gemma RMSNorm hits the Triton shared-mem limit on B200

- **Problem**: pi05 on 4 x B200 (with `compile_mode=default`, so Q14
  is already handled) fails to compile at per-GPU batch >= 160 with
  `No valid triton configs. OutOfMemoryError: out of resource:
  triton_per_fused_..._mean_mul_pow_rsqrt_sum_...`; batch 152 and
  below compile and train fine. This is NOT a VRAM OOM — batch 152
  peaks at only 71.5 % of the 183 GB B200.
- **Cause**: pi05's gemma backbone compiles RMSNorm to a Triton
  persistent-reduction kernel whose per-block shared memory grows
  with batch; at batch 168 it needs 294976 B vs the B200 per-SM
  limit of 232448 B, so inductor finds no valid config and the build
  fails before step 0. pi0 (Q14) does not hit this — different norm.
- **Fix**: Cap per-GPU batch at the largest rung that compiles
  (152 here, effective 608 on 4 GPUs). The usual VRAM ladder is not
  the binding constraint for pi05 on this box.
- **Rule**: For pi05 on B200, the batch ceiling is the compile
  shared-mem limit, not VRAM — probe for the largest batch that
  *compiles*, and read "out of resource" / "No valid triton configs"
  as a compile-codegen limit (reduce batch), distinct from a CUDA
  OOM. (from B200 pi05-adv probe, 2026-06-15, gh #89)

### Q16. pi05_base needs the same v051compat snapshot treatment as pi0

- **Problem**: `lerobot/pi05_base` HEAD fails to load on this v0.5.1
  fork with `ImportError: Processor step 'relative_actions_processor'
  not found` (preprocessor) and, once that is removed,
  `'absolute_actions_processor' not found` (postprocessor).
- **Cause**: Same mutable-HEAD drift as Q12 — the pi05_base HEAD adds
  two `enabled: false` no-op processor steps this fork's registry
  lacks; the preprocessor and postprocessor each carry one.
- **Fix**: `snapshot_download` the HEAD to
  `models/pi05_base_v051compat/`, then strip the two disabled steps
  from `policy_preprocessor.json` and `policy_postprocessor.json`
  (weights byte-identical); point `--policy.pretrained_path` there.
  The gh #88 vision-tower key remap (Q13) already covers pi05.
- **Rule**: When porting a pi0 checkpoint fix to pi05, check BOTH
  processor JSONs (pre and post) for unknown steps, not just the
  preprocessor. (from B200 pi05-adv probe, 2026-06-15, gh #89)

---

## §4. Workflow Lessons

### W1. `Closes #N` in commit message auto-closes the issue

- **Lesson**: Adding `Closes #N` to the commit body closes the
  referenced issue on push to the default branch; a follow-up
  `gh issue close` is redundant and returns an error.
- **Rule**: Always write `Closes #N` (or `Refs #N` for partial
  work) in commit messages; use `gh issue comment` for trailing
  notes instead of `gh issue close`. (from ToDo#7, ToDo#29)

### W2. Stage explicit file paths; never `git add -A` or `git add .`

- **Lesson**: Broad staging pulled in IDE auto-trim whitespace,
  unrelated scratch files, and previously-staged artifacts;
  `outputs/datasets/` briefly slipped in this way.
- **Rule**: Always stage files by explicit path and verify the
  staged set with `git status` plus `git diff --cached --stat`
  before committing. (from ToDo#29)

### W3. GitHub 100 MB push rejection requires history rewrite

- **Lesson**: GitHub's pre-receive hook rejects commits that
  contain files over 100 MB; `.gitignore` alone cannot fix this
  once the file is already in a commit.
- **Fix pattern**: `git reset --soft` → `git rm -r --cached` →
  re-commit → re-push.
- **Rule**: When GitHub rejects a push for size, always rewrite
  the offending commits; never try to "force through" the block.
  (from ToDo#29)

### W4. Hardware verification gates follower-facing changes

- **Lesson**: ServoJ, gripper compliance, camera-timing, and
  multi-GPU issues only closed after the user confirmed live
  hardware behaviour; smoke-tests alone were insufficient.
- **Rule**: Always require user hardware verification before
  closing any issue that touches Fairino motion, gripper
  compliance, or camera streams. (from ToDo#3, ToDo#22, ToDo#25,
  ToDo#27)

### W5. Diff-first approval for structural edits

- **Lesson**: Major structural edits to `CLAUDE.md`, `ToDo.md`, or
  shared launch scripts are presented as a plan or preview before
  execution; the user rejects several proposals between read and
  write.
- **Rule**: Always preview structural edits (renumbering, section
  reorganisation, content migration) before executing; apply only
  after explicit user approval. (from ToDo#1, this sync task)

### W6. Deleting user work is last resort; confirm before `rm -rf`

- **Lesson**: Clearing `~/.cache/huggingface/...` directories to
  unblock training required explicit user approval before
  execution.
- **Rule**: Always confirm with the user before `rm -rf` on any
  path outside of `/tmp` or the build output directories.
  (from ToDo#40)

---

## §5. Environment Specifics

### E1. Fairino network defaults — follower 192.168.58.2 / leader 192.168.59.2

- **Note**: Both robots share the NUC LAN; IPs are factory
  defaults used throughout launch scripts.
- **Rule**: Always check robot IP against these defaults before
  debugging "no connection" on a Fairino test. (from ToDo#2)

### E2. Docker `--privileged` + Wayland host needs Xauth bridge

- **Note**: The container runs `--privileged`; the NUC host runs
  Wayland with Xwayland. Opening a GUI tool (feh, xdpyinfo, etc.)
  inside the container requires importing the host's Xwayland MIT
  cookie from
  `/proc/<xwayland-pid>/root/run/user/<uid>/.mutter-Xwaylandauth.*`
  into the container `xauth`.
- **Rule**: Always source the Xauth cookie via `/proc/.../mutter-Xwaylandauth.*`
  before GUI tools can render on the NUC monitor from inside the
  container; `Xserver.sh` wraps this for SSH sessions. (from ToDo#16)

### E3. RealSense D455 requires a USB 3.2 port

- **Note**: USB 2.0 and USB 3.0 ports do not sustain 1280 × 720
  RGB at 30 fps; the device enumerates but the stream hangs or
  stalls.
- **Rule**: Always plug the RealSense D455 into a USB 3.2 port on
  the NUC; verify with `lsusb -t`. (from ToDo#14)

### E4. Camera fps must not fall below dataset fps

- **Note**: Dataset fps=20 with a camera stream at fps=30 caused
  timing mismatches; the project now uses fps=30 end-to-end.
- **Rule**: Always align camera fps with dataset fps, or keep
  camera fps ≥ dataset fps; never record with
  `dataset.fps > camera.fps`. (from ToDo#19)

### E5. Conda install root differs across hosts

- **Note**: FR5 control PC uses `/home/inno-controller/anaconda3`;
  H200 training box uses `/opt/conda`; laptop shells may use
  `$HOME/anaconda3` or `$HOME/miniconda3`.
- **Rule**: Always use the portable conda-source block from
  `7__train_act.sh`, which probes all four roots in order.
  (from ToDo#38)

### E6. LeRobot dataset v3 media is Git LFS; do not `git clone`

- **Note**: See G3 — confirmed as environment behaviour on this
  Hub namespace.
- **Rule**: Always pull datasets via `hf download --repo-type dataset`.
  (from ToDo#27)

### E7. Group changes do not propagate while `systemd --user` survives

- **Problem**: After `sudo usermod -aG input <user>` (see Q11) and
  a desktop re-login, `id` still lacked `input` and
  `evdev.list_devices()` stayed empty in every terminal (VSCode
  and native alike).
- **Cause**: The `systemd --user` daemon from the previous login
  survived the logout (a leftover process kept the session alive),
  and desktop apps are spawned through it — so every new terminal
  inherited the daemon's stale supplementary-group list.
- **Fix**: Full reboot. (`sudo loginctl terminate-user <user>` also
  works but kills the running desktop session just the same.)
  Post-reboot, evdev discovered 21 readable devices and
  `EvdevKeyboardListener.find_keyboard_devices()` matched 4
  keyboards.
- **Rule**: After changing group membership, always verify the new
  GID appears in `grep Groups /proc/$(pgrep -u $USER -x
  systemd)/status`; if it is missing, reboot — a plain desktop
  re-login is not sufficient. (from ToDo#79)

### E8. NUC USB4 xHCI "HC died" — RealSense ignores replug until reboot

- **Problem**: RealSense D455 was not recognized after unplugging
  and replugging; only a full reboot restored it (2026-06-11
  18:46 incident).
- **Cause**: The NUC15's USB4-side xHCI controller
  (`0000:00:0d.0`, USB buses 1/2) hung on a `stop endpoint`
  command while a RealSense stream was closing; the kernel logged
  `xHCI host controller not responding, assume dead` / `HC died`
  and tore down the whole bus. A dead host controller cannot
  enumerate anything, so replugging the camera is a no-op.
  Contributing factor: the camera was connected through the
  external USB3.2 dock hub (`usb 2-3`) on that controller rather
  than directly to the body.
- **Fix**: Reset the controller via PCI instead of rebooting:
  `echo 1 | sudo tee /sys/bus/pci/devices/0000:00:0d.0/remove`,
  wait 2 s, then `echo 1 | sudo tee /sys/bus/pci/rescan`.
  Prevention: plug the D455 directly into a body port wired to
  the other controller (`0000:00:14.0`, buses 3/4) and avoid the
  dock hub.
- **Rule**: When a USB camera ignores replug, always check
  `journalctl -k` for `HC died` before suspecting the camera or
  cable; recover with a PCI remove/rescan of the dead xHCI, not
  a reboot. (from RealSense diagnosis session, 2026-06-11)

### E9. D455 on shared USB hub — EPROTO (-71) storms reset the hub mid-recording

- **Problem**: `lerobot-record` aborted mid-episode; the D455
  repeatedly vanished from the bus (three full hub resets in four
  minutes, 2026-06-12 morning).
- **Cause**: The D455 was connected through an external Generic
  USB3.2 hub (`usb 2-1`) on the §E8 controller (`0000:00:0d.0`),
  sharing the hub with an r8152 gigabit LAN adapter. The kernel
  logged repeated `uvcvideo: Non-zero status (-71) in video
  completion handler` (EPROTO, link-level transfer errors) before
  each hub reset — a signal-integrity / shared-hub problem, not a
  software bug. The read thread dies after 11 consecutive
  failures, then `get_observation()` kills the session.
- **Fix** (verified 2026-06-12): Connect the D455 through an
  **active repeater USB 3 cable** into its own hub (VIA Labs,
  `usb 2-4`), separated from the r8152 LAN hub. A passive cable
  over this run length is what produced the EPROTO storms. After
  the change: USB 3.2 link confirmed, zero `-71` errors, and a
  clean 15 s lerobot stream test (333 reads, 0 timeouts, 30 fps).
  Moving the camera to a body port of the other controller
  (`0000:00:14.0`) remains the §E8-preferred fallback.
- **Rule**: Always use an active repeater cable for the D455 run
  and never share its hub with other bandwidth-hungry USB devices;
  when recordings die mid-episode, check `journalctl -k` for `-71`
  storms and hub disconnects before debugging the camera stack.
  (from mid-recording crash diagnosis, 2026-06-12, gh #83)

### E10. B200 bare-metal box: NCCL P2P must stay ENABLED (inverse of Q5)

- **Problem**: First allreduce hung forever on 4 x B200 with the
  scripts' inherited `NCCL_P2P_DISABLE=1`; every DDP run stalled
  or died.
- **Cause**: Q5's rule is specific to the Docker container on the
  H200 box. On the bare-metal NVLink/NVSwitch B200 host, disabling
  P2P forces a SHM path whose first barrier never clears.
- **Fix**: Removed `NCCL_P2P_DISABLE=1`. Measured (batch 96,
  40 steps): p2p_on OK at 4.00 s/step; p2p_off HANG; pure socket
  (`P2P+SHM` off) OK but 6.67 s/step (+67 %).
- **Rule**: Treat NCCL transport flags as per-host configuration
  and never copy them between machines; on the B200 box leave NCCL
  at its defaults (P2P on). (from B200 probe, 2026-06-13, gh #87)

### E11. B200 box `/tmp` is noexec — torch.compile cannot dlopen kernels

- **Problem**: `compile_model=true` crashed at step 0 with
  `ImportError: /tmp/torchinductor_*/...cuda_utils...so: failed to
  map segment from shared object`.
- **Cause**: `/tmp` is mounted `rw,nosuid,nodev,noexec`;
  triton/inductor write compiled kernels there and `dlopen` is
  blocked by noexec.
- **Fix**: `export TORCHINDUCTOR_CACHE_DIR` and `TRITON_CACHE_DIR`
  to an exec-allowed dir (repo `.cache/`) before launch; baked into
  the probe drivers and `7__train_pi0_adv_b200.sh`.
- **Rule**: Always redirect inductor/triton caches off `/tmp` on
  hosts with a noexec `/tmp` whenever torch.compile is enabled.
  (from B200 probe, 2026-06-13, gh #87)

### E12. B200 box conda is miniforge3; `activate` leaves `python` shadowed

- **Problem**: The portable conda block (E5) found no root on the
  B200 box; after manual activation, bare `python` still resolved
  to `/usr/bin/python` (lerobot import failed) even though
  `accelerate`/`lerobot-train` resolved into the env.
- **Cause**: This host's conda root
  (`/NHNHOME/workspace/sungwoo/miniforge3`) was missing from the
  probe list, and in non-interactive shells `conda activate` does
  not win the PATH race for bare `python`.
- **Fix**: Added the miniforge3 root to the probe loop and
  `export PATH="$root/envs/lerobot/bin:$PATH"` after activation.
- **Rule**: Always add a new host's conda root to the portable
  block and prepend the env bin to PATH when bare `python` must
  resolve into the env. (from B200 probe, 2026-06-13, gh #87)

---

## §99. Uncategorized

Procedural ritual that recurs across nearly every top-level `ToDo.md`
task, preserved here so the full `[x]` inventory is accounted for
per the §10 Bootstrap constraint against discarding items.

- Per-task workflow steps — `gh issue 등록` / `gh issue create`,
  `commit + push`, `gh issue update` / `gh issue close`. These are
  workflow scaffolding captured in the Task Management section of
  `CLAUDE.md`, not transferable lessons.
- Per-task `ruff check` / `ruff format --check` runs on modified
  Python files. Enforced mechanically by the `post-write-lint.sh`
  hook; not a lesson to record here.
- Per-task `bash -n` syntax checks on edited shell scripts.
- Per-task import-smoke probes
  (`python -c "from lerobot... import ..."`). Subsumed into the
  general "verify the import path" rule, already covered in R1.
- ToDo#23 (2026-04-20): cancelled Follower 30-second inactivity
  misdiagnosis. Closed issue #23; no lesson beyond "re-confirm the
  direction of the problem before filing", which W4 already covers.
