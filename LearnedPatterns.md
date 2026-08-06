# LearnedPatterns.md

> Patterns extracted from `ToDo.md` `[x]` items per the §10 Learned
> Patterns Bootstrap procedure in `CLAUDE.md`. Consult the relevant
> sections before drafting new `ToDo.md` entries. Append new patterns
> after each task completes (see CLAUDE.md "Learned Patterns
> Reference").
>
> Last updated: 2026-07-26
> Total patterns: 59
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

### G12. Backgrounding a compound command over ssh hangs the session

- **Problem**: `ssh host "... cd repo && nohup bash 8__run_server.sh
  > log 2>&1 < /dev/null & echo done"` started the remote policy
  server, but the ssh call never returned and the client script hung.
- **Cause**: Backgrounding a compound list (`cd && nohup ... &`) makes
  bash fork a wrapper subshell that inherits sshd's stdout/stderr
  pipes (only the inner command's fds are redirected) and then waits
  on the never-exiting server, so sshd keeps the channel open even
  after the main remote shell exits.
- **Fix**: Split `cd` into its own statement and background only the
  simple command (`cd repo || exit 1; nohup bash 8__run_server.sh
  > log 2>&1 < /dev/null &`) — bash fork+execs it directly with every
  fd redirected, and ssh returns immediately.
- **Rule**: Never background a compound command inside a remote ssh
  command string; background a simple command with stdin, stdout, and
  stderr all redirected. (from ToDo 2026-07-29 ACT client automation,
  gh #112)

### G13. Concurrent async-inference sessions silently hijack each other

- **Problem**: while a verification harness restarted the policy
  server, another operator's `9__run_client_act.sh` run was starting
  concurrently; the harness killed that run's fresh server and
  streamer, the other run's readiness probe then attached its robot
  client to the harness's server, and both clients fed one server.
- **Cause**: the restart blocks (pkill server / pkill streamer / pkill
  tunnel) assume exclusive ownership of the async stack. There is no
  lock, and the server accepts any number of clients: `Ready()` from a
  second client resets server state under the first, and both share
  one observation queue. The only reason no foreign action reached the
  FR5 is the client-side stale-chunk filter (timesteps <=
  latest_action are dropped), which happened to discard the other
  client's chunks.
- **Fix**: stopped the harness immediately and verified quiescence
  (no local robot_client, server log idle) before continuing. No code
  guard exists yet.
- **Rule**: Never restart the policy server, tunnel, or streamer
  without first checking for a live session (`pgrep -af robot_client`
  locally, recent write activity on outputs/policy_server.log
  remotely); only one operator may run the async stack at a time.
  (from ToDo 2026-07-30 inference_latency, gh #114)

### G14. Async-client queue plot appears only after clean shutdown

- **Problem**: `--debug_visualize_queue_size=true` seemed to do
  nothing during or after `9__run_client_act.sh` runs.
- **Cause**: `visualize_action_queue_size` runs in `async_client()`'s
  finally block only after `stop()` (multi-second robot/camera
  teardown) plus receiver join, then blocks in `plt.show()`. A second
  Ctrl+C during that teardown, or an unsuppressed `cam.disconnect()`
  exception in `FairinoFollower.disconnect()`, kills the process
  before the window; the Tk window can also open behind the editor
  while the terminal looks hung.
- **Fix**: press Ctrl+C once and wait for the teardown; verified via
  `claude_test/debug_queue_plot_shutdown.py` (single SIGINT ->
  finally -> TkAgg window opens). PNG-save fix proposed in #118.
- **Rule**: Never expect the queue-size plot mid-run; after Ctrl+C,
  press nothing else until "Client stopped" or the plot window
  appears. (from ToDo 2026-07-30 queue plot diagnosis, gh #118)

---

### G14a. Episode timings are wrong at the recording boundary

- **Problem**: The last "attempt" of `task3_pi0_200` measured 9.8 s against a 33-45 s norm for that task, and was flagged as a quick return -- a failure mode it did not have.
- **Cause**: The recording is 1027 s long and that span runs 1017-1027 s. It was not a short attempt; the operator stopped recording on top of a normal one. Eight of eighteen recordings end mid-attempt and five begin mid-attempt, because recording start/stop is manual and independent of the robot.
- **Fix**: Flag any span touching either end of the recording (within 3 s) as `truncated`, keep it out of the duration statistics, mark it `+` in the report to show the value is a lower bound, and never let it be classified by duration. `task3_pi0_200` went from 33.4 +/- 9.0 s to 36.1 +/- 3.8 s once the artifact was removed.
- **Rule**: Always treat the first and last episode of a manually-started recording as censored data; never let a boundary-clipped duration feed a statistic or a duration-based classifier. (from ToDo#last)

### G15a. A short robot excursion is a failure mode, not noise

- **Problem**: The eval-video segmenter classified every away-from-home span under ~12 s as a "transit" (homing move) and dropped it, reporting 5 attempts for `task2_act_100.webm` when the operators had logged 10.
- **Cause**: Reasoned from the video alone that a span too short to reach the props could not be a rollout. It can: the policy leaves home, aborts, and drives straight back. The operators already had a name for it -- `갑자기 원점으로 돌아감` -- which appears four times in the `data.xlsx` notes for that one condition.
- **Fix**: Count every away-from-home span as an attempt and only *flag* the short ones (`kind=quick_return`). After the change 11 spans were detected against 10 logged tries, and all four `갑자기 원점으로 돌아감` notes lined up with flagged spans.
- **Rule**: Never let a detector discard a category it cannot explain; flag it and let the reviewer decide, because a dropped attempt is invisible in the output while a spurious one is not. Always read the operators' own note vocabulary before deciding what counts as signal.

### G15. Overhead eval video: motion energy cannot bound an attempt

- **Problem**: Segmenting `outputs/evaluation/*.webm` into attempts by frame-difference motion energy either merged four operator resets into one 134 s span or chopped single rollouts into fragments.
- **Cause**: Two false assumptions about the fixed overhead camera. The bottom-right of the frame is never quiet -- a bystander sits at the laptop for the whole session -- so an "operator intrusion" ROI fires constantly; and a policy that stalls mid-rollout goes quiet, so idle time is not a boundary either.
- **Fix**: Segment on deviation from the arm's parked home pose instead (`src/lerobot/scripts/eval_video_report.py`). Build the home template from the tight cluster of still frames -- a plain median blends home with working poses and destroys the contrast -- then take every away-from-home span as an attempt. Props are reset while the arm is parked, so resets fall outside attempts for free.
- **Rule**: Always segment robot episodes by pose state against a learned rest template, never by raw motion energy, when the camera also sees humans. (from ToDo#last)

### G16. Otsu is the wrong splitter for a dozen values

- **Problem**: Classifying ~11 span durations into short transits and real rollouts put an 11.75 s span in the rollout group while every 11.0 s span went to transits, even though the nearest real rollout was 57.8 s.
- **Cause**: Otsu maximises between-class variance over a 256-bin histogram. With 11 points that variance is flat across the whole empty gap, so `argmax` returns a bin edge decided by the bin width rather than by the data.
- **Fix**: Split at the widest *ratio* gap in the sorted values (`sorted[1:] / sorted[:-1]`), cut at the geometric midpoint, and refuse to split when the widest gap is under 2x -- treating everything as a rollout is far safer than dropping one from the report.
- **Rule**: Never use histogram thresholding on a handful of samples; split at the largest gap in the sorted values and require that gap to be decisive. (from ToDo#last)

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

### Q15b. Cutting GPU_NUMBER while holding BATCH_SIZE raises per-GPU batch

- **Problem**: With 3 of 4 B200s enumerating, `7__train_pi0_task2_b200.sh`
  (`GPU_NUMBER=3`, `BATCH_SIZE=704`) died before step 0 with
  `No valid triton configs. OutOfMemoryError: out of resource:
  triton_per_fused_..._mean_mul_pow_rsqrt_... Required: 294976
  Hardware limit: 232448` -- the Q15 signature, on pi0 rather than pi05.
  The same script had run fine for months.
- **Cause**: `BATCH_SIZE` in these scripts is the GLOBAL batch;
  `--batch_size` is passed as `BATCH_SIZE / GPU_NUMBER`. Holding the
  global batch while dropping the GPU count therefore *raises* per-GPU
  batch, which is the quantity the shared-mem ceiling binds on. The
  same `BATCH_SIZE=704` gives 117/GPU on 6 GPUs (OK), 176/GPU on 4
  (OK), and 234/GPU on 3 (fails). Nothing about the box changed --
  only the divisor.
- **Fix**: Scale `BATCH_SIZE` with `GPU_NUMBER` to hold per-GPU at the
  validated rung (pi0: 176*3 = 528; pi05: 152*3 = 456), and sqrt-rescale
  the LR for the new global batch. Verified: pi05 at 152/GPU on 3 GPUs
  peaks at 131022 MiB vs 131024 MiB on 4 GPUs -- per-GPU batch alone
  determines compile/VRAM behaviour; GPU count does not.
- **Rule**: Always re-derive `BATCH_SIZE` when changing `GPU_NUMBER` --
  per-GPU batch, not global batch, is what hits the ceiling. Note the
  global batch then changes too, so an LR rescale is required and the
  run is no longer directly comparable to a differently-batched one.
  (from ToDo#50, gh #102)

### Q15c. The gemma shared-mem ceiling is policy-specific, not shared

- **Problem**: Q15 records the shared-mem failure as a pi05 trait
  ("pi05's gemma RMSNorm"), which reads as though pi0 is exempt or
  shares the 160 threshold. Both readings are wrong and led to a bad
  prediction that pi0 would fail above ~160.
- **Cause**: pi0 and pi05 share the gemma backbone and therefore the
  same failing kernel and the same 232448 B per-SM limit, but the
  per-block shared memory scales with model width, so the batch at
  which they cross the limit differs.
- **Fix**: Read the thresholds off this repo's own run logs
  (`grep "Effective batch size" outputs/train/**/*.log`, then check
  whether any step lines follow). Measured on B200:
  pi0 -> 160 OK, 176 OK, 192/208/224/234/240 FAIL;
  pi05 -> 152 OK, >=160 FAIL. (The one failing pi0 176 run predates
  `compile_mode=default` and is Q14, not this.)
- **Rule**: Never carry a batch ceiling across policies -- pi0's 176
  does not license 176 for pi05. Treat each policy's largest
  step-producing rung in the logs as its ceiling. (from ToDo#50, gh #102)

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

### Q17. `push_to_hub` leaves stale shards after `delete_episodes` repacks `meta/episodes`

- **Problem**: After `lerobot-edit-dataset delete_episodes` (removed
  ep119) then `push_to_hub`, the Hub `info.json`/README correctly read
  119 episodes, but reloading `meta/episodes/*.parquet` from the Hub
  gave 229 rows with ep119 still present.
- **Cause**: `delete_episodes` re-packs episode metadata through a
  size-based writer, collapsing 12 small shards (10 rows each) into a
  single `file-000.parquet` (119 rows). `push_to_hub` uses
  `HfApi.upload_folder`, which OVERWRITES matching paths but NEVER
  deletes Hub files absent locally — so the 11 orphan shards
  `file-001..011` (eps 10..119, incl. 119) survived and double-counted.
  Data/video files kept identical names, so only `meta/episodes`
  orphaned.
- **Fix**: After the push, diff Hub file list vs local
  (`HfApi.list_repo_files` vs local glob) and delete orphans in one
  `HfApi.create_commit` with `CommitOperationDelete`. Re-verify by
  reloading the Hub `meta/episodes` row count.
- **Rule**: Always reconcile the remote file list against local after a
  `push_to_hub` that follows a structural edit (delete/split/merge);
  `upload_folder` is overwrite-only, so repacked shard counts leave
  orphans. (from ep119 deletion, 2026-06-30, gh #98)

### Q18. Long `JOB_NAME` overflows the Hub's 96-char repo name limit

- **Problem**: `7__train_act_task2_h200.sh` ran all 100K steps and saved
  every checkpoint, then the end-of-training `push_model_to_hub` died
  with `HFValidationError: ... the maximum length is 96`. Nothing was
  uploaded, and the training log ended cleanly on `End of training`, so
  the failure was invisible from the log alone.
- **Cause**: `--policy.repo_id` was `${JOB_NAME}_act_h200_model`. The
  task2 `JOB_NAME` (a full sentence-length task description) is 86
  chars, and the `_act_h200_model` suffix pushed it to 101 — past the
  Hub's 96-char repo name maximum. The shorter task3 job names had
  always fit, so the suffix looked safe.
- **Fix**: Dropped the `_model` suffix (`${JOB_NAME}_act_h200`, 86
  chars) and uploaded the intact local
  `checkpoints/last/pretrained_model` with `HfApi.upload_folder`. No
  retraining needed — all 7 files verified byte-identical to local.
- **Rule**: Always budget the `repo_id` suffix against a 96-char cap
  when `JOB_NAME` encodes the task description; push failures land on
  stderr, so a clean training log never proves the model reached the
  Hub — verify with `list_models`. (from task2 h200 push, 2026-07-21,
  gh #103)

### Q19. transformers v5 dropped `create_causal_mask(cache_position=)`

- **Problem**: Pi0 async inference never produced a single action
  chunk. The client looked hung on `[server] Starting receiver`, but
  the server was crashing on observation #0 with
  `Error in StreamActions: create_causal_mask() got an unexpected
  keyword argument 'cache_position'` and the client kept reconnecting,
  re-logging that line at ~10 Hz.
- **Cause**: `pi_gemma.py` passed `cache_position=` to
  `transformers.masking_utils.create_causal_mask`. Transformers removed
  that parameter partway through the v5 series (v5.3.0 accepts it,
  v5.11.0 does not, deriving query positions from `position_ids`), and
  upstream commit f0d2b37b bumped the pin to `transformers>=5.3.0,<6.0.0`
  without updating this call site. Both versions satisfy the pin, so the
  breakage depends on which v5 the box happens to have.
- **Fix**: Probe `inspect.signature(create_causal_mask)` once at import
  and pass `cache_position` only when the installed version accepts it;
  `position_ids` was already supplied, so behaviour is unchanged either
  way. Deleting the argument outright would have broken v5.3 boxes.
- **Rule**: Never assume a kwarg survives inside a permitted version
  range -- when a pin spans a major series (`>=5.3,<6`), probe the
  signature instead of matching the version installed on one machine.
  (from Pi0 inference crash, 2026-07-31, gh #122)

---

### Q20. VLA constructors random-init billions of parameters before loading

- **Problem**: Starting the async policy server with a Pi0 checkpoint
  took about three minutes (`Time taken to put policy on cuda:
  111.37 s`), paid again on every run because the client restarts the
  server. Measured split: imports 6.7 s, constructor 120.7 s,
  8.9 GB safetensors read 1.3 s, `.to(cuda)` ~0 s.
- **Cause**: `PI0Policy.from_pretrained` calls `cls(config)` before
  reading the checkpoint, and `PaliGemmaWithExpertModel.__init__`
  random-initializes 4.03 B parameters in float32 on the CPU, casts
  the graph to bfloat16, and uploads ~9 GB to the GPU -- all of it
  overwritten by `load_state_dict` moments later. Suppressing only
  the random-init math still cost 54 s (the float32 allocation plus
  the bfloat16 copy).
- **Fix**: Build the skeleton under
  `accelerate.init_empty_weights(include_buffers=False)` and load with
  `load_state_dict(..., assign=True)`, reading the safetensors
  straight onto the target device. `nn.Module.to` must be neutralized
  during construction, since both policy constructors end with a
  device move and a dtype cast that cannot touch meta tensors.
  `include_buffers=False` is required: the six non-persistent buffers
  (`position_ids`, `embed_scale`, rotary `inv_freq` /
  `original_inv_freq`) never appear in a checkpoint, so nothing would
  materialize them. Result: 103.2 s -> 2.0 s, all 784 tensors
  bit-identical.
- **Rule**: Never let a multi-billion-parameter model initialize real
  weights it is about to overwrite -- build on meta, load with
  `assign=True`, and assert afterwards that nothing is still on meta.
  (from Pi0 load time, 2026-08-04, gh #123)

---

### Q21. `assign=True` takes the dtype from the checkpoint, not the config

- **Problem**: The meta-init fix above was weight-identical on the Hub
  checkpoints but would have silently made every fine-tune float32,
  ignoring `--policy.dtype=bfloat16` and doubling model memory.
- **Cause**: `load_state_dict(assign=True)` replaces the parameter
  object, so the loaded tensor's dtype wins. `models/pi0_base_v051compat`
  and `models/pi05_base_v051compat` are stored as pure float32 (777 and
  812 F32 tensors); the old copy-based load quietly cast them into the
  constructor's bfloat16 parameters instead.
- **Fix**: `apply_precision_layout(precision)` re-derives the dtype
  layout after loading. It must cast only where the dtype is *wrong*:
  re-running the constructor's `to(bfloat16)` + float32-keep-list would
  round-trip the vision path through float32 -> bfloat16 -> float32 and
  lose precision on real weights (harmless on the random weights it was
  written for). Verified bit-exact on both a float32 base checkpoint and
  a mixed-dtype Hub checkpoint.
- **Rule**: Always re-apply the configured dtype layout after an
  `assign=True` load, and cast only tensors whose dtype differs.
  (from Pi0 load time, 2026-08-04, gh #123)

---

### Q22. The pi0 vision-tower remap was never ported to pi05

- **Problem**: Loading `models/pi05_base_v051compat` failed on every
  vision-tower key. Before the loud-failure change of Q20 it did not
  fail at all -- `from_pretrained` caught the error, printed a single
  `Warning: Could not load state dict:` line, and returned a pi05
  whose vision encoder was still randomly initialized.
- **Cause**: The checkpoint stores 437 keys under the nested
  `vision_tower.vision_model.*` naming; transformers 5.11 flattens the
  model side to `vision_tower.*` (435 keys). pi0 got a remap for this
  in gh #88, but `PI05Policy._fix_pytorch_state_dict_keys` was never
  updated, so pi05 kept hitting the original bug in silence.
- **Fix**: Port the pi0 remap verbatim, including its guard (rewrite
  only when the checkpoint naming is absent from the model and the
  rewritten naming is present, so matching checkpoints pass through
  untouched). `models/pi05_base_v051compat` then loads with 0 missing
  and 0 unexpected keys.
- **Rule**: Always apply a checkpoint-compatibility fix to every
  policy that shares the backbone -- pi0 and pi05 wrap the same
  PaliGemma and diverge only by copy-paste drift.
  (from Pi0 load time, 2026-08-04, gh #123)

### Q23. `hf models list` pages at 30 results and truncates silently

- **Problem**: Building the Pi0.5 client checkpoint list from
  `hf models list --author coport-uni` returned 30 repos and stopped
  mid-2026-07, hiding every checkpoint pushed after that date --
  including the task2 200-episode Pi0.5 model the script already
  pinned.
- **Cause**: The CLI applies a default page size (30) and prints the
  first page only. Nothing in the output marks the cut, so the listing
  reads as complete.
- **Fix**: Pass an explicit `--limit` (`--limit 500`), which returned
  42 repos; cross-check the count with `wc -l` before treating a
  listing as exhaustive.
- **Rule**: Always pass `--limit` to `hf models list` / `hf datasets
  list` when the result feeds a "every checkpoint we have" claim --
  a short listing is a page boundary, not an empty shelf.
  (from Pi0.5 client checkpoint list, 2026-08-05, gh #125)

### Q24. A moved PTZ preset breaks every checkpoint identically

- **Problem**: task1 Pi0.5 rollouts failed with the same symptom for
  both the final and the mid-training (`_half`) checkpoint, while
  task2 and task3 Pi0.5 rollouts an hour earlier were fine.
- **Cause**: `1__setup_camera.sh` calls
  `goto_preset_all_ipcamera.py`, which sends every HikVision PTZ to
  "Preset 1". Re-running it at 22:14 moved the cameras off the
  framing the datasets were recorded at -- every training frame of
  `top_left`/`top_right` shows the robot's energy chain and a wider
  field, both live views were tighter with the chain out of frame.
  The policy was fed a viewpoint it never saw in training.
- **Fix**: Restore the recording framing (re-teach "Preset 1"), then
  verify by decoding frame 0 of
  `videos/observation.images.<cam>/chunk-000/file-000.mp4` from the
  dataset and putting it side by side with a live grab off
  `/dev/video18/19`.
- **Rule**: Never trust identical symptoms across different
  checkpoints to be a policy problem -- when weights change and
  behaviour does not, compare the observation against the training
  distribution first. (from task1 Pi0.5 rollout, 2026-08-06, gh #135)

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

### W7. Same fix committed from two machines looks like divergence

- **Problem**: Local `main` reported "ahead 1, behind 13" and
  `git cherry` flagged the commit as unmerged, but the merge
  produced conflicts whose two sides were the same change.
- **Cause**: The identical fix was committed on two machines. The
  added lines match byte for byte; only the surrounding context
  differs, so the patch-ids differ and git cannot detect the
  duplicate.
- **Fix**: Diff the two commits with `diff <(git show A --format=)
  <(git show B --format=)` -- when only index/`@@` headers and
  context lines differ, resolve every conflict to the upstream
  side, then confirm `git diff --stat origin/main` is empty before
  committing the merge.
- **Rule**: Always prove a "divergent" commit is not a duplicate
  before merging, and always verify the merged tree against
  upstream instead of trusting a clean conflict resolution.
  (from ToDo#109)

### W8. A detached HEAD hides which branch work belongs to

- **Problem**: The working copy sat on a detached HEAD that was
  already an ancestor of `origin/main`, so uncommitted work had no
  branch to land on and `git checkout main` would have carried the
  changes onto an unrelated line of history.
- **Cause**: A previous session checked out a commit rather than a
  branch and left it that way.
- **Fix**: `git checkout -b <topic>` at the detached commit, commit
  the working tree there, then merge that branch into `main` after
  `main` is synced with upstream.
- **Rule**: Always run `git status` before starting git work; when
  it says "Not currently on any branch", create a branch before
  committing anything. (from ToDo#109)

### W9. A pushed secret needs rotation, not just history rewrite

- **Problem**: An OpenSSH private key (`outputs/appeal_test_key`)
  rode a bulk `outputs/` snapshot commit onto the public GitHub
  repo.
- **Cause**: `.gitignore` excluded only specific `outputs/`
  subtrees (datasets, smoke_logs, checkpoints), so a stray key
  dropped at the `outputs/` top level was swept up by the snapshot
  commit; no rule blocked key-like filenames.
- **Fix**: `git rm --cached` + `commit --amend` +
  `push --force-with-lease` removed the file from `main` (the
  W3 rewrite pattern), and repo-wide ignore rules for `*.pem`,
  `*_key`, `*.key`, `id_rsa*`, `id_ed25519*` now block a repeat;
  the old commit stayed fetchable by SHA via the GitHub API, so
  the key itself must be rotated.
- **Rule**: Never treat history rewrite as containment for a
  pushed secret — GitHub keeps the old commit reachable by SHA
  until garbage collection, so always rotate the credential and
  treat it as compromised. (from ToDo#123)

### W10. `rebase --autostash` yanks the user's live IDE edits mid-task

- **Problem**: A push was rejected (the appeal box had pushed first),
  so `git rebase --autostash origin/main` ran. The autostash reverted
  the user's uncommitted client-script edits while they were still
  editing in the IDE; they re-edited the reverted files, and
  `git rebase --continue` then died with "You must edit all merge
  conflicts and then mark them as resolved using git add" even though
  `git ls-files -u` was empty.
- **Cause**: Two compounding traps. The autostash window is not
  atomic -- anyone editing the worktree during the rebase edits a
  reverted file. And `rebase --continue` refuses on *unstaged* changes
  of any kind, reporting them with the merge-conflict wording; the
  message names the wrong problem.
- **Fix**: Copy every dirty file to the scratchpad, `git checkout --`
  them so the worktree matches the index, `git rebase --continue` (the
  autostash then applies cleanly), and restore the backups afterwards
  -- keeping the newer worktree copy where it superseded the autostash
  and the autostash copy where the user had not re-made the edit.
- **Rule**: Never trust `git ls-files -u` to explain a
  `rebase --continue` refusal -- check `git diff --stat` for unstaged
  work first; and always snapshot dirty files before an autostash
  rebase, because the user may be editing them at that moment.
  (from Pi0.5 client checkpoint list, 2026-08-05, gh #125)

---

### W11. An LFS pattern never converts already-committed blobs

- **Problem**: `outputs/evaluation/` held three webm recordings as raw
  git blobs; adding `*.webm` to `.gitattributes` left them raw, so the
  directory now mixes raw-blob and LFS-backed files.
- **Cause**: `.gitattributes` filters apply when a file is staged, not
  retroactively. Objects already in history are untouched.
- **Fix**: Accepted the mix. Converting needs `git lfs migrate import`
  plus a force-push, which rewrites every downstream commit.
- **Rule**: Always add the LFS pattern BEFORE the first large file of
  that type is committed; once it is in history only a rewrite fixes it.
  (from ToDo#131)

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

### E13. Container swap moves the mount path; it does not delete the env

- **Problem**: After a container migration `conda` was gone from PATH
  and every root in the portable probe block (E5) was missing, so the
  env looked destroyed and a 13 GB reinstall looked necessary.
- **Cause**: The env was intact on the persistent disk the whole time.
  Only the mount path moved (`/NHNHOME/workspace` ->
  `/NHNHOME/WORKSPACE/26msit002_E/appeal_workspace`). Conda bakes its
  install prefix into every console script, so the move broke 134
  shebangs in `envs/lerobot/bin` (including `conda` itself) and the
  editable-install pointer `__editable__.lerobot-0.5.1.pth`. The
  interpreter binary still ran fine -- only the prefix-baked wrappers
  failed, which is why it presented as "everything is gone".
- **Fix**: `ln -s <new-root> <old-path>` restored conda, all 134
  scripts, the editable import, and all 5 training scripts that
  hard-code the old root -- unmodified, in one command.
- **Rule**: Never conclude a conda env is lost because `conda` is not
  found; first check whether the install still exists at a moved path
  and test the real interpreter (`envs/<name>/bin/python3.x`) directly.
  Prefer a symlink restoring the old prefix over relocating a
  prefix-baked env. (from ToDo#49, gh #101)

### E14. Only `/NHNHOME` persists; `$HOME` and `/tmp` are container overlay

- **Problem**: The 2026-07-01 rebuild installed conda to `~/miniconda3`
  and the next container swap wiped it again. The same swap destroyed
  the HF cache (`~/.cache/huggingface`), the HF/wandb/gh credentials,
  and `gh` itself.
- **Cause**: `/home` and `/tmp` live on the container overlay; only
  `/NHNHOME` (the mounted nvme) survives a swap.
- **Fix**: Install envs and caches under `/NHNHOME`. `HF_HOME` is now
  redirected to `/NHNHOME/workspace/sungwoo/hf_cache` in the 5 B200
  training scripts, alongside the existing inductor/triton redirect
  (E11). Note `hf auth login` writes its token to `~/.cache` unless
  `HF_HOME` is exported first, so the token must be placed on the
  persistent disk explicitly.
- **Rule**: Always install environments, caches, and tokens under
  `/NHNHOME`; never `$HOME` or `/tmp`. Losing an overlay HF cache turns
  a silent dependency on a gated repo into a hard training failure
  (E15). (from ToDo#49, gh #101)

### E15. pi0/pi05 need the GATED paligemma tokenizer at startup

- **Problem**: pi05 training died before step 0 with
  `Failed to instantiate processor step 'tokenizer_processor' ...
  You are trying to access a gated repo`, even though the FR5 dataset
  is public and the pi05 weights are pinned locally.
- **Cause**: pi0/pi05 build `tokenizer_processor` from
  `google/paligemma-3b-pt-224`, a gated repo requiring an authenticated
  token whose account accepted the licence. It had always resolved
  silently from the overlay HF cache; once that cache was wiped (E14)
  the hidden network dependency surfaced. The pinned
  `models/pi05_base_v051compat` snapshot carries weights but no
  tokenizer, so there is no local fallback.
- **Fix**: `hf auth login` with a licence-accepting account, with
  `HF_HOME` pointed at the persistent disk so the tokenizer and token
  survive the next swap.
- **Rule**: Always treat the paligemma tokenizer as a required gated
  dependency of pi0/pi05, not an implementation detail of the cache; a
  pinned local checkpoint does NOT make training offline-capable.
  (from ToDo#49, gh #101)

### E16. HWE kernel upgrade wedges apt when a DKMS module cannot build

- **Problem**: `1__setup_camera.sh` died at its `sudo apt-get install`
  line before ever reaching `modprobe`, so /dev/video18-20 were never
  created and downstream capture silently lost the loopback cameras.
- **Cause**: unattended HWE upgrade pulled kernel 7.0.0-28, whose
  postinst fails because v4l2loopback 0.12.7 does not compile against
  kernel 7.0 headers (`v4l2_fh_add`/`v4l2_fh_del` gained a
  `struct file *` parameter). The kernel packages sit half-configured
  (`iF`), so EVERY later apt run exits 100, and the script's
  `set -euo pipefail` turns that unrelated failure into an abort.
- **Fix**: purge `linux-image/-headers-7.0.0-28-generic` (pauses HWE
  kernel updates until v4l2loopback catches up). Short-term: modprobe
  manually — the running 6.17.0-40 kernel has a good module build —
  then use the script's `stream` mode.
- **Rule**: Never make a routine launch script depend on `apt-get`
  succeeding at run time; guard installs behind a "package missing"
  check, and after any kernel upgrade verify DKMS built the module for
  the new kernel before rebooting. (from ToDo#107, gh #104)

### E17. GPU-hub exposes only an HTTPS path proxy; gRPC needs an SSH tunnel

- **Problem**: the FR5 control PC could not reach the async-inference
  policy server running inside the NHN GPU-hub container. The only
  address the console hands out is an HTTPS entry point with a path
  token, `https://cl1.gpuhub.nhncloud.com:50030/FiF42P8A3y/`, and
  putting it in `--server_address` cannot work.
- **Cause**: two independent blockers. The client passes that value
  straight to `grpc.insecure_channel()`, which parses `host:port` only,
  and even on a TLS channel gRPC carries the service/method in the
  HTTP/2 `:path` (`/async_inference.AsyncInference/GetActions`), so the
  proxy's `/FiF42P8A3y/` prefix has nowhere to live. Separately the
  container's own egress to that host is filtered — TCP 50030 and 443
  both time out while `huggingface.co` returns 200 — so the URL is an
  inbound path for external clients only.
- **Fix**: tunnel the port instead of routing gRPC through the proxy.
  From the FR5 PC, `ssh -N -C -i <key> -p 45406 -L 17040:127.0.0.1:17040
  appeal@59.150.32.1`, then dial `127.0.0.1:17040`. Confirmed working:
  the robot PC established a gRPC connection to the container's server.
- **Rule**: Always give `--server_address` a bare `host:port`; when the
  only public entry point is an HTTP path proxy, forward the port over
  SSH rather than trying to route gRPC through it. (from ToDo#114,
  gh #110)

### E18. Server-side code edits are inert until the appeal-box clone pulls

- **Problem**: a `policy_server.py` log-message fix committed on the
  FR5 control PC did not show up in the streamed `[server]` log; the
  remote server still printed the old text.
- **Cause**: the async-inference server runs from a *separate clone*
  of this repo on the appeal box
  (`/NHNHOME/WORKSPACE/26msit002_E/appeal_workspace/sungwoo/FR5ControllerVLA`).
  `9__run_client_act.sh` restarts that server over SSH but never
  syncs its code, so local commits change nothing remotely until the
  clone pulls.
- **Fix**: push to origin/main, then `ssh appeal ... git pull
  --ff-only` (after `git status --porcelain` confirms the clone is
  clean) before restarting the server.
- **Rule**: Always push and pull the appeal-box clone before
  expecting any `src/lerobot/async_inference/` server-side change to
  take effect at inference time. (from ToDo 2026-07-30 log
  streaming, gh #113)

---

### E19. git-lfs is absent on the controller; install it user-space

- **Problem**: `git lfs` was not a git command on the control machine,
  and `sudo apt install git-lfs` could not run -- sudo needs a password
  that a non-interactive session cannot supply.
- **Cause**: The box ships git without the LFS extension, and package
  installs require an interactive sudo.
- **Fix**: Untarred the upstream `git-lfs-linux-amd64-v3.7.0` release
  into `~/.local/bin`, then `git lfs install --local` to scope hooks to
  this repository rather than touching `~/.gitconfig`.
- **Rule**: Always prefer the user-space release tarball plus
  `git lfs install --local` when sudo is unavailable; never assume the
  controller has LFS just because the repo declares LFS patterns.
  (from ToDo#131)

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
