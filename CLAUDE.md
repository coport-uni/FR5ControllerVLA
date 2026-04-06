# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

A fork of [HuggingFace LeRobot](https://github.com/huggingface/lerobot) (v0.5.1) with Fairino FR5 6-DOF collaborative robot integration for VLA (Vision-Language-Action) research. The Fairino-specific code lives alongside the standard LeRobot framework.

## Build & Test Commands

```bash
# Install (editable, with test + dev extras)
pip install -e ".[test,dev]"

# Run all tests
pytest tests -vv

# Run a single test file
pytest tests/test_fairino_joint_move.py -vv

# Lint & format
ruff check src/                    # lint
ruff check src/ --fix              # lint with auto-fix
ruff format src/                   # format

# Pre-commit (runs ruff, typos, mypy, bandit, gitleaks)
pre-commit run --all-files

# Fairino hardware test (requires robot at 192.168.58.2)
.venv/bin/python tests/test_fairino_joint_move.py --ip 192.168.58.2
```

**Ruff config:** line-length=110, target=py312, select: E/W/F/I/B/C4/T20/N/UP/SIM. `__init__.py` files ignore F401/F403.

## Code Style: MIT Code Convention

All **new code** in this repository follows the [MIT CommLab Coding and Comment Style](https://mitcommlab.mit.edu/broad/commkit/coding-and-comment-style/). Key rules:

### Naming
- **Variables and classes** are nouns; **functions** are verbs.
- Names must be pronounceable. Name length is proportional to scope.
- Avoid abbreviations unless self-explanatory. If unavoidable, define them in a comment.
- Python typography: `lower_case` for variables/functions/constants, `CamelCase` for classes, `lowercase` for modules.

### Structure and Spacing
- **80-column limit** for new Fairino code (project-wide ruff is 110; keep new code at 80).
- One statement per line. Indent with 4 spaces (never tabs).
- Place operators on the **left side** of continuation lines.
- One space after commas, around `=` and comparison operators.

### Comments
- Write complete sentences. Only comment for **context** or **non-obvious choices** -- never restate what the code already says.
- Outdated comments are worse than none; keep them in sync.
- TODO format: `# TODO: (@owner) specific action -- reason why.`

### Documentation
- All public functions and classes must have docstrings (PEP 257 / Google style).
- Docstrings state **what** and **why**, not **how** (the code shows how).

### Debug and Test Files
- All debug/exploratory test scripts go in `claude_test/` (not `tests/`).
- Only production-quality tests belong in `tests/`.

## Architecture

### Core Abstractions (all in `src/lerobot/`)

**Robot** (`robots/robot.py`): Abstract base class. Every robot implements `connect()`, `disconnect()`, `get_observation() -> dict`, `send_action(dict) -> dict`. Observations and actions are flat dicts with keys like `"joint1.pos"`, `"observation.images.cam0"`.

**Teleoperator** (`teleoperators/teleoperator.py`): Abstract base class for input devices. Implements `connect()`, `disconnect()`, `get_action() -> dict`, `send_feedback(dict)`.

**Config registration**: Configs use `@RobotConfig.register_subclass("name")` (or `TeleoperatorConfig`). Factory functions `make_robot_from_config()` / `make_teleoperator_from_config()` dispatch by `config.type` string.

**Processor pipeline** (`processor/`): Chains of `ProcessorStep` that transform data between teleop/policy and robot. Default pipeline is identity. Steps exist for normalization, delta-to-absolute conversion, image transforms.

**Teleoperate loop** (`scripts/lerobot_teleoperate.py`):
```
obs = robot.get_observation()
teleop.send_feedback(obs)           # sync teleop state with robot
raw_action = teleop.get_action()
action = teleop_processor(raw_action, obs)
robot_action = robot_processor(action, obs)
robot.send_action(robot_action)
```

### Fairino FR5 Integration

**Communication**: XMLRPC on port 20003 (commands) + TCP on port 20004 (state feedback at ~50Hz). The SDK class is `RPC` in `robots/fairino/fairino/Robot.py`.

**Motion control**: Uses **ServoJ** (real-time servo), not MoveJ. MoveJ returns error 101/154 on this firmware. ServoJ requires:
1. `ServoMoveStart()` before first command
2. `robot.ServoJ(joint_pos, axis_pos, acc, vel, cmdT, filterT, gain)` — 7 params, no `id` arg (firmware V3.9.1 quirk)
3. `ServoMoveEnd()` on disconnect

**Init sequence** (in `connect()`): `ServoMoveEnd()` (cleanup) → `RobotEnable(0)` → `ResetAllError()` → `RobotEnable(1)` → `Mode(0)` → `ServoMoveStart()`. This handles stale servo sessions and error states.

**Key files**:
- `robots/fairino/fairino_follower.py` — Robot class, uses ServoJ via direct XMLRPC (`self._rpc.robot.ServoJ(...)`)
- `robots/fairino/config_fairino_follower.py` — Config: IP, joint limits, control_hz (default 20)
- `teleoperators/keyboard/teleop_keyboard_fairino.py` — Keyboard teleop, outputs **absolute** positions (tracks internal target state, initialized via `send_feedback`)
- `scripts/teleop_fairino.py` — Standalone teleop CLI with terminal display

**Units**: All joint values in **degrees** (Fairino's native unit). Joint limits: J1,5,6=±175°, J2=[-265,85]°, J3=±160°, J4=[-265,85]°.

### CLI Entry Points

```bash
lerobot-teleoperate     # Generic teleoperation (works with --robot.type=fairino_follower --teleop.type=keyboard_fairino)
lerobot-train           # Policy training
lerobot-eval            # Policy evaluation
lerobot-record          # Dataset recording
lerobot-calibrate       # Motor calibration
```

## Adding a New Robot

1. Create `robots/my_robot/` with config class (`@RobotConfig.register_subclass`) and robot class (inherits `Robot`)
2. Add factory case in `robots/utils.py:make_robot_from_config()`
3. Import in `robots/__init__.py` and `scripts/lerobot_teleoperate.py`
4. Properties `observation_features` and `action_features` must work when disconnected (used for dataset schema)
