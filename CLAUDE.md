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
- Python conventions:

| Element    | Style        | Example               |
|------------|--------------|-----------------------|
| Variable   | `lower_case` | `joint_angle`         |
| Function   | `lower_case` | `send_action`         |
| Class      | `CamelCase`  | `FairinoFollower`     |
| Constant   | `lower_case` | `_settle_mid_s`       |
| Module     | `lowercase`  | `fairino_follower`    |

### Structure and Spacing
- **80-column limit** for new Fairino code (project-wide ruff is 110; keep new code at 80).
- One statement per line. Indent with 4 spaces (never tabs).
- Place operators on the **left side** of continuation lines.
- One space after commas, around `=` and comparison operators.

### Comments
- Write complete sentences. Only comment for **context** or **non-obvious choices** -- never restate what the code already says.
- Outdated comments are worse than none; keep them in sync.
- TODO format: `# TODO: (@owner) specific action -- reason why.`

### Language

- All code comments, docstrings, commit messages, documentation files (including README), **GitHub issues, and pull requests** must be written in **English**.

### Documentation
- All public functions and classes must have docstrings (PEP 257 / Google style).
- Docstrings state **what** and **why**, not **how** (the code shows how).
- Include `Args:`, `Returns:`, and `Raises:` sections when applicable.

---

## Debug File Management

All debug, exploratory, and throwaway test scripts must be saved in `claude_test/`, **not** in `tests/`.

### Rules

| Location        | What goes there                                      |
|-----------------|------------------------------------------------------|
| `tests/`        | Production-quality tests that are part of CI/CD.     |
| `claude_test/`  | Debug scripts, one-off experiments, diagnostic code. |

### When writing debug code

1. Create the file directly in `claude_test/` (e.g., `claude_test/debug_servo_timing.py`).
2. Add a one-line docstring at the top explaining the purpose.
3. If the debug script leads to a real fix, move the relevant parts into a proper test under `tests/` and delete or archive the debug version.

### README

`claude_test/README.md` is the index. When adding a new debug file, add a row to the table in that README describing what the file does and what was learned.

---

## Task Management

> **MANDATORY**: This workflow applies to **every task without exception**, regardless of size or complexity. No task may begin without writing `ToDo.md` and creating a GitHub issue via `gh`. Skipping any step is not allowed.

### Rules

1. **Write ToDo.md**: For every task requested by the user, create a `ToDo.md` file and confirm the contents with the user before starting work.
2. **Accumulate ToDo.md**: Do not overwrite previous entries in `ToDo.md`. Always **append** new tasks below existing ones so that the file serves as a cumulative command history for Claude's actions.
3. **Register GitHub issues**: When possible, use the `gh` CLI to register the Todo list and details as a GitHub issue.

### Command Input Validation

Before writing ToDo.md, the following two checks must be performed:

1. **Is the command explicit?**: If the request is ambiguous or open to interpretation, do not start work. Instead, ask the user for specifics:
   - What is being changed? (target)
   - How is it being changed? (method)
   - Why is it being changed? (purpose)
2. **Are there reference materials?**: Check whether related PDFs, websites, or documents exist. If so, review them before incorporating into the work.

> Do not proceed if either check is not satisfied.

### Workflow

1. Receive the user's task request and **validate the command input**.
2. Once validated, organize the task list in `ToDo.md`.
3. Get the user's confirmation on the `ToDo.md` contents.
4. Once confirmed, create a GitHub issue via `gh issue create`.
5. Check off completed items in `ToDo.md` as work progresses.
6. Update the GitHub issue via `gh issue edit` for completed items.
7. **Commit and push** changes after every user command is completed.

> **Reminder**: Steps 2 (`ToDo.md`) and 4 (`gh issue create`) are **non-negotiable**. Every task must have a corresponding `ToDo.md` entry and a GitHub issue before any work begins.

---

## Testing Rules

Tests exist to verify the **correctness and quality** of code. Code quality must never be sacrificed just to pass tests.

### Rules

1. **No magic numbers**: Do not use arbitrary numbers or values directly to pass tests. All values must be defined as meaningful constants or variables.
   ```python
   # Bad: passing tests with magic numbers
   def calculate_area(radius):
       return 3.14 * radius * radius  # Why 3.14?

   # Good: use meaningful constants
   import math

   def calculate_area(radius):
       return math.pi * radius * radius
   ```

2. **No hardcoding**: Do not hardcode values to match expected test results. Code must work through correct logic, not through branches or fixed values tailored to specific inputs.
   ```python
   # Bad: hardcoded to match test inputs
   def convert_temperature(celsius):
       if celsius == 100:
           return 212
       if celsius == 0:
           return 32
       return celsius * 1.8 + 32

   # Good: correct logic implementation
   def convert_temperature(celsius):
       return celsius * 1.8 + 32
   ```

3. **Code quality first**: Prioritize readability, maintainability, and correctness over whether tests pass. If a test fails, fix the logic correctly rather than tricking the test.

---

## Linting

All Python code must pass **Ruff** checks before committing.

### Rules

1. **Line length**: 110 columns project-wide (`ruff.toml`); **80 columns** for new Fairino code.
2. **Run on every commit**: Before committing, run:
   ```bash
   ruff check <file>.py
   ruff format --check <file>.py
   ```
3. **Fix before committing**: If either command reports errors, fix them before proceeding. Use `ruff format <file>.py` to auto-format.

---

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
- `robots/fairino_follower/fairino_follower.py` — Robot class, uses ServoJ via direct XMLRPC (`self._rpc.robot.ServoJ(...)`)
- `robots/fairino_follower/config_fairino_follower.py` — Config: IP, joint limits, control_hz (default 20)
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
