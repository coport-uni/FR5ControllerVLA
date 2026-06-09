# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

A fork of [HuggingFace LeRobot](https://github.com/huggingface/lerobot) (v0.5.1) with Fairino FR5 6-DOF collaborative robot integration for VLA (Vision-Language-Action) research. The Fairino-specific code lives alongside the standard LeRobot framework.

## Rule Priority

This project-level `CLAUDE.md` takes precedence over the global [CommonClaude](https://github.com/coport-uni/CommonClaude) ruleset. Specific rules beat general ones — when a conflict arises, the more-specific context wins (e.g. this project's 110-column ruff limit overrides CommonClaude's global 80-column rule).

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

Example:
```python
def send_action(joint_pos: list[float]) -> dict:
    """Stream a servo joint command to the robot.

    The command is non-blocking and returns once the controller
    acknowledges receipt -- it does not wait for motion to finish.

    Args:
        joint_pos: Six target joint angles in degrees, ordered
            from base (J1) to flange (J6).

    Returns:
        Controller acknowledgement payload with keys
        ``"errcode"`` and ``"timestamp"``.

    Raises:
        ConnectionError: If the controller socket has dropped.
        ValueError: If ``joint_pos`` is not exactly six values.
    """
```

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
7. **Commit and push** changes after every user command is completed; every commit follows the Conventional Commits format (see Commit Messages below).

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

1. **Line length**: 110 columns project-wide (configured in `pyproject.toml`, which is this project's override of CommonClaude's global 80-column rule). Keep **new Fairino code at 80 columns** to stay compatible with the upstream MIT convention.
2. **Run on every commit**: Before committing, run:
   ```bash
   ruff check <file>.py
   ruff format --check <file>.py
   ```
3. **Fix before committing**: If either command reports errors, fix them before proceeding. Use `ruff format <file>.py` to auto-format.

---

## Research Before Coding & MCP Servers

Before calling into an unfamiliar library, API, or CLI, verify its actual interface rather than guessing from memory. Three MCP servers are **mandatory** tools for this workflow: **Serena** for semantic code navigation, **Context7** for up-to-date library documentation, and **Fetch** for pulling in reference material from the web.

### Required MCP Servers

| MCP server | Purpose | When to use |
|------------|---------|-------------|
| **Serena** | Semantic, symbol-level code retrieval and editing (LSP-backed). | Exploring the codebase, locating symbols/definitions/references, and making precise edits — **before** falling back to plain text search or full-file reads. |
| **Context7** | Fetches current, version-accurate documentation for libraries, frameworks, and APIs. | Before writing or changing any code that calls an external library, framework, or API. |
| **Fetch** | Retrieves a web page and returns its content as Markdown. | Reviewing reference materials (spec pages, articles, docs not covered by Context7) cited in a task — see Task Management, Command Input Validation. |

### Rules

1. **Use Serena for code understanding and edits**: Prefer Serena's symbol-level tools (find symbol, find references, navigate definitions, targeted symbol edits) over raw text search or rewriting whole files. This keeps context focused and edits precise.
2. **Consult official documentation first via Context7**: When touching an unfamiliar or version-sensitive library/API, query Context7 for its current interface before coding.
3. **Pull reference material with Fetch**: When a task cites a URL, spec, or article, use Fetch to read it as Markdown before incorporating it. Fall back to plain web search only when neither Context7 nor Fetch yields the source.
4. **Search the repository** for prior implementations (via Serena) before writing new code against the same interface. The Fairino SDK (`robots/fairino_follower/fairino/`) and LeRobot base classes (`robots/robot.py`, `teleoperators/teleoperator.py`) already contain worked examples.
5. **Trust documentation over intuition**: when the docs disagree with the mental model, update the mental model.

### Setup

The servers are registered with Claude Code via `claude mcp add`. Example configuration:

```bash
# Serena — semantic code toolkit (run from the project root)
claude mcp add serena -- \
  uvx --from git+https://github.com/oraios/serena \
  serena start-mcp-server --context ide-assistant --project "$(pwd)"

# Context7 — up-to-date library documentation
claude mcp add context7 -- npx -y @upstash/context7-mcp

# Fetch — retrieve web pages as Markdown
claude mcp add fetch -- uvx mcp-server-fetch
```

Verify all three are connected with `claude mcp list` (or `/mcp` inside a session) before relying on them.

> **Sources**: [Serena](https://github.com/oraios/serena) · [Context7](https://github.com/upstash/context7) · [Fetch](https://github.com/modelcontextprotocol/servers/tree/main/src/fetch)

---

## Exceptions

The rules above are written for production code and CI tests. The following contexts receive formal waivers.

### `claude_test/` scripts

Scripts inside `claude_test/` are exempt from:

- The 80-column line limit (MIT Code Convention, Structure and Spacing).
- Mandatory docstrings on public functions and classes (MIT Code Convention, Documentation).

Rationale: `claude_test/` is a scratch area for one-off diagnostics where strict readability conventions slow exploration. Anything later promoted into `tests/` must conform fully.

### One-off exploratory analysis

Exploratory or analysis scripts (typically under `claude_test/`) may use numeric literals directly, provided the file opens with a short intent comment explaining purpose and expected lifetime. This waiver does not apply to code under `tests/` or to production modules under `src/lerobot/`.

### `ToDo.md` checkbox updates

Marking completion checkboxes in `ToDo.md` (flipping `- [ ]` to `- [x]`, or appending a commit hash or issue link to a completed line) is permitted. The append-only rule in Task Management Rule 2 and the "do not modify `ToDo.md`" constraint in Learned Patterns Bootstrap forbid prose rewrites, reordering of entries, and deletion of historical items — not progress marking.

---

## Learned Patterns Reference

`LearnedPatterns.md` lives in the repository root. Treat it as part of the workflow — it captures lessons from past work so they can be reused rather than rediscovered.

### Rules

1. **Before drafting a `ToDo.md` entry**, read the sections of `LearnedPatterns.md` relevant to the new task. Relevance can be filtered by library (Fairino SDK, LeRobot, pynput, accelerate), environment (Docker, Wayland, NCCL), or general problem domain.
2. **Reference applicable patterns in the ToDo entry** using `(see LP §X)` where `X` is the section of `LearnedPatterns.md` being cited. Example:
   ```
   - [ ] Add servo session recovery on error 14 (see LP §3)
   ```
3. **After the task completes**, if a new recurring issue, gotcha, library quirk, workflow lesson, or environment-specific note surfaced, append it to the correct section of `LearnedPatterns.md`. Use the Problem / Cause / Fix / Rule format specified in Learned Patterns Bootstrap below.
4. **Promote stable patterns**: entries in `LearnedPatterns.md` that stabilize across many tasks should be lifted into a formal rule inside this `CLAUDE.md`. Remove the promoted entry from `LearnedPatterns.md` to avoid duplication.

---

## Learned Patterns Bootstrap

If `LearnedPatterns.md` does not exist in the repository root, generate it by analyzing the `[x]` items in `ToDo.md` using the procedure below. Once the file exists, this bootstrap procedure no longer applies — consult the file directly.

### Procedure

1. Read every `[x]` item across all sections in `ToDo.md`.
2. Classify each item into exactly one of the following categories:
   - **§1. Recurring Issues** — the same or a similar problem appeared **two or more times**.
   - **§2. Solved Gotchas** — a one-time trap with a credible chance of recurring.
   - **§3. Library Quirks** — hidden or surprising behavior of a specific library or tool (Fairino SDK, pynput, NCCL, accelerate, etc.).
   - **§4. Workflow Lessons** — lessons learned about the development or collaboration process itself.
   - **§5. Environment Specifics** — Docker, Ubuntu, Wayland, hardware, or network-specific notes.
3. Items that do not cleanly fit any category go into **§99. Uncategorized**. Do **not** discard them.
4. For each entry, record four single-line fields:
   - **Problem**: what went wrong.
   - **Cause**: the underlying reason.
   - **Fix**: the specific change that resolved it.
   - **Rule**: a short general directive in `Always ...` or `Never ...` form.
5. Append `(from ToDo#N)` at the end of each entry, where `N` identifies the source ToDo section (1-based index of the top-level `##` heading), so the original record can be recovered on later review.

### Constraints

- **Do not modify `ToDo.md`.** It is append-only (checkbox updates excepted per the Exceptions section above); all other edits happen only in `LearnedPatterns.md`.
- **Create `LearnedPatterns.md` as a new file** in the repository root. Do not inline patterns into `ToDo.md` or `CLAUDE.md`.
- **Do not invent patterns.** When a ToDo item is ambiguous, place it under §99 rather than guessing.
- **Write all content in English**, consistent with the Language rule under MIT Code Convention.

---

## Commit Messages

Follow the **Conventional Commits** specification. The English-only rule for commit messages, PR titles, and PR bodies follows the Language rule under MIT Code Convention.

### Format

```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

### Types

| Type | Purpose |
|---|---|
| `feat` | New feature |
| `fix` | Bug fix |
| `refactor` | Code restructuring without behavior change |
| `docs` | Documentation only |
| `test` | Adding or modifying tests |
| `chore` | Build config, .gitignore, etc. |
| `style` | Formatting only (no behavior change) |
| `perf` | Performance improvement |

### Rules

- Description in **imperative mood**: "Add", "Fix" (NOT "Added", "Fixed")
- Subject line **under 50 characters**
- **No period** at the end of the subject line
- Wrap body at 72 characters
- Body explains **"what and why"** (the code shows "how")
- Keep scope short and focused on the affected area (e.g., `fairino`, `teleop`, `train`)

### Examples

```
feat(fairino): add servo session recovery on error 14

Re-runs the ServoMoveStart init sequence when the controller
reports a stale session, instead of aborting the control loop.
```

```
fix(teleop): prevent first-loop gripper slam
```

```
chore(build): update pyproject.toml for new module
```

### Breaking Changes

Mark backward-incompatible changes with `!` or a footer:

```
feat(api)!: change return type of parse() to dict

BREAKING CHANGE: parse() previously returned a tuple;
it now returns a dict.
```

> **Source**: [Conventional Commits v1.0.0](https://www.conventionalcommits.org/en/v1.0.0/)

---

## Branching Strategy

Adopt **GitHub Flow** — a lightweight single-main-branch strategy.

> **Project override** (per Rule Priority): the Task Management workflow in this repository commits routine task work directly to `main` after each completed user command. Cut a working branch and open a PR for large, risky, or multi-session changes; for everyday task commits the direct-to-main flow stands.

### Principles

- `main` is **always in a deployable state**
- Larger changes happen on **separate branches** cut from `main` and merge via **Pull Requests**
- **Delete branches after merging**
- **Open PRs even when working solo** (for self-review and history tracking)
- **Prefer the `gh` CLI over raw `git` whenever possible.** Use `gh` for any GitHub-side operation (issues, PRs, reviews, releases, repo inspection — e.g. `gh issue create`, `gh pr create`, `gh pr merge`, `gh release create`). Reserve plain `git` for local version control that `gh` does not cover (staging, commits, local branches). This keeps the workflow consistent with Task Management, which already drives issues through `gh`.

### Branch Naming

```
<type>/<short-description>
```

Examples:
- `feature/csv-parser`
- `fix/memory-leak-in-loader`
- `fix/issue-42`
- `refactor/error-handling`
- `docs/api-reference`

### Standard Workflow (branched changes)

```bash
# 1. Get latest main
git checkout main
git pull origin main

# 2. Create a working branch
git checkout -b feature/csv-parser

# 3. Work and commit
git add <explicit paths>
git commit -m "feat(parser): add CSV reader"

# 4. Push to remote
git push origin feature/csv-parser

# 5. Open a PR on GitHub → review → merge

# 6. Clean up locally after merge
git checkout main
git pull origin main
git branch -d feature/csv-parser
```

> **Source**: [GitHub Flow Documentation](https://docs.github.com/en/get-started/using-github/github-flow)

---

## .gitignore

Cover Python build/cache artifacts plus standard editor and OS files. Use GitHub's official Python template as a base.

### Base Template

```gitignore
# ===== Python =====
__pycache__/
*.py[cod]
*$py.class
*.egg-info/
*.egg
.eggs/
build/
dist/
.pytest_cache/
.ruff_cache/
.mypy_cache/
.coverage
htmlcov/
.tox/

# ===== Virtual environments =====
.venv/
venv/
env/

# ===== Editor / OS =====
.vscode/
.idea/
*.swp
*.swo
.DS_Store
Thumbs.db

# ===== Secrets =====
.env
.env.local
*.key
*.pem
```

### Global .gitignore

Keep OS/editor-specific files in a personal `~/.gitignore_global`:

```bash
git config --global core.excludesfile ~/.gitignore_global
```

> **Source**: [GitHub Official .gitignore Template (Python)](https://github.com/github/gitignore/blob/main/Python.gitignore)

---

## Versioning

Follow **Semantic Versioning (SemVer)**.

### Format

```
MAJOR.MINOR.PATCH
```

| Component | When to increment |
|---|---|
| **MAJOR** | Backward-incompatible changes |
| **MINOR** | Backward-compatible new features |
| **PATCH** | Backward-compatible bug fixes |

### Mapping to Conventional Commits

- `fix:` → **PATCH** bump
- `feat:` → **MINOR** bump
- `BREAKING CHANGE` → **MAJOR** bump

### Tagging

```bash
# Create an annotated tag (recommended)
git tag -a v0.1.0 -m "Initial release"

# Push the tag
git push origin v0.1.0

# Push all tags
git push origin --tags
```

### Initial Development

- `0.y.z` is for initial development — the public API is considered unstable
- The first stable release should be `1.0.0`

> **Source**: [Semantic Versioning 2.0.0](https://semver.org/)

---

## Pull Request Guidelines

### Title

Use the same Conventional Commits format as commit messages:

```
feat(parser): add JSON config loader
```

### Description Template

```markdown
## Changes
- Brief summary of what changed

## Why
- Motivation behind the change

## Testing
- How the change was verified (added tests, manual testing, etc.)

## Related Issues
Closes #42
```

### Size

- Keep PRs **under 400 lines** when possible (for effective review)
- Split large changes into multiple PRs

---

## Git Automation (Optional)

Use **pre-commit** for automated style checks and formatting. This repository already ships a `.pre-commit-config.yaml` (ruff, typos, mypy, bandit, gitleaks — see Build & Test Commands); the snippet below is the CommonClaude baseline for new projects.

### Installation

```bash
pip install pre-commit
```

### Example `.pre-commit-config.yaml`

```yaml
repos:
  - repo: https://github.com/pre-commit/pre-commit-hooks
    rev: v4.5.0
    hooks:
      - id: trailing-whitespace
      - id: end-of-file-fixer
      - id: check-yaml
      - id: check-added-large-files

  # Python
  - repo: https://github.com/astral-sh/ruff-pre-commit
    rev: v0.6.9
    hooks:
      - id: ruff
        args: [--fix]
      - id: ruff-format
```

### Enable Hooks

```bash
pre-commit install
```

Checks and formatting will now run automatically on `git commit`.

> **Source**: [pre-commit Documentation](https://pre-commit.com/)

---

## References (Git Convention)

### Primary Sources (Specifications / Official Docs)

| Item | URL |
|---|---|
| Conventional Commits | https://www.conventionalcommits.org/ |
| GitHub Flow | https://docs.github.com/en/get-started/using-github/github-flow |
| Semantic Versioning | https://semver.org/ |
| GitHub .gitignore Templates | https://github.com/github/gitignore |
| pre-commit | https://pre-commit.com/ |

### Learning Resources

| Resource | URL |
|---|---|
| Pro Git (free book) | https://git-scm.com/book/en/v2 |
| MIT Missing Semester — Version Control | https://missing.csail.mit.edu/2020/version-control/ |
| Oh Shit, Git!?! (recovery guide) | https://ohshitgit.com/ |
| Learn Git Branching (interactive) | https://learngitbranching.js.org/ |

### Commit Message Writing Guides

| Resource | URL |
|---|---|
| Tim Pope, "A Note About Git Commit Messages" | https://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html |
| Chris Beams, "How to Write a Git Commit Message" | https://cbea.ms/git-commit/ |

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
