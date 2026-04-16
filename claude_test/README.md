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

## Key Findings

- **MoveJ** returns error 101 or 154 on firmware V3.9.1; use **ServoJ** instead.
- Init sequence: `RobotEnable(0)` -> `ResetAllError` -> `RobotEnable(1)` -> `Mode(0)` -> `ServoMoveStart`.
- ServoJ XMLRPC call takes **7 params** (no `id` arg); the SDK wrapper passes 8.
- `main_code=1` indicates a residual error state; must disable-then-enable to clear.
- **Init settle times must be ≥1 s** per step; 0.2–0.3 s causes persistent ServoJ error 101.
- **ServoJ must run from the main thread**; daemon threads always get error 101 on this firmware.
- Hold-position ServoJ (same position) returns 101; after ~40 consecutive 101s the controller terminates the session (error 14).
