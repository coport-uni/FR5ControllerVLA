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

## Key Findings

- **MoveJ** returns error 101 or 154 on firmware V3.9.1; use **ServoJ** instead.
- Init sequence: `RobotEnable(0)` -> `ResetAllError` -> `RobotEnable(1)` -> `Mode(0)` -> `ServoMoveStart`.
- ServoJ XMLRPC call takes **7 params** (no `id` arg); the SDK wrapper passes 8.
- `main_code=1` indicates a residual error state; must disable-then-enable to clear.
