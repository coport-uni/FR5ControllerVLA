# ToDo

## 2026-04-13: Update CLAUDE.md and hooks from CommonClaude

- [x] Fetch CommonClaude repo contents (CLAUDE.md, hooks, settings.json)
- [x] Merge CommonClaude sections into project CLAUDE.md (Language, Debug File Management, Task Management, Testing Rules, Linting)
- [x] Preserve project-specific content (Architecture, FR5 Integration, CLI Entry Points)
- [x] Create `.claude/hooks/pre-write-guard.sh`
- [x] Create `.claude/hooks/post-write-lint.sh`
- [x] Create `.claude/hooks/post-write-debug-remind.sh`
- [x] Add hooks block to `.claude/settings.json` (preserve existing permissions)
- [x] Verify JSON validity and script permissions

## 2026-04-13: FR5-to-FR5 Leader-Follower Teleoperation

- [x] Create `FairinoLeaderConfig` in `src/lerobot/teleoperators/fairino_leader/config_fairino_leader.py`
- [x] Create `FairinoLeader` teleoperator in `src/lerobot/teleoperators/fairino_leader/fairino_leader.py`
- [x] Create `__init__.py` for fairino_leader module
- [x] Register `fairino_leader` in `teleoperators/utils.py` factory
- [x] Add import in `scripts/lerobot_teleoperate.py`
- [x] Create `5__fr5_leader_follower.sh` launch script
- [x] Run ruff check and verify imports
- [x] Live test with both robots (follower=192.168.58.2, leader=192.168.59.2)

## 2026-04-13: Fix ServoJ error 101 in FairinoFollower

- [x] Diagnose ServoJ error 101 — init settle times too short (0.2s → 1.0s needed)
- [x] Diagnose ServoJ error 101 in daemon threads — firmware rejects non-main-thread calls
- [x] Remove background servo thread; call ServoJ synchronously in `send_action()`
- [x] Increase `_initialise_servo_mode()` settle times to 1.0s per step
- [x] Add `_recover_servo_session()` for error 14 (servo session expired)
- [x] Add `StopMotion()` to init sequence for cleaner reset
- [x] Verify FairinoFollower J1 +5° move: SUCCESS (err=0.00°)
- [x] Verify FairinoLeader + FairinoFollower end-to-end: SUCCESS (all joints err=0.00°)
- [x] Run ruff check and format on all modified files

## 2026-04-13: Increase follower servo speed by 30%

- [x] Change `max_servo_speed` from 60.0 to 78.0 deg/s in config

## 2026-04-13: Add gripper support to FairinoLeader

- [x] Add gripper config fields to `config_fairino_leader.py`
- [x] Add gripper read/init to `fairino_leader.py`
- [x] Update `4__fr5_leader_follower.sh` with gripper flags
- [x] Ruff check passed
- [x] Gripper control via keyboard O/C (hardware drag not viable)
- [x] Reuse _StdinReader from keyboard teleop for O/C keys
- [ ] User terminal test with 4__fr5_leader_follower.sh

## 2026-04-13: piper_follower HardwareAdd.md 준수 수정

- [x] `get_status()` 버그 수정 (print를 dict 리터럴 안에 사용 → set 반환)
- [x] `send_action()` 시그니처에서 `is_conv` 파라미터 제거 (Robot 인터페이스 불일치)
- [x] `connect()` 반환 타입 `bool` → `None` (실패 시 예외 raise)
- [x] `calibrate()` 에서 `return True` 제거
- [x] 미사용 import 정리 (`HF_LEROBOT_CALIBRATION`, `ROBOTS` — `Path`는 type hint에 필요하여 유지)
- [x] ruff check 통과 확인

## 2026-04-14: fairino_leader / fairino_follower HardwareAdd.md 준수 점검

- [x] fairino_follower 디렉토리/파일 구조 확인 (완전 준수)
- [x] fairino_follower 네이밍 패턴 확인 (완전 준수)
- [x] fairino_follower @RobotConfig.register_subclass() 등록 확인
- [x] fairino_follower 필수 메서드 확인 (모두 정확)
- [x] fairino_follower __init__.py export 확인
- [x] fairino_follower robots/utils.py factory 등록 확인 — **버그 발견** (`.fairino` → `.fairino_follower`)
- [x] fairino_leader 디렉토리/파일 구조 확인 (완전 준수)
- [x] fairino_leader 네이밍 패턴 확인 (완전 준수)
- [x] fairino_leader @TeleoperatorConfig.register_subclass() 등록 확인
- [x] fairino_leader 필수 메서드 확인 (모두 정확)
- [x] fairino_leader __init__.py export 확인
- [x] fairino_leader teleoperators/utils.py factory 등록 확인
- [x] `robots/utils.py:72` import 경로 수정 (`.fairino` → `.fairino_follower`)
- [x] CLAUDE.md 문서 경로 업데이트 (`robots/fairino/` → `robots/fairino_follower/`)
- [x] Ruff check/format 통과
- [x] Import 검증 성공 (make_robot_from_config → FairinoFollower)
- [ ] commit + push
