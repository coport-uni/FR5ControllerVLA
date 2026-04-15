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
- [x] commit + push (bb8c235f) + issue #7 close

## 2026-04-15: 2__find_camera.sh RealSense/OpenCV 분리

- [x] `2__find_camera.sh` 에 카메라 타입 인자 지원 추가 (realsense/opencv/all)
- [x] 기본값을 `all` 로 두되, 인자 전달 시 해당 타입만 탐색
- [x] RealSense 뎁스/메타데이터 노드(`/dev/video2`, `/dev/video4`) 경고 회피 경로 확보
- [x] Shell 스크립트 동작 검증 (문법 체크)
- [x] commit + push + gh issue 등록/close (#8)

## 2026-04-15: fairino_leader 그리퍼 초기화 버그 수정

- [x] `FairinoLeader._gripper_pos` 가 0.0 으로 고정 시작되어 첫 루프에서 팔로워 그리퍼가 강제로 완전히 닫히는 문제 확인
- [x] `FairinoLeader.send_feedback()` 를 첫 호출 한정으로 `feedback["gripper.pos"]` 로 `_gripper_pos` 초기화하도록 수정 (`KeyboardFairinoTeleop` 패턴과 동일)
- [x] `_gripper_initialised` 플래그 추가하여 이후 호출은 무시 (키보드 입력 권한 유지)
- [x] Ruff check/format 통과 확인
- [x] gh issue 등록 (#9) + commit + push

## 2026-04-15: lerobot_teleoperate.py fairino import 경로 수정

- [x] `src/lerobot/scripts/lerobot_teleoperate.py:78` 에서 `fairino` 를 import 하지만 실제 모듈명은 `fairino_follower` 이므로 ImportError 발생
- [x] `fairino` → `fairino_follower` 로 교체
- [x] Ruff check/format 통과 확인
- [x] gh issue 등록 (#10) + commit + push

## 2026-04-15: fairino_leader.py 내부 fairino SDK import 경로 수정

- [x] `src/lerobot/teleoperators/fairino_leader/fairino_leader.py:139` 의 lazy import `lerobot.robots.fairino.fairino.Robot` → `lerobot.robots.fairino_follower.fairino.Robot` 로 수정
- [x] Ruff check/format 통과 확인
- [x] 런타임 import 검증
- [x] gh issue 등록 (#11) + commit + push

## 2026-04-15: find_cameras.py RealSense/OpenCV 필터 + warmup 수정 (#12)

- [x] RealSense V4L2 노드를 OpenCV 스캔에서 필터링 (`/sys/class/video4linux/<dev>/name` 기반)
- [x] RealSense `warmup_s` 를 1초 → 5초 로 상향 (D400 시리즈 첫 프레임 지연 대비)
- [x] `pyrealsense2` 를 lerobot conda 환경에 설치
- [x] Ruff check/format 통과
- [x] `./2__find_camera.sh opencv` 검증 — 경고 없이 HikVision 3대 저장
- [x] `./2__find_camera.sh realsense` 엔드투엔드 이미지 저장 — USB 3.2 포트로 이동 후 1280x720 RGB PNG 정상 저장
- [x] 기본(둘 다) 모드 검증 — 경고 없이 RealSense + HikVision 3대 모두 저장
- [x] commit + push + issue close

## 2026-04-15: 5__fr5_record.sh + lerobot_record.py fairino 등록 (#13)

- [x] `src/lerobot/scripts/lerobot_record.py` 에 `fairino_follower`/`fairino_leader` import 추가 (RobotConfig/TeleoperatorConfig subclass 등록용)
- [x] `5__fr5_record.sh` 작성
  - Leader 192.168.59.2, Follower 192.168.58.2, gripper 활성화
  - Cameras: top_left `/dev/video18`, top_right `/dev/video19` (OpenCV 640x480@20), hand RealSense SN 333422300435 (640x480@30)
  - dataset.repo_id=${HF_USER}/FR5_pick_red_colored_marker_to_box, num_episodes=10, fps=20
  - single_task="Pick the red marker and put in the box"
- [x] Ruff check/format 통과 확인
- [x] `lerobot-record --help` 로 fairino_follower/leader 인식 여부 검증
- [x] gh issue 등록 + commit + push

## 2026-04-15: 5__fr5_record.sh RealSense 에러 진단 및 검증

- [x] `lsusb` / `pyrealsense2.context().query_devices()` 로 D455 SN 333422300435 인식 확인 (USB 3.2, FW 5.13.0.55)
- [x] `lerobot-find-cameras realsense` 정상 동작 확인
- [x] RealSense 640x480@30fps connect/read/disconnect 직접 검증
- [x] OpenCV `/dev/video18`, `/dev/video19` 640x480@20fps 검증
- [x] 결론: 카메라 설정 정상. 에러는 직전 프로세스 점유로 인한 transient busy 로 추정
