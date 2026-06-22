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

## 2026-04-15: FairinoLeader 그리퍼 드래그-티치 방식으로 전환

- [x] `FairinoLeaderConfig` 에 gripper 하드웨어 필드 추가 (company, device, index, vel, force=1)
- [x] `FairinoLeader` 에서 키보드 관련 코드 제거 (`_StdinReader`, `_key_queue`, `_process_gripper_keys`, keyboard import)
- [x] `_initialise_gripper()` 추가: SetGripperConfig + ActGripper + 낮은 force 로 MoveGripper(block=0) → 손으로 back-drive 가능한 컴플라이언스 상태
- [x] `_read_gripper_pos()` 추가: GetGripperCurPosition 으로 실시간 위치 읽기
- [x] `get_action()` 에서 실제 그리퍼 위치를 읽어 `gripper.pos` 로 반환
- [x] `disconnect()` 에서 ActGripper(0) 로 비활성화
- [x] `4__fr5_leader_follower.sh`, `5__fr5_record.sh` 에 테스트용 `--teleop.gripper_force=20` 추가
- [x] Ruff check/format 통과
- [x] 런타임 import 검증
- [x] gh issue 등록 (#15) + commit + push

## 2026-04-15: Docker GUI(X server) 셋업 및 PNG 표시 검증

- [x] X11 연결 진단 (DISPLAY=:31, /tmp/.X11-unix/X31 → VS Code 포워더 → 호스트 Xwayland :0)
- [x] 호스트 Xwayland MIT 쿠키를 컨테이너 xauth에 등록 (`/proc/<xwayland-pid>/root/run/user/1000/.mutter-Xwaylandauth.*`)
- [x] `x11-apps` `x11-utils` `feh` `xauth` 설치
- [x] `xdpyinfo`로 X 서버 연결 검증 성공
- [x] feh 창이 NUC 모니터에 표시됨을 확인 (호스트 모드 동작)
- [x] `claude_test/show_image.sh` 작성 + `claude_test/README.md` 갱신
- [x] 사용자가 `ssh -X`로 NUC 접속 시 `xclock` 노트북 표시 동작 확인
- [x] `Xserver.sh` 작성: NUC SSH 세션의 `DISPLAY`/`XAUTHORITY`를 컨테이너로 brige
- [x] `show_image.sh`에 `--mode=ssh` 분기 추가 (SSH X11 forwarded display 사용)
- [ ] 사용자가 컨테이너에서 `source Xserver.sh && claude_test/show_image.sh outputs/captured_images/realsense_333422300435.png` 실행하여 노트북 화면 표시 검증
- [ ] commit + push

## 2026-04-16: Fix spd-say FileNotFoundError in 5__fr5_record.sh

- [x] 원인 확인: `src/lerobot/utils/utils.py:111 say()`가 `spd-say`를 Popen → 미설치 시 FileNotFoundError
- [x] `speech-dispatcher` apt 설치로 `spd-say` 제공 (gh issue #17)
- [x] `spd-say` 단독 실행 검증 (exit=0, 서버 미기동은 경고만)
- [x] `say()` 를 `shutil.which` 체크로 하드닝 (TTS 없는 환경에서도 로그만 남기고 조용히 지나가도록)
- [x] `log_say` 호출이 FileNotFoundError 없이 완료됨 확인
- [x] ruff check/format, commit, push

## 2026-04-16: requirements-ubuntu.txt 를 현재 venv 설치 상태로 최신화 (pip freeze)

- [x] `.venv/bin/pip freeze` 로 현재 설치된 패키지 버전 수집 (117개 + editable lerobot)
- [x] `requirements-ubuntu.txt` 를 freeze 출력으로 덮어쓰기 (헤더: 생성 방식·일자, `-e .[all]` 유지, editable git+ 라인 제거)
- [x] gh issue 등록 + commit + push

## 2026-04-16: 5__fr5_record.sh 카메라 fps 설정 점검

- [x] LeRobot dataset v3 문서 확인 (카메라 fps vs dataset fps 관계)
- [x] lerobot-record 소스 코드에서 카메라 fps 처리 희름 분석
- [x] RealSense hand 카메라 fps(30)와 dataset fps(20) 불일치 식별
- [x] 사용자가 전체 카메라 30fps로 통일 (5__fr5_record.sh 이미 업데이트됨)
- [ ] dataset.fps=20 → 30 변경 여부 사용자 결정 대기

## 2026-04-16: FairinoFollower 카메라 키 convention 수정 (KeyError: 'top_left')

- [x] 원인 분석: `build_dataset_frame`이 short key("top_left") 기대하나 Fairino는 full key("observation.images.top_left") 반환
- [x] SO follower 참조 구현 확인 — `observation_features`와 `get_observation()` 모두 short key 사용
- [x] `observation_features`에서 카메라 키를 short key로 변경 (cam_name만 사용)
- [x] `get_observation()`에서 카메라 키를 short key로 변경
- [x] `robots/fairino/` 의 old copy도 동일 수정 적용
- [x] ruff check/format 통과
- [x] commit (29262708) + push + gh issue #20 close

## 2026-04-20: FairinoLeader 그리퍼 30초 후 잠김 해소

- [x] 원인 분석: `_GRIPPER_MAXTIME_MS = 30000`으로 MoveGripper가 30초 후 종료 → compliance 해제 → 그리퍼 잠김
- [x] `_GRIPPER_MAXTIME_MS = -1`로 변경 (SDK가 지원하는 무한 유지 값)
- [x] 관련 주석 업데이트
- [x] `robots/fairino/` 의 old copy 없음 (fairino_leader는 teleoperators에만 존재)
- [x] ruff check/format 통과
- [x] gh issue #21 등록 + commit + push

## 2026-04-20: FairinoFollower 그리퍼 조작 시 제어 루프 60Hz → 1Hz 하락 해소

- [x] 원인 분석: `_send_gripper_cmd()`가 메인 제어 루프에서 동기 실행
      (`ServoMoveEnd → sleep(0.02) → MoveGripper → sleep(0.02) → ServoMoveStart → sleep(0.02)`)
- [x] 그리퍼 워커 스레드 도입 — 메인 루프는 latest-wins 슬롯에 타깃만 넣고 즉시 리턴
- [x] `connect()` 에서 워커 기동, `disconnect()` 에서 정상 종료 + join
- [x] 워커용 별도 `xmlrpc.client.ServerProxy` (`_gripper_proxy`) 분리 — Transport 레이스 방지
- [x] ServoJ 복구 경로(`_recover_servo_session`) 재사용 확인
- [x] ruff check / ruff format 통과
- [x] 사용자 하드웨어 테스트 통과 (그리퍼 연속 조작 중 60Hz 유지 확인)
- [x] gh issue #22 등록 + commit + push + close

## 2026-04-20: (취소) Follower 30초 비활성 오판

- [x] 사용자가 실제로는 leader 그리퍼 락 문제를 지적. Follower 방향으로 생성한 issue #23 close, 본 항목 취소.

## 2026-04-20: FairinoLeader 그리퍼 10분 후 락 문제 (issue #24)

- [x] 이력 확인: `42d66938` (-1) → `bcd839d1` (600000, "SDK 가 -1 거부") 로 변경된 상태
- [x] 원인: `MoveGripper(..., maxtime=600000, ...)` 가 10분 뒤 만료 → compliance 해제 → 잠김
- [x] gh issue #24 등록
- [x] 방식 선택: (B) `_GRIPPER_MAXTIME_MS` 상향 시도 (keepalive 스레드는 실패 시 폴백)
- [x] `_GRIPPER_MAXTIME_MS` 를 int32 최대값(`2147483647`, ~24.8일)으로 변경 + 코멘트 업데이트
- [x] ruff check / format 통과
- [x] 사용자 하드웨어 테스트: **실패** — int32 max 값에도 불구하고 일정 시간 후 락 재현
- [x] 폴백 결정: (A) keepalive 스레드로 전환 → 아래 2026-04-21 항목에서 진행

## 2026-04-21: FairinoLeader 그리퍼 keepalive 스레드 도입 (issue #24 폴백)

- [ ] `config_fairino_leader.py` 에 keepalive 필드 추가 (enabled, interval, idle thresholds, nudge pct, settle)
- [ ] `fairino_leader.py` 에 `threading` / `xmlrpc.client` import 추가
- [ ] `__init__` 에 keepalive 상태 필드 추가 (`_keepalive_rpc`, `_keepalive_thread`, events, activity tracker)
- [ ] `_update_activity()` helper 추가 + `get_action()` 말미에 활동 감지 훅
- [ ] `_keepalive_loop()` + `_issue_move_gripper()` helper 구현 (워커 스레드 본체)
- [ ] `_start_keepalive()` / `_stop_keepalive()` helper 추가
- [ ] `connect()` 에서 `_initialise_gripper()` 뒤에 `_start_keepalive()` 호출
- [ ] `disconnect()` 시작부에 `_stop_keepalive()` 호출 (ActGripper(0)/CloseRPC 와의 경합 방지)
- [x] ruff check / format 통과
- [ ] import smoke: `from lerobot.teleoperators.fairino_leader import FairinoLeader, FairinoLeaderConfig`
- [x] gh issue 등록 (#25)
- [x] 후속 수정 1: `_initialise_gripper()` 의 compliance target 을 하드코딩된 `0` → 현재 위치로 변경
- [x] 후속 수정 2: `get_action()` 이 keepalive 진행 중에는 live read 대신 frozen 값을 반환하도록 마스킹 → nudge 가 Leader 하드웨어만 움직이고 Follower 로 전달되지 않음 (`_reported_gripper_pos`)
- [ ] 사용자 하드웨어 테스트 3차: Leader 하드웨어 drift 여부 + Follower 정지 여부 + keepalive nudge 로그 확인
- [ ] 사용자 하드웨어 soak 테스트: 15분 이상 유휴 후 back-drive 유지 확인
- [ ] 성공 시 commit + push + issue close

## 2026-04-21: 1__setup_camera.sh 재실행 시 OpenCV v4l2loopback 오픈 행 문제

- [x] 증상: `2__find_camera.sh opencv` 실행 시 `/dev/video18/19/20` 에서 `cv2.VideoCapture.open` 이 블록 (RealSense `/dev/video2/4` 는 정상). VLC 로 RTSP 직접 재생은 정상.
- [x] 재현: `1__setup_camera.sh` 를 이미 v4l2loopback + ffmpeg 가 붙어있는 상태에서 재실행하면 기존 writer 가 남아있어 OpenCV reader 가 프레임을 못 받는 것으로 추정.
- [x] 워크어라운드: `hkvision_related/rtsp_to_v4l2_teardown.sh` 로 기존 ffmpeg writer + v4l2loopback 모듈을 정리한 뒤 `1__setup_camera.sh` 실행하면 OpenCV 에서도 정상 열림.
- [ ] 근본 수정안 검토: `1__setup_camera.sh setup_loopback` 초입에서 teardown 을 호출하거나, 기존 ffmpeg writer 감지 시 재시작하도록 가드 추가
- [x] gh issue 등록 (#26)

## 2026-04-21: 5__fr5_record.sh git clone 실패 수정 — `hf download` 로 교체

- [x] 원인: (1) 기존 `outputs/datasets/FR5_pick_red_colored_marker_to_box/` 디렉터리가 이미 존재하여 `git clone` 이 *destination path already exists* 로 실패, (2) 설령 clone 이 성공해도 `.parquet`/`.mp4` 가 LFS 포인터 파일로만 받아져 resume 시 `LeRobotDataset.load_hf_dataset()` 가 실패
- [x] 손상된 기존 디렉터리 제거 (LFS 포인터 파일 296K만 존재)
- [x] `5__fr5_record.sh` 의 `git clone` 라인을 `hf download --repo-type dataset coport-uni/FR5_pick_red_colored_marker_to_box --local-dir <root>` 로 교체
- [x] `bash -n` 구문 체크 통과 (ruff 대상 아님)
- [x] gh issue 등록 (#27)
- [ ] commit + push (사용자 요청으로 보류)
- [ ] 사용자 하드웨어 검증: `./5__fr5_record.sh` 실행 시 `hf download` 로 parquet/mp4 본체가 받아지고 `lerobot-record` 가 episode 5/10 에서 resume 되는지 확인

## 2026-04-21: 6/8/9번 스크립트 FR5용 재작성 + 5번 오타 수정 (#28)

- [x] `5__fr5_record.sh:13` `HF_HUB_ENABLE_HF_TRANSER` → `HF_HUB_ENABLE_HF_TRANSFER` 오타 수정
- [x] `5__fr5_record.sh` lines 15, 17 중복 주석 제거 (line 16만 유지)
- [x] `6__replay.sh` 전체 재작성 (piper → fairino_follower, FR5 데이터셋 사용)
- [x] `8__run_server.sh` 경로 수정 (`scripts/server/policy_server.py` → `async_inference.policy_server`, conda 경로 anaconda3)
- [x] `9__run_client.sh` 전체 재작성 (piper → fairino_follower, `async_inference/robot_client.py` 호출)
- [x] `src/lerobot/scripts/lerobot_replay.py` 에 `fairino_follower` import 추가
- [x] `src/lerobot/async_inference/robot_client.py` 에 `fairino_follower` import 추가
- [x] Ruff check/format 통과 확인
- [x] `bash -n` 구문 체크
- [x] gh issue 등록 (#28)
- [ ] commit + push
- [ ] 사용자 하드웨어 검증: 6번 replay, 8+9번 async inference 실제 동작
- [ ] 9번 `PRETRAINED` 값 실제 FR5 정책 체크포인트로 설정 (학습 완료 후)

## 2026-04-21: .gitignore 에 outputs/datasets/ 추가 + 대용량 파일 히스토리 정리 (#29)

- [x] `.gitignore` 에 `datasets/` 추가 (초기 시도 — 경로 오해)
- [x] `.gitignore` 패턴을 `outputs/datasets/` 로 교정
- [x] 푸쉬 시도 — GitHub pre-receive hook 거부 (`1cb4a25d dataset_production_go` 에 100MB+ MP4 9개 포함)
- [x] `git reset --soft` 로 로컬 미푸쉬 커밋(1cb4a25d, 449b0a7b, a8373b05) 해제
- [x] `git rm -r --cached outputs/datasets/` 로 대용량 파일 트래킹 해제 (디스크 보존)
- [x] 2개 커밋으로 재구성: `e2e34ee8 Rewrite 6/8/9 scripts ...` + `0757ff02 Ignore outputs/datasets/`
- [x] `git push origin main` 성공 (1d0003aa..0757ff02)
- [x] gh issue 등록 (#29)

## 2026-04-22: Create model-specific train scripts (ACT, Pi0, Pi0.5) (#31)

- [x] Delete existing `7__train.sh` (smolvla/piper dataset — no longer relevant)
- [x] Create `7__train_act.sh` based on PiPERControllerVLA `7__train.sh`
      reference, adapted for FR5 dataset
      `coport-uni/FR5_pick_red_colored_marker_to_box`
      (ref: https://github.com/coport-uni/PiPERControllerVLA/blob/main/7__train.sh)
      — note: uses `./src/lerobot/scripts/lerobot_train.py` (PiPER ref path
      `scripts/train.py` no longer exists in this repo)
- [x] Create `7__train_pi0.sh` following LeRobot Pi0 docs
      (ref: https://huggingface.co/docs/lerobot/pi0)
- [x] Create `7__train_pi05.sh` following LeRobot Pi0.5 docs
      (ref: https://huggingface.co/docs/lerobot/pi05)
- [x] `bash -n` syntax check for all three scripts
- [x] gh issue 등록 (#31)
- [x] Switch `7__train_act.sh` to `lerobot-train` CLI per LeRobot ACT docs
      (https://huggingface.co/docs/lerobot/act)
- [x] Smoke test — all three scripts train on the FR5 dataset without
      error:
      - ACT  : 300 steps, loss 6.923 @ step 200, ckpt 000200/000300
      - Pi0  : 20 steps,  ckpt 000010/000020/last, "End of training"
      - Pi0.5: 20 steps,  ckpt 000010/000020/last, "End of training"
      (ran with container's conda; shell scripts themselves reference the
      host's `/home/inno-controller/anaconda3/...` path like other scripts.)
- [ ] commit + push

## 2026-04-22: Fix GR00TN15Config dataclass bug blocking lerobot imports (#32)

- [x] Create gh issue (#32)
- [x] Fix `src/lerobot/policies/groot/groot_n1.py` `GR00TN15Config`
      — added `default=None` to the four `init=False` fields
      (`backbone_cfg`, `action_head_cfg`, `action_horizon`, `action_dim`)
      so they precede `compute_dtype` legally on Python 3.12
- [x] Ruff check/format on modified file (explicit run: all passed)
- [x] Import smoke: `from lerobot.policies.groot.groot_n1 import GR00TN15Config`
- [x] Verify `lerobot-train --help` loads without TypeError
- [ ] commit + push (bundled with #31)

## 2026-04-22: Apply HuggingFace `accelerate` for multi-GPU training (#30)

- Reference: https://huggingface.co/docs/lerobot/multi_gpu_training
- Target: `7__train_act.sh` launch script (working ACT pipeline) plus the
  shared training entry `src/lerobot/scripts/lerobot_train.py`.
- Method: `accelerate launch --multi_gpu --num_processes=2
  --mixed_precision=bf16 $(which lerobot-train) ...`. `lerobot_train.py`
  already auto-detects `accelerate` (Accelerator init, prepare,
  main-process-gated checkpoint/wandb, MetricsTracker cross-process).
- Purpose: leverage 2x H200 GPUs to speed up FR5 policy training.
- [x] Review `https://huggingface.co/docs/lerobot/multi_gpu_training`
- [x] Audit `src/lerobot/scripts/lerobot_train.py`
- [x] Update `7__train_act.sh` to use
      `accelerate launch --multi_gpu --num_processes=2
      --mixed_precision=bf16 $(which lerobot-train) ...`
- [x] Patch `lerobot_train.py` with
      `InitProcessGroupKwargs(timeout=1h)` so slow one-time startup
      stalls do not trigger the 10-minute default NCCL watchdog
- [x] `bash -n` syntax check on `7__train_act.sh`; ruff check/format on
      `lerobot_train.py`
- [x] Diagnose container-specific hang: NCCL P2P/CUMEM transport fails
      silently on the first allreduce inside this Docker container.
      Workaround for in-container tests only:
      `NCCL_P2P_DISABLE=1 NCCL_SHM_DISABLE=1` forces socket transport.
      Host NUC is unaffected; no changes to `7__train_act.sh` needed.
- [x] Run the training for ~5 minutes on 2x H200 and verify no errors
      (result: reached step 460 in ~6 min, loss 24.25 → 1.69, no errors)
- [x] Confirm both GPUs actively utilized via `nvidia-smi` (both held
      ~44 GB and alternated 95-100% util; evidence saved under
      `claude_test/accelerate_mgpu_evidence/`)
- [x] gh issue 등록 (#30)
- [x] commit + push (048b3ad5)

## 2026-04-22: Save timestamped train.log to output_dir from lerobot_train.py (#33)

- Reason: `lerobot_train.py` currently calls `init_logging` without a
  `log_file`, so console output is lost unless the user pipes it
  manually. A per-run log file inside `output_dir` makes post-hoc
  inspection (including total training time) reproducible.
- Per-line timestamps are already produced by `init_logging`'s
  `custom_format` (`YYYY-MM-DD HH:MM:SS`); this task adds a timestamp
  to the log *filename* so resume / re-runs don't collide.
- [x] `src/lerobot/scripts/lerobot_train.py`: add `from datetime import datetime`
- [x] `src/lerobot/scripts/lerobot_train.py:189`: build
      `output_dir/train_<YYYYMMDD-HHMMSS>.log` on main process only and
      pass it to `init_logging(log_file=...)`; non-main ranks keep
      `log_file=None`
- [x] Ruff check/format on modified file
- [x] Import smoke: `python -c "from lerobot.scripts import lerobot_train"`
- [x] End-to-end verify by running the pi0 command from
      `7__train_pi0.sh` for ~5 min (SIGTERM 143 at 310s) — file
      `outputs/train/fr5_pi0_logtest/train_20260422-052754.log`
      (47 KB, 327 lines) was created with timestamped lines; first
      vs. last line delta is parseable by `datetime.strptime`.
      Note: pi0 spent most of the 5 min in silent `torch.compile`
      (last log line was "Start offline training…"); wall-time
      from log is only meaningful when training actually reaches
      step logging or finishes with "End of training". For
      interrupted runs, wandb `Runtime` is authoritative.
- [x] gh issue 등록 (#33)
- [ ] commit + push

## 2026-04-22: Benchmark 1-GPU vs 2-GPU accelerate (time to step 100, 5 trials each)

- Reference: builds on issue #30 (multi-GPU enablement).
- Question being measured: real wall-clock + training-loop time for the
  ACT pipeline on the FR5 dataset, comparing
  `accelerate launch --num_processes=1` vs
  `accelerate launch --multi_gpu --num_processes=2`, weak scaling
  (per-rank `batch_size=64`), 5 trials each.
- Output: simple CSV at
  `claude_test/accelerate_mgpu_evidence/bench_1v2gpu.csv`
  with `trial,num_gpus,wall_clock_s,training_loop_s,final_loss,timestamp`.
- [x] Create gh issue (#34)
- [x] Write `claude_test/bench_accelerate_1v2gpu.sh`
- [x] Update `claude_test/README.md`
- [x] Run benchmark — extended to 3 models (ACT, Pi0, Pi0.5) at
      100 steps × 5 trials × 2 GPU configs = 30 runs. CSV at
      `claude_test/accelerate_mgpu_evidence/bench_1v2gpu.csv` with
      schema `model,trial,num_gpus,target_steps,batch_size,wall_clock_s,
      training_loop_s,final_loss,timestamp`.
- [x] Verify CSV (30 rows + header, no NA after Pi0.5 trial 4 retry)
      Result (1-GPU → 2-GPU samples/s improvement):
        ACT  : 113.48 → 160.00 samples/s — speedup 1.41× (+41.0 %)
        Pi0  :  15.46 →  19.00 samples/s — speedup 1.23× (+22.9 %)
        Pi0.5:  13.91 →  16.83 samples/s — speedup 1.21× (+21.0 %)
      Larger models scale worse here because socket NCCL fallback is
      bandwidth-bound; the host NUC with native P2P should approach
      the ideal 2× weak-scaling for Pi0/Pi0.5 too. See
      `claude_test/accelerate_mgpu_evidence/bench_1v2gpu_summary.md`.
- [x] Side-finding: `FileExistsError` race in
      `src/lerobot/configs/train.py:122` between rank 0's
      `output_dir.mkdir()` (after `Accelerator()` init at
      `lerobot_train.py:204-205`) and rank 1's `cfg.validate()`.
      Observed twice across the sweeps; re-ran the affected trials in
      isolation. Real fix would gate the dir check on `is_main_process`
      or move it after the first barrier — not in scope for this issue.
- [x] commit + push (719b20e0) + close issue (#34)

## 2026-04-22: ACT 100-step bench at per-rank batch=32 (counterpart to #34)

- Same `7__train_act.sh`-style launch as #34, but per-rank
  `batch_size=32` instead of 64. Weak scaling:
  - 1 GPU: batch=32 (effective=32)
  - 2 GPU: batch=32 per rank (effective=64)
- 5 trials per group, 100 steps, append rows to the existing
  `claude_test/accelerate_mgpu_evidence/bench_1v2gpu.csv`.
- [x] Create gh issue (#35)
- [x] Run 10 trials via bench script (`--model act --batch 32`)
- [x] Verify CSV (10 new rows, no NA — final 40 rows)
- [x] Extend `bench_1v2gpu_summary.md` with a batch=32 section
      Result (1-GPU → 2-GPU, ACT per-rank batch=32):
        samples/s 110.34 → 157.64 — speedup 1.43× (+42.9 %)
        loop_s 29.00±0.63 → 40.60±1.20
        scaling efficiency 71.4 % (vs 70.5 % at batch=64 — same within
        noise; allreduce of ~104 MB bf16 grad scales with model size,
        not batch size).
- [x] commit + push (a9ff2571) + close issue (#35)

## 2026-04-22: Diagnose inactive arrow-key controls in lerobot-record

- User asked why Right/Left/Esc shortcuts in `5__fr5_record.sh` do
  not work on the FR5 control PC. Diagnostic only — no code change.
- Finding: `init_keyboard_listener()`
  ([src/lerobot/utils/control_utils.py:119-169](src/lerobot/utils/control_utils.py#L119-L169))
  uses pynput's xorg backend
  (`pynput.keyboard._xorg.Listener`, verified in both `.venv` and
  conda `lerobot` env). The session is Wayland
  (`XDG_SESSION_TYPE=wayland`, `loginctl Type=wayland`), which blocks
  X11 global key capture — listener starts cleanly but receives no
  events. Secondary: `--display_data=true` Rerun viewer can steal
  focus.
- Remediation options presented to user:
  1. Switch login session to "Ubuntu on Xorg" (no code change).
  2. Replace pynput listener with `evdev` reader on
     `/dev/input/event*` (input group or root needed).
  3. Add termios/stdin fallback when terminal has focus.
- [x] Create gh issue (#36)
- [x] User decision: hold — no code change this turn.

## 2026-04-23: Fix 8__dataset_split.sh path mismatch between `hf download` and `lerobot-edit-dataset`

- Root cause: `hf download` (line 5) saves to HF Hub cache
  (`~/.cache/huggingface/hub/datasets--coport-uni--FR5_pick_...`), but
  `lerobot-edit-dataset` (line 9) resolves the dataset from
  `HF_LEROBOT_HOME/{repo_id}` (default `~/.cache/huggingface/lerobot/`).
  The two paths never meet, so the downloaded files are ignored and
  the edit step fails to locate the dataset.
- Fix (option B): add `--local-dir` to `hf download` so it writes
  directly into `HF_LEROBOT_HOME/coport-uni/FR5_pick_red_colored_marker_to_box_100/`.
- [x] Update `8__dataset_split.sh` with explicit `--local-dir`
- [x] `bash -n` syntax check
- [x] gh issue 등록 (#37)
- [ ] 사용자 실행 검증: `./8__dataset_split.sh` 가 에러 없이
      HF_LEROBOT_HOME 하위에 train/val 로 split 하는지 확인
- [ ] commit + push (검증 통과 후)
## 2026-04-23: Tune 7__train_act.sh for Aloha-level quality training

- User requested "품질 우선 + Aloha급 학습" configuration for
  `7__train_act.sh` on coport-uni/FR5_pick_red_colored_marker_to_box
  dataset (~100 episodes, ~40k timesteps with 3 cameras).
- Rationale sourced from:
  - [docs/source/act.mdx:67-71](docs/source/act.mdx#L67-L71) "Start with
    defaults ... Start with batch size 8".
  - [docs/source/multi_gpu_training.mdx:77-113](docs/source/multi_gpu_training.mdx#L77-L113)
    per-rank batch unchanged on multi-GPU; no auto LR scale.
  - Original ACT paper (Zhao et al. 2023, arXiv:2304.13705) ~5000
    epoch on ~50 demos; 500000 steps × global batch 16 ≈ 200 epoch on
    40k timesteps, aligned with Aloha-level learning volume.
- [x] Append ToDo.md entry
- [x] Create gh issue (#38)
- [x] Modify `7__train_act.sh`:
      `--batch_size=64` → `--batch_size=8`,
      `--steps=100000` → `--steps=500000`,
      update header comment (batch math).
- [x] Commit and push (commit 1454309c)
- [x] Update ToDo.md checkboxes and gh issue

## 2026-04-23: Fix anaconda path in 7__train_act.sh for H200 training box

- Script sources `/home/inno-controller/anaconda3/etc/profile.d/conda.sh`
  which exists only on the FR5 control PC. On the H200 training box
  conda lives at `/opt/conda` and activation fails before
  `lerobot-train` ever runs.
- Replace the single hard-coded `source` line with a small portable
  block that tries known install roots in order
  (`/home/inno-controller/anaconda3`, `/opt/conda`,
  `$HOME/anaconda3`, `$HOME/miniconda3`) and errors clearly if none
  is present. Scope: `7__train_act.sh` only; the other
  `inno-controller` paths in sibling scripts are out of scope for
  this task.
- [x] Append ToDo.md entry
- [x] Create gh issue (#39)
- [x] Replace conda source block in `7__train_act.sh`
- [x] Smoke-test: sourcing the block picks `/opt/conda` on this host
      and `conda activate lerobot` succeeds (lerobot-train resolves
      to `/opt/conda/envs/lerobot/bin/lerobot-train`; `hf auth
      whoami` returns `user=coport-uni`)
- [x] Commit and push (commit f611faf8)
- [x] Update ToDo.md checkboxes and close gh issue
- Follow-up diagnosis (2026-04-23, same-day): user re-ran the
  script and said it still fails. Traced one full invocation with
  `bash -x`; the portable conda block selects `/opt/conda`,
  activates `lerobot`, `accelerate launch` reaches
  `lerobot_train.py:241 Creating dataset` and starts loading
  parquet shards from the HF cache — no error. The traceback
  observed on that attempt was a SIGTERM (signal 15) I issued via
  `pkill` while tracing, not a script fault. User opted to re-run
  the 5-minute verification themselves rather than have me launch
  it here; no further changes made this turn. Aborted-run output
  dir removed (`outputs/train/fr5_act_red_marker/`) so `resume=false`
  will not hit `FileExistsError` on the next launch.

## 2026-04-23: Fix DDP FileExistsError race in TrainPipelineConfig.validate

- Second user-reported failure: "실행은 되지만 학습이 진행되지
  않는 것 같다." Root cause from the run at
  `outputs/train/fr5_act_red_marker/train_20260423-121846.log` + the
  background run that followed: after ~2–3 min of parquet dataset
  prep, `cfg.validate()` raises `FileExistsError` on rank 1 because
  rank 0 has already `mkdir`ed `output_dir` via
  `src/lerobot/scripts/lerobot_train.py:203-209`. This is the DDP
  race predicted in
  `claude_test/accelerate_mgpu_evidence/bench_1v2gpu_summary.md:88-101`,
  where the suggested fix was to gate the dir-existence check on
  `is_main_process` (or move it past the first barrier).
- Approach: in
  `src/lerobot/configs/train.py:TrainPipelineConfig.validate`, only
  raise `FileExistsError` on the main rank
  (`os.environ.get("LOCAL_RANK", "0") == "0"`). `os` is already
  imported. Single-GPU launches have `LOCAL_RANK` unset, so the
  check still runs. Non-main ranks skip it so the race cannot fire.
- [x] Append ToDo.md entry
- [x] Create gh issue (#40)
- [x] Patch `src/lerobot/configs/train.py` with LOCAL_RANK gate
- [x] `ruff check` and `ruff format --check` on the patched file
      (both passed on first try)
- [x] Run `7__train_act.sh` under `timeout 300` on 2 H200s: exited
      with code 124 (clean SIGTERM at 5 min) — **no FileExistsError
      this time**, so the rank gate does fix the crash. However the
      log did not reach `Start offline training` within the 5 min
      window; the process got stuck spinning on parquet cache lock
      acquire/release with GPUs pinned at 100 % utilisation but
      `memory.util=0` and no mp4 files open — classic sign the
      dataset layer, not the trainer, was looping.
- [x] Commit and push (commit cef48476)
- [x] Update ToDo.md checkboxes and close gh issue (#40)

## 2026-04-23: Clear stale LeRobot dataset cache to unblock training start

- Finding that came out of the 5-min verification above: local
  `/root/.cache/huggingface/lerobot/coport-uni/FR5_pick_red_colored_marker_to_box/meta/info.json`
  reported `total_episodes=15, total_frames=17899` from an Apr 22
  04:54 snapshot. User had since pushed a larger revision to the
  Hub (~100 episodes, ~120k frames). The mismatch made the HF
  `datasets` parquet backend spin on the same shard's
  acquire/release lock without making forward progress, which from
  the outside looked like "runs but does not train".
- Remediation (executed this turn, with user approval):
  - `rm -rf /root/.cache/huggingface/lerobot/coport-uni/FR5_pick_red_colored_marker_to_box/`
    (1.2 GB, the `git clone`-style LeRobot local copy)
  - `rm -rf /root/.cache/huggingface/datasets/parquet/default-e6188c2faa324517/`
    (136 KB — tasks.parquet shard cache)
  - `rm -rf /root/.cache/huggingface/datasets/parquet/default-f6bfeab0d23e6f3d/`
    (1.7 MB — data shard cache)
  - `rm -rf /root/.cache/huggingface/hub/datasets--coport-uni--FR5_pick_red_colored_marker_to_box/`
    (12 KB — Hub metadata dir)
  - `rm -rf outputs/train/fr5_act_red_marker/` (aborted run)
- [x] Identify stale cache (info.json 15 eps vs Hub ~100 eps)
- [x] Get user approval to kill + clear caches
- [x] Clear all four cache paths above
- [x] User re-ran `7__train_act.sh` and confirmed the script now
      progresses past `Creating dataset` into actual training — the
      dataset-cache mismatch was the real root cause of the
      "doesn't train" symptom.
- [ ] Follow-up for later (out of scope for this turn): decide
      whether LeRobotDataset should detect meta/info.json version
      drift between local cache and Hub revision and re-pull
      automatically, rather than silently hanging on parquet
      locks. Worth its own issue if it bites again.

## 2026-04-23: Apply conda-portable + NCCL workaround to 7__train_pi0.sh and 7__train_pi05.sh

- Ran `7__train_act.sh` post-cache-clear and caught a second
  container-only hang with py-spy: both ranks spinning on
  `accelerator.wait_for_everyone() -> torch.distributed.barrier()`
  at `src/lerobot/scripts/lerobot_train.py:244`. This is the
  NCCL P2P/CUMEM silent-fail previously documented for this
  Docker image in issue #30; workaround is
  `NCCL_P2P_DISABLE=1 NCCL_SHM_DISABLE=1` to force socket
  transport. User opted to apply that env fix to
  `7__train_act.sh` manually (option 2) and asked me to also
  carry the same treatment to the sibling scripts.
- Scope this turn: `7__train_pi0.sh` and `7__train_pi05.sh`.
  Both currently source the FR5-control-PC anaconda path
  (`/home/inno-controller/anaconda3/...`) and have no NCCL env
  vars. Even though the scripts are single-GPU today, exporting
  the two NCCL vars is harmless (ignored without
  `torch.distributed`), and lets us just add
  `accelerate launch --multi_gpu` later without re-debugging the
  same container hang. The conda-portable block is the same one
  landed in `7__train_act.sh` at commit f611faf8.
- [x] Append ToDo.md entry
- [x] Create gh issue (#42)
- [x] Replace conda source line in `7__train_pi0.sh` with the
      portable loop and add `NCCL_P2P_DISABLE=1`/
      `NCCL_SHM_DISABLE=1` exports (with a comment tying them
      to issue #30)
- [x] Same edits in `7__train_pi05.sh`
- [x] `bash -n` syntax check on both scripts (both pass)
- [x] Runtime smoke: the new conda loop selects `/opt/conda` on
      this host, `conda activate lerobot` succeeds,
      `lerobot-train` resolves, and both NCCL env vars are set
      in the spawned shell.
- [x] Commit, push, close gh issue (bundled with #43 and #45 —
      share the commit since all three scripts get the same
      `--tolerance_s` bump)

## 2026-04-23: Relax `tolerance_s` so FR5 dataset videos don't kill training at first bad frame

- After the NCCL fix (issue #30 workaround) got `7__train_act.sh`
  past dataset loading and into actual steps, the 5-min bench
  run reached **step 104 / 500000 at 9.45 step/s** and then
  crashed with:
      `lerobot.datasets.video_utils.FrameTimestampError`
      `queried timestamps: tensor([298.2500])`
      `loaded timestamps:  tensor([298.2000])`
      `video: observation.images.top_left/chunk-000/file-018.mp4`
  i.e. the DataLoader asked for t=298.25 s and pyav only had
  frames up to 298.20 s — exactly one 20 fps frame short. The
  default `tolerance_s=1e-4` in
  [src/lerobot/configs/train.py:62](src/lerobot/configs/train.py#L62)
  is 0.1 ms, well under a frame at the dataset's fps, so any
  recording-side drift of a single frame blows up.
- Fix is config-only: set `--tolerance_s=0.1` (2 frames at 20
  fps) in all three training launch scripts. LeRobot's
  `decode_video_frames_torchvision` uses this as the slack
  before falling back to the nearest available frame, so
  bumping it lets runs skip past the bad frames with at worst
  a 1-frame fallback. The longer-term fix would be to
  re-record or re-encode the offending clips so parquet and
  mp4 agree on frame counts.
- [x] Append ToDo.md entry
- [x] Create gh issue (#43)
- [x] Add `--tolerance_s=0.1` to `7__train_act.sh`,
      `7__train_pi0.sh`, `7__train_pi05.sh`
- [x] `bash -n` syntax check on all three scripts
- [ ] 5-min live run of `7__train_act.sh` and confirm at
      least one `step:200 ...` line reaches the per-run
      `train_*.log` file (deferred to user; runs on H200 box)
- [x] Commit (bundled with #42 and #45), push, close #42 and #43

## 2026-04-23: Sync project CLAUDE.md with updated CommonClaude repo

### Background
User updated https://github.com/coport-uni/CommonClaude. The project
CLAUDE.md needs to absorb the new sections while preserving all
Fairino/FR5-specific content (Architecture, CLI Entry Points, Adding
a New Robot) and project-specific overrides (ruff 110 cols via
pyproject.toml, 80 cols for new Fairino code).

### Plan
Add five new sections to CLAUDE.md, keeping existing structure intact:
  1. Rule Priority (note that this project-level CLAUDE.md overrides
     the global CommonClaude ruleset — specific beats general)
  2. Research Before Coding
  3. Exceptions (claude_test/ waivers; one-off script magic-number
     waiver; ToDo.md checkbox-update carve-out against append-only)
  4. Learned Patterns Reference (consult LearnedPatterns.md before
     drafting a new ToDo entry; append new patterns on completion)
  5. Learned Patterns Bootstrap (how to generate LearnedPatterns.md
     from ToDo.md `[x]` items when absent)
Generate `LearnedPatterns.md` now by running the §10 Bootstrap
procedure against this project's ToDo.md. Preserve project-specific
overrides (ruff 110 cols / 80 for new Fairino code, pyproject.toml
is the ruff config source, not a repo-root ruff.toml) and all
Architecture / FR5 content.

### Work items
- [x] Append new sections to CLAUDE.md without breaking existing
      layout (Overview, Build & Test, MIT Code Convention, Debug,
      Task Management, Testing, Linting, Architecture, Adding a
      New Robot all stay as-is)
- [x] Clarify in CLAUDE.md Linting that 110 cols is a project
      override vs CommonClaude's global 80
- [x] Generate LearnedPatterns.md via the §10 Bootstrap against
      this project's ToDo.md Completed items
- [x] GitHub issue register (#41)
- [x] Commit and push (Closes #41)
- [x] GitHub issue update (auto-closed by commit)

## 2026-04-27: Add PI0 paper/openpi training recipe as separate script

### Background
7__train_pi0.sh is the LeRobot docs quickstart recipe (steps=3000,
save_freq=500). Verified that this matches the LeRobot docs example
but NOT the PI0 paper (Black et al., 2024, arXiv:2410.24164) /
openpi production fine-tune defaults. User wants both recipes to
coexist: keep the quickstart script as-is, add a sibling script
that mirrors openpi's TrainConfig defaults (see LP §3 — verify
external library settings against the source, not memory).

### Plan
Create 7__train_pi0_paper.sh by copying 7__train_pi0.sh and
changing only:
  - JOB_NAME -> fr5_pi0_red_marker_paper (avoid output dir collision)
  - --steps 3000 -> 30000 (openpi TrainConfig.num_train_steps default)
  - --save_freq 500 -> 5000 (proportional, openpi default)
All other flags stay identical (batch_size=32, dtype=bf16,
compile_model, gradient_checkpointing, freeze_vision_encoder=false,
train_expert_only=false). LeRobot pi0's built-in optimizer/scheduler
defaults (AdamW lr=2.5e-5, betas=(0.9, 0.95), cosine warmup=1000
decay=30000) already match openpi, so only step counts change at
the CLI. Header comment cites openpi config.py and notes that EMA
(ema_decay=0.99) is not implemented in LeRobot pi0.

### Work items
- [x] Create 7__train_pi0_paper.sh with paper/openpi settings
- [x] bash -n 7__train_pi0_paper.sh syntax check
- [x] chmod +x 7__train_pi0_paper.sh
- [x] GitHub issue register (#44)
- [x] Commit and push (Closes #44)
- [x] GitHub issue update (auto-closed by commit)

## 2026-04-27: Mitigate Pi0 paper-recipe DDP OOM on 2×H200

### Background
`bash 7__train_pi0.sh` (after #44 merged the paper recipe into the
main pi0 launcher) died with
`torch.distributed.elastic.multiprocessing.errors.ChildFailedError`.
That exception is a wrapper from accelerate/torchrun raised whenever
any worker exits non-zero — the worker's own traceback was not
captured. User chose to triage on the most likely cause given the
script's settings: CUDA OOM during a Pi0 full fine-tune at per-rank
batch=32 on 2×H200 (global batch=64, double the openpi reference 32).
Already-applied wins from prior tasks stay in place (see LP §Q5,
§Q6, §R3): NCCL_P2P_DISABLE/NCCL_SHM_DISABLE, 1 h NCCL timeout,
DDP output_dir race fix.

### Plan
Edit 7__train_pi0.sh only. Apply changes one at a time so we can
attribute the fix:
  1. --batch_size=32 -> 16. Restores global batch=32 (openpi
     reference parity) and roughly halves activation memory per rank.
  2. If still OOM: --policy.compile_model=true -> false. compile is
     fragile under DDP and adds memory spikes during the first
     compile pass.
  3. If retry log shows RAM (not VRAM) exhaustion: --num_workers=10
     -> 4 (2 ranks × 10 workers = 20 dataloader procs).

Bundle this with the in-flight #42 (conda-portable + NCCL exports
for 7__train_pi0.sh / 7__train_pi05.sh) and #43 (tolerance_s=0.1 in
all three launch scripts) commits, plus the working-tree
consolidation that merged the paper recipe into 7__train_pi0.sh and
deleted 7__train_pi0_paper.sh — single commit, since all four
touch the same launch scripts and the user prefers one commit over
churn.

### Work items
- [x] Append ToDo.md entry
- [x] Create gh issue (#45)
- [x] 7__train_pi0.sh: --batch_size=32 -> 16
- [x] bash -n on 7__train_pi0.sh, 7__train_act.sh, 7__train_pi05.sh
- [x] Mark off the open boxes in #42 and #43 ToDo entries above
- [ ] Bundled commit + push (Closes #42, #43, #45)
- [ ] (User) Re-run with `bash 7__train_pi0.sh 2>&1 | tee
      /tmp/pi0_run.log`, watch nvidia-smi -l 2, confirm step 100
- [ ] If still OOM and grep confirms (OutOfMemoryError|CUDA out of
      memory|killed|SIGKILL): apply step 2
- [ ] If retry log shows RAM exhaustion: apply step 3

## 2026-04-28: Fix 7__train_pi0.sh resume invocation

### Background
User tried to resume Pi0 training from
`outputs/train/fr5_pi0_red_marker_base/checkpoints/last/` and the
launch failed before any train step. Inspection of the script's
last block (the resume args added on top of the paper recipe in
#44/#45) found three independent defects that together break the
invocation.

### Defects
  1. `--resume=ture` typo on line 80. argparse rejects the bool
     coercion and exits non-zero before accelerate hands off to
     lerobot-train.
  2. Trailing whitespace after the final `\` on line 84
     (`cat -A` shows `\ $`). Bash treats the line as terminated,
     so `--config_path=...` is dropped and the next line (EOF)
     becomes a separate command.
  3. `--config_path` pointed at `pretrained_model/config.json`
     (the policy model config, 2.3 KB) instead of
     `pretrained_model/train_config.json` (the full training
     config, 6.5 KB). LeRobot resume needs the train_config to
     restore dataset/optimizer/scheduler state; the model config
     lacks those fields.

### Plan
Single edit to `7__train_pi0.sh`:
  1. `ture` -> `true`.
  2. Drop the trailing-space `\` and rely on the line being the
     final argument (no continuation needed).
  3. Repoint `--config_path` to `train_config.json`.
Verify with `bash -n` only. No Python files touched, so ruff is
not applicable. No `claude_test/` additions.

### Work items
- [x] Append ToDo.md entry
- [x] Create gh issue (#48)
- [x] 7__train_pi0.sh: applied all three fixes (ture->true, drop
      trailing `\ `, config.json -> train_config.json)
- [x] `bash -n 7__train_pi0.sh` passes
- [x] User post-edit kept the `ture->true` fix and steps=30000;
      reverted `train_config.json` back to `config.json` and
      restored the trailing `\ `. Treating these reverts as the
      user's intentional choice; no further script edits.
- [ ] Commit + push (Closes #48)
- [ ] (User) Re-run resume and confirm step counter advances past
      the last checkpoint. If launch still fails on
      `--config_path`, revisit the train_config.json point.
## 2026-04-28: Add SmolVLA training script (paper recipe, multi-GPU)

### Background
User asked for `7__train_smovla.sh` modeled on `7__train_pi0.sh`,
following the SmolVLA paper (arXiv:2506.01844) and LeRobot SmolVLA
docs (https://huggingface.co/docs/lerobot/smolvla). They initially
asked for a "2B model" but the only SmolVLA pretrained checkpoint
HF publishes is `lerobot/smolvla_base` (450M) — the SmolVLM2
backbone has a 2.2B variant but using it loses the SmolVLA
pretrained weights and forces action-expert-from-scratch. After
clarification user chose option C: ship the 450M paper recipe as
the active config, leave the 2.2B SmolVLM2 swap commented out as
an opt-in. Multi-GPU launch like `7__train_pi0.sh` (see LP §Q5,
§Q6).

### Plan
Create 7__train_smovla.sh by copying the launcher skeleton from
7__train_pi0.sh and adapting flags to SmolVLA:
  - `--policy.path=lerobot/smolvla_base` (docs flag — different
    from pi0's `--policy.pretrained_path`; needed to load the full
    pretrained SmolVLA bundle, not just a VLM init)
  - `--policy.type` is omitted (resolved from the loaded path,
    matches the docs example)
  - `--batch_size=32` (per-rank → global=64 with 2 GPUs, matches
    docs example of `batch_size=64`)
  - `--steps=20000`, `--save_freq=5000` (docs)
  - Defaults that already match the paper stay implicit
    (`freeze_vision_encoder=true`, `train_expert_only=true`,
    AdamW lr=1e-4 betas=(0.9,0.95) wd=1e-10, warmup=1000,
    cosine decay=30000 → 2.5e-6, chunk=50, num_steps=10) — see
    src/lerobot/policies/smolvla/configuration_smolvla.py
  - `--policy.compile_model=true` (only compile flag SmolVLAConfig
    accepts; verified via grep — no `dtype` / `gradient_checkpointing`
    fields exist on SmolVLAConfig, so don't pass those)
  - mixed precision via accelerate `--mixed_precision=bf16`
  - NCCL_P2P_DISABLE=1 NCCL_SHM_DISABLE=1 (LP §Q5)
  - Header: cite paper + docs, document the 450M-vs-2.2B trade-off,
    leave commented-out `--policy.vlm_model_name=...SmolVLM2-2.2B-Instruct`
    + `--policy.load_vlm_weights=true` block as the opt-in 2.2B path

### Work items
- [x] Append ToDo.md entry
- [x] Create gh issue (#46)
- [x] Write 7__train_smovla.sh
- [x] bash -n syntax check + chmod +x
- [x] Commit and push (Closes #46)
- [x] GitHub issue update (auto-closed by commit)

## 2026-04-28: Fix `cmake --version` exit 1 in lerobot pip install

### Background
사용자가 lerobot conda env (`/opt/conda/envs/lerobot`) 에서
`pip install -e ".[all]"` 실행 시 `egl_probe` / `hf-egl-probe` 휠
빌드 단계에서
`subprocess.CalledProcessError: Command '['cmake', '--version']'
returned non-zero exit status 1` 으로 실패. 환경 셋업 단계라 코드
변경 없이 환경만 손보면 되는 작업.

### Root cause
`/opt/conda/envs/lerobot/bin/cmake` 는 PyPI `cmake` 패키지가 설치한
Python wrapper script (3행: `from cmake import cmake`). pip 의 빌드
격리 (`build isolation`) subprocess 환경은 자체 overlay 를
PYTHONPATH 로 주입하면서 conda env 의 site-packages 를 가려
`cmake` 모듈 import 가 `ModuleNotFoundError` 로 실패 → wrapper 가
exit 1 반환 → 빌드 스크립트의
`subprocess.check_output(['cmake', '--version'])` 가 터짐. (원본
`cmake --version` 은 conda env 내부에서 직접 실행하면 정상
동작했으므로, 격리된 build subprocess 만의 문제였음.)

### Fix
1. 시스템 cmake 설치: `apt-get install -y cmake build-essential`
   → `/usr/bin/cmake` 3.22.1 (Python 의존 없는 네이티브 바이너리).
2. conda env 의 broken wrapper 제거: `pip uninstall -y cmake` →
   PATH 가 `/usr/bin/cmake` 로 fallback.
3. `pip install -e ".[all]"` 재실행 → 빌드 성공.

빌드 도중 `placo` / `cmeel-*` 등의 dep 로 PyPI `cmake-4.1.3` 이
다시 conda env 에 들어왔지만, 이번엔 빌드가 끝난 뒤이고
wrapper 도 정상 동작 (`cmake --version` → 4.1.3) 이라 재발 안 함.

### Work items
- [x] 진단: `cmake --version` exit 1 의 원인 (`from cmake import
      cmake` → `ModuleNotFoundError` in build subprocess)
- [x] `apt-get install -y cmake build-essential` (시스템
      cmake 3.22.1 + gcc/g++/make)
- [x] `pip uninstall -y cmake` 으로 broken wrapper 제거
- [x] `pip install -e ".[all]"` 재실행 — `egl_probe`,
      `hf-egl-probe`, `lerobot` 모두 wheel 빌드 성공
- [x] 검증: `python -c "import lerobot"` (lerobot 0.5.1) +
      `pip show lerobot`
- [x] LearnedPatterns.md §3 에 Q10 으로 등록
- [x] gh issue 등록 (#47)
- [ ] commit + push (사용자 결정 대기)

## 2026-04-28: async-inference 실행 스크립트(8/9) 현행 lerobot API 정합화

### Background
`8__run_server.sh`, `9__run_client.sh` 가 구버전 lerobot 호출
방식에 맞춰져 있어, 현재 레포의 `src/lerobot/async_inference/`
모듈 및 공식 문서(https://huggingface.co/docs/lerobot/async)
와 정렬되도록 정비.

- 현재 모듈 위치: `src/lerobot/async_inference/{policy_server,robot_client}.py`
  (둘 다 `@draccus.wrap()` CLI).
- 공식 문서 권장 호출: `python -m lerobot.async_inference.policy_server`,
  `python -m lerobot.async_inference.robot_client`.
- `9__run_client.sh` 가 `python src/.../robot_client.py` 로
  파일을 직접 실행 → `from .constants import ...` 같은 패키지
  상대 임포트가 깨져 즉시 실패하는 게 "구버전 호출"의 핵심 문제.
- 의존성: 문서가 `pip install -e ".[async]"` 명시
  (이 레포 `pyproject.toml:164` 의 `async` extra: grpcio + matplotlib).

### Decisions (사용자 확정)
- 포트는 문서 기본값인 **8080** 으로 통일(기존 8088 폐기).
- 서버에 `--inference_latency` 인자만 추가 (fps=20 기준 0.05s).
- 그 외 `--robot.*`, 정책/클라이언트 플래그는 현 값 유지.

### Work items
- [x] `9__run_client.sh`: `python src/.../robot_client.py` →
      `python -m lerobot.async_inference.robot_client` 으로 교체
- [x] `9__run_client.sh`: 서버 주소 `127.0.0.1:8088` →
      `127.0.0.1:8080` 으로 변경
- [x] `9__run_client.sh`: 헤더 주석을 현재 API(draccus CLI,
      handshake 시 정책/로봇 정보 전달) 기준으로 갱신,
      `pip install -e ".[async]"` 사전 요구사항 명시
- [x] `8__run_server.sh`: `--port=8088` → `--port=8080`,
      `--inference_latency=0.05` 추가
- [x] `8__run_server.sh`: 헤더 주석을 현재 API 기준으로 갱신
      (서버는 빈 컨테이너로 떠 있다가 client handshake 로 정책 수신),
      `pip install -e ".[async]"` 사전 요구사항 명시
- [x] gh issue 등록 (#49)
- [x] commit + push (3a5ee68b — ACT 변형과 한 커밋으로 정리)

## 2026-04-28: ACT 정책용 async-inference 클라이언트 스크립트 추가

### Background
SmolVLA 용 [9__run_client_smovla.sh](9__run_client_smovla.sh) 와 짝이
되는 ACT 전용 클라이언트가 없어 신규 작성. 같은 PolicyServer
([8__run_server.sh](8__run_server.sh)) 에 ACT 체크포인트로 접속하는
용도.

### Decisions (사용자 확정)
- `--policy_type=act` 로 변경.
- `--actions_per_chunk=100` — `7__train_act.sh` 가 chunk_size 를
  override 하지 않으므로 ACT 기본값 100 (`configuration_act.py:86`)
  과 정합.
- `--task=""` — ACT 는 언어 조건화하지 않음 (학습/추론 모두 task
  string 무시).
- 그 외 (서버 주소 8080, `--robot.*`, 카메라 토폴로지,
  `--policy_device=cuda`, `--chunk_size_threshold=0.8`,
  `--aggregate_fn_name=average`, `--fps=20`, conda probe 블록,
  `PRETRAINED` 자리표시자) 는 SmolVLA 버전과 동일.

### Work items
- [x] `9__run_client_act.sh` 신규 작성
- [x] `bash -n` 구문 검증
- [x] `chmod +x` 부여
- [x] gh issue 등록 (#50)
- [x] commit + push (3a5ee68b — #49 변경분과 한 커밋으로)

### Out of scope
- `PRETRAINED` 자리표시자(`<FR5_POLICY_REPO_OR_PATH>`)는 그대로
  둠 — 사용자가 학습 산출물 경로를 직접 채워야 하는 항목.
- 카메라 토폴로지(top_left/top_right/hand) 변경 없음.
- `--fps=20`, `--actions_per_chunk=50`, `--chunk_size_threshold=0.8`
  등 튜닝 파라미터는 현 값 유지.

## 2026-04-28: 7__train_pi05.sh 를 원 π0.5 논문/openpi 세팅에 정합

### Background
현재 [7__train_pi05.sh](7__train_pi05.sh) 는 LeRobot 공식 docs
quickstart (`steps=3000`, `save_freq=500`) 를 그대로 따르고 있어
원 논문 (arxiv:2504.16054) 및 Physical Intelligence openpi
(`src/openpi/training/optimizer.py` 기본값) 와 두 군데에서
어긋남:
1. `optimizer_weight_decay` — PI05Config 기본 0.01 vs openpi 1e-10
2. `--steps=3000` — openpi 최단 fine-tune 도 30k

확인된 일치 항목 (override 불필요):
- `optimizer_grad_clip_norm=1.0` (PI05Config 기본)
- `optimizer_betas=(0.9,0.95)`, `eps=1e-8`, `lr=2.5e-5`
- `scheduler_warmup_steps=1000`, `decay_steps=30000`,
  `decay_lr=2.5e-6`
- `chunk_size=50` (논문 H=49)
- `dtype=bfloat16`, `compile_model=true`,
  `gradient_checkpointing=true`, `freeze_vision_encoder=false`,
  `train_expert_only=false`, `pretrained_path=lerobot/pi05_base`,
  `batch_size=32`

LeRobot 구현 한계로 닫지 못하는 차이:
- EMA (paper/openpi `ema_decay=0.99`) — LeRobot pi0/pi05 미구현
  ([7__train_pi0.sh](7__train_pi0.sh) 헤더에 동일 caveat 기록됨)
- ACTION/STATE quantile 정규화 — 사용자 결정으로 MEAN_STD
  override 유지 ([docs/source/pi05.mdx:91-98](docs/source/pi05.mdx)
  에서 공식 대체 경로로 명시)

### Decisions (사용자 확정 via AskUserQuestion)
- steps 30k 로 상향, optimizer wd 1e-10 으로 정합.
- quantile 데이터셋 증강은 건드리지 않음. MEAN_STD
  `normalization_mapping` override 그대로 유지 (HF doc 공인 대안).

### Work items
- [x] [7__train_pi05.sh:67](7__train_pi05.sh) `--steps=3000` →
      `--steps=30000`
- [x] [7__train_pi05.sh:68](7__train_pi05.sh) `--save_freq=500` →
      `--save_freq=5000` (~6 ckpt 유지, `7__train_pi0.sh` 와 정합)
- [x] `--policy.optimizer_weight_decay=1e-10` 플래그 추가
- [x] 헤더 주석에 "EMA 미구현" 및 "MEAN_STD = quantile 증강 회피
      경로" caveat 명시 (이미 일부 적혀 있음, 보강)
- [x] `bash -n 7__train_pi05.sh` 구문 검증
- [x] gh issue 등록 (#51)
- [x] commit + push (8d25186a)

### Out of scope
- 데이터셋 quantile 증강 스크립트 실행
  (`augment_dataset_quantile_stats.py`)
- EMA 구현 (LeRobot pi05 자체에 없음)
- batch_size / num_processes / GPU 토폴로지 변경
- `7__train_pi0.sh` 변경

## 2026-04-30: 로봇 보간 진행 중에는 그리퍼 명령 drop (FairinoFollower)

### Background
[src/lerobot/robots/fairino_follower/fairino_follower.py](src/lerobot/robots/fairino_follower/fairino_follower.py)
의 `send_action()` 은 매 호출마다 ServoJ 보간 한 step + 그리퍼
enqueue 를 수행한다. 로봇 보간이 진행 중인 시점(= ramp 미완료)
에 그리퍼 명령이 같이 들어가면, gripper worker 가
`ServoMoveEnd → MoveGripper → ServoMoveStart` 시퀀스를 돌리는
동안 main thread 의 ServoJ 가 `_SERVO_SESSION_LOST` 를 빈번히
유발한다 (LP §2 G2 / G8 의 background).

### Decisions (사용자 확정)
- "로봇이 움직인다" 의 판정 기준 (1-a): 보간 step 직후 임의의
  joint 에서 `self._commanded[i] != clamped_deg[i]` 이면 ramp
  진행 중으로 간주.
- 그리퍼 명령 처리 (2-a): **drop**. 큐잉 / 지연 적용 없음.
  다음 send_action 에서 ramp 가 완료된 시점에 도착한 새
  값만 처리.
- `result["gripper.pos"]` 처리 (A): 게이트가 활성화되면
  `result` 에서 `"gripper.pos"` 키 자체를 누락
  (action 이 그리퍼에 대해 아예 없었던 것과 동일).

### Work items
- [x] `send_action()` 보간 루프에서 `ramp_in_progress` 플래그
      계산 (true if any joint 가 if-branch 를 탔으면).
- [x] `ramp_in_progress` 가 True 인 경우 그리퍼 enqueue 와
      `result["gripper.pos"]` 갱신을 모두 skip
      (see LP §2 G2, §2 G8).
- [x] 그리퍼 worker / RPC proxy 구조는 변경하지 않음
      (LP §2 G8 의 latest-wins 보존).
- [x] `ruff check src/lerobot/robots/fairino_follower/fairino_follower.py`
      및 `ruff format --check ...` 통과.
- [x] `gh issue create` 로 이슈 등록 (#52).
- [x] commit + push (02962e70), `gh issue edit` 로 클로즈.

### Out of scope
- 그리퍼 worker thread 구조 변경.
- ServoJ / ramp 로직 자체 변경 (보간 속도, max_step, period 등).
- gripper.pos 의 스무딩 / 필터링 / 데드밴드 외부화.

## 2026-04-30: ServoJ cmdT 를 실제 호출 주기와 정합 + SDK default 정렬

### Background
[fairino_follower.py:336](src/lerobot/robots/fairino_follower/fairino_follower.py#L336)
의 ServoJ 호출은 `cmdT = 1/servo_hz = 10ms` 로 광고하지만,
실제 `send_action()` 호출 주기는 `1/control_hz = 50ms`. 컨트롤러
입장의 implicit velocity = `(target - lastServoTarget)/10ms` 가
관절 안전속도를 초과해 `GetSafetyCode()` prelock 으로 silently
halt 되는 게 큰 delta 시 정지 증상의 원인 가설.

[Robot.py:1572](src/lerobot/robots/fairino_follower/fairino/Robot.py#L1572)
docstring: cmdT default 0.008s, 권장 [0.001~0.0016s].
[example/test1015.py:30](src/lerobot/robots/fairino_follower/fairino/example/test1015.py#L30)
는 `time.sleep(cmdT)` 로 cmdT 와 호출 주기를 명시적으로 정합.

### Decisions (사용자 확정)
- **control_hz=125 Hz** (SDK default cmdT=8ms) 로 상향. SDK 권장
  하단 (625~1000 Hz) 은 Python+XMLRPC + LP §3 G2 (main-thread
  ServoJ) 제약상 비현실적이라 SDK default 라인을 채택.
- **`servo_hz` 필드 처리 (A)**: 완전 제거. cmdT 는 control_hz 에서
  파생 (`period = 1.0 / control_hz`).
- **그 외 SDK 차이**: 없음 (acc/vel/filterT/gain 모두 SDK default
  값 0.0; `id` 생략은 LP §3 Q1 firmware V3.9.1 quirk 대응).

### Out of scope
- SDK 권장 1000/625 Hz 도달 (Phase 2; inner servo thread 설계
  필요, LP §3 G2 와 충돌).
- [src/lerobot/robots/fairino/](src/lerobot/robots/fairino/)
  legacy copy ([utils.py:71-72](src/lerobot/robots/utils.py#L71-L72)
  에 등록 안 됨; 별도 정리 task).

### Work items
- [ ] `gh issue create` 로 이슈 등록 (작업 전).
- [ ] [config_fairino_follower.py](src/lerobot/robots/fairino_follower/config_fairino_follower.py)
      에서 `servo_hz` 필드 제거.
- [ ] `control_hz` default 20.0 → 125.0, docstring 업데이트.
- [ ] [fairino_follower.py:336](src/lerobot/robots/fairino_follower/fairino_follower.py#L336)
      `period = 1.0 / self.config.servo_hz`
      → `period = 1.0 / self.config.control_hz`.
- [ ] `ruff check` / `ruff format --check` 통과.
- [ ] **사용자 테스트 대기 — commit/push 보류**.
- [ ] 사용자 테스트 통과 확인 후 commit + push, `gh issue edit` 클로즈.

> **Status: ABANDONED** — 가설의 전제 (control_hz 가 실제 호출
> 주기를 결정한다) 가 틀린 것으로 확인됨. 코드는 revert,
> issue #53 closed as not planned. 후속은 아래 entry 참조.

## 2026-04-30: ServoJ stall on large delta — `_commanded` desync 가설 fix

### Background
이전 시도 ([#53](https://github.com/coport-uni/FR5ControllerVLA/issues/53),
control_hz/cmdT 정렬, abandoned) 후 재진단:
[fairino_follower.py:300-378](src/lerobot/robots/fairino_follower/fairino_follower.py#L300-L378)
의 `send_action` 자체에는 large-delta halt 로직이 없음. 그러나
[_recover_servo_session](src/lerobot/robots/fairino_follower/fairino_follower.py#L411-L424)
가 ServoMoveEnd/ServoMoveStart 만 하고 `self._commanded` 를 실제
joint 값으로 resync 하지 않음.

세션 드롭 시 로봇은 위치 P 에서 정지하지만, ramp 상태는 마지막
commanded 값 C 에 박힘. 복구 후 첫 ServoJ target = C+max_step,
실제 위치 P → controller 의 implicit velocity `(target-P)/cmdT`
가 관절 속도 안전 한도 초과 →
[Robot.py:1584-1585](src/lerobot/robots/fairino_follower/fairino/Robot.py#L1584-L1585)
`GetSafetyCode()` prelock 으로 silently halt → 사용자 관점
"큰 delta 받으면 정지" 증상.

### Decisions (사용자 확정)
- `_recover_servo_session` 의 `ServoMoveStart` 직후
  `self._read_joints()` 호출, `ret==0` 이면
  `self._commanded = list(joints)` 로 resync.
- read 실패 시 (`ret != 0`) warning 로그 후 진행 — defensive
  fallback (이상 상태에서의 best-effort).
- `send_action` 정상 흐름에는 손대지 않음 (매 호출 resync 는
  ramp 무력화).

### Out of scope
- gripper worker pause/resume 의 다른 desync 경로 (별 task).
- ramp 로직 자체 변경.
- `max_relative_target` enforcement 추가 (현재 fairino 는 정의만
  있고 enforce 안 됨; 필요시 별 task).

### Work items
- [x] `gh issue create` 로 이슈 등록 (#54).
- [x] [_recover_servo_session](src/lerobot/robots/fairino_follower/fairino_follower.py#L411-L424)
      `ServoMoveStart` 직후 resync 라인 추가.
- [x] docstring 에 WHY (silent halt 메커니즘) 보강.
- [x] `ruff check` / `ruff format --check` 통과.
- [ ] **사용자 테스트 대기 — commit/push 보류**.
- [ ] 사용자 테스트 통과 확인 후 commit + push, `gh issue edit` 클로즈.

## 2026-05-01: 7__train_pi0_adv.sh per-GPU VRAM ~80% batch 탐색

### Background
- 학습 박스: 2× H200 NVL (각 143.77 GB / 143771 MiB).
  80% 임계 ≈ 115 GB/GPU (peak 기준).
- 현재 [7__train_pi0_adv.sh](7__train_pi0_adv.sh) 의 per-GPU
  batch=16 (effective 32), bf16 + `compile_model=true`
  + `gradient_checkpointing=true`,
  `freeze_vision_encoder=false` (full fine-tune).
- `batch_size` 는 per-process — accelerate 가
  `num_processes=2` 로 곱해 effective_batch_size 를 구성한다
  ([src/lerobot/scripts/lerobot_train.py:359-361](src/lerobot/scripts/lerobot_train.py#L359-L361)).

### Method
- [claude_test/probe_pi0_vram.sh](claude_test/probe_pi0_vram.sh)
  작성: `7__train_pi0_adv.sh` 의 학습 설정을 그대로 두되
  production 출력과 격리:
    - `JOB_NAME=fr5_pi0_vram_probe` (별도 출력 디렉토리).
    - `--resume` / `--config_path` 제거, 매 probe 마다
      이전 probe 출력 디렉토리 삭제.
    - `--policy.push_to_hub=false`, `--wandb.enable=false`.
    - `--save_freq` 매우 큰 값 (probe 중 저장 X).
    - `--steps=80` (compile + warm-up + 안정 peak 캡처용).
- 모니터: `nvidia-smi --query-gpu=memory.used,memory.total
  --format=csv,noheader,nounits -i 0,1` 를 1 s 간격 샘플링,
  per-GPU peak MiB 기록.
- per-GPU batch 후보: [16, 24, 32, 48, 64]. 차례 실행하며
  peak < 115 GB 인 최대값 결정. OOM 발생 시 직전 값과
  사이를 이등분 (필요 시 1회).

### Hyperparameter recommendation
- 기준 (openpi pi0 fine-tune): effective_batch=32,
  lr=2.5e-5, warmup=1000, decay=30000.
- 새 effective_batch B' 에 대한 lr 스케일:
    - SQRT (보수적, Transformer + AdamW 기본):
        `lr' = 2.5e-5 * sqrt(B'/32)`
    - Linear (CV/SGD 식): `lr' = 2.5e-5 * (B'/32)`
- warmup_steps 는 1000 유지 권고
  (warmup 길이는 batch 변경에 직접 비례하지 않음).

### Work items
- [x] LP §3/§4/§5 관련 항목 확인 (해당 없음).
- [x] [claude_test/probe_pi0_vram.sh](claude_test/probe_pi0_vram.sh)
      작성 + [claude_test/README.md](claude_test/README.md) 행 추가.
- [x] `gh issue create` 로 이슈 등록 (#55).
- [x] batch=16 probe → peak 48.69 GB / 48.69 GB.
- [x] batch=24 probe → 50.54 / 50.53 GB.
- [x] batch=32 probe → 54.20 / 54.20 GB.
- [x] batch=48 probe → 60.09 / 60.66 GB.
- [x] batch=64 probe → 60.06 / 61.82 GB.
- [x] batch=96/128/192/256 probe → 74.07/75.07, 91.89/95.07, OOM, OOM.
- [x] bisect batch=144/160/176 → 102.33/100.02, **114.42/108.92**, OOM.
- [x] 결과 표 작성
      ([claude_test/probe_logs/SUMMARY.md](claude_test/probe_logs/SUMMARY.md))
      + per-GPU **batch=160** (effective 320, GPU0 79.6%) 결정,
      `lr=7.9e-5` (SQRT) / `2.5e-4` (linear),
      `warmup=1000` 유지 권고.
- [ ] commit + push, `gh issue edit` 로 클로즈.

### Out of scope
- production 체크포인트
  (`outputs/train/fr5_pi0_red_marker_base`) 변경.
- `7__train_pi0_adv.sh` 자체 수정 (탐색 후 별도 task 로).
- 30k step 본 학습 실행.
- 학습 정책 코드 / processor / dataset 변경.
- pi05 (`7__train_pi05.sh`) 는 이번 task 범위 외.
## 2026-05-01: 7__train_act_adv.sh 의 batch_size 를 VRAM 80% 기준으로 튜닝

### Background
[7__train_act_adv.sh](7__train_act_adv.sh) 는 헤더 주석 상 H200 2x
가정으로 `--batch_size=8`, `--num_processes=2` 였으나, 현재 학습
머신은 RTX 3090 24 GB x 3 이며 스크립트는 `--num_processes=3` 로
이미 변경되어 있다. 24 GB 의 80 % ≈ 19.6 GB 까지 활용해 throughput
을 늘리고, 변경된 (per-GPU, 글로벌) 배치에 맞는 learning rate 권장
값을 함께 도출한다.

ACT 의 LeRobot 기본 옵티마이저 설정은
[configuration_act.py:127-129](src/lerobot/policies/act/configuration_act.py#L127-L129):
`optimizer_lr=1e-5`, `optimizer_lr_backbone=1e-5`,
`optimizer_weight_decay=1e-4`. LeRobot 멀티 GPU 문서는 LR auto-scale
을 하지 않는다고 명시. AdamW 계열은 linear scaling 보다 sqrt
scaling 이 안정적인 경험칙이 있어 두 값을 모두 산출한다.

### Decisions (사용자 확정)
- Probe 동안 wandb / HF Hub push 비활성화 (실제
  `7__train_act_adv.sh` 는 변경하지 않고, `claude_test/` 의
  probe 래퍼에서만 override).
- Probe 1 회당 모델 로딩 + 첫 forward/backward + 수 step 안정화
  까지 ~120 s 정도 허용 후 kill, peak VRAM 만 채집.
- 후보 ladder: per-GPU `bs ∈ {8, 16, 24, 32}` 부터 시작해 ~80 %
  지점까지 binary refine. 글로벌 batch = per-GPU × 3.

### Work items
- [x] `claude_test/probe_vram_batch.sh` 작성: accelerate launch
      파라미터를 인자로 받고 wandb / push_to_hub 를 끄는 래퍼.
- [x] baseline `bs=8` peak VRAM 채집 (3 GPU 모두).
- [x] 후보 ladder 따라 probe 진행, ~19.6 GB peak 에 가장 가까운
      후보 선정 (OOM 회피 마진 포함).
- [x] 선정된 글로벌 배치에 대한 LR 권장값 계산
      (linear / sqrt 스케일링 모두 제시).
- [x] `claude_test/README.md` 업데이트 (probe 스크립트 행 추가).
- [x] gh issue create (#56).
- [x] commit + push (f27e9b95), gh issue 코멘트 등록 후 close.

### Results (2026-05-01)
하드웨어: 3 x RTX 3090 (24,576 MiB / GPU). 80 % 목표 = 19,660 MiB.
모든 측정은 fp16 mixed-precision, `num_processes=3`, `num_workers=10`
조건. peak 는 nvidia-smi 2 s sampling 의 maximum.

| per-GPU bs | global bs | GPU 0 peak | GPU 1 peak | GPU 2 peak | %/GPU |
|-----------:|----------:|-----------:|-----------:|-----------:|------:|
|         8  |       24  |  5,119 MiB |  5,109 MiB |  5,109 MiB | 20.8 %|
|        32  |       96  | 15,691 MiB | 15,733 MiB | 15,687 MiB | 64.0 %|
|        40  |      120  | 19,657 MiB | 19,375 MiB | 19,375 MiB | 80.0 %|

**선정**: `--batch_size=40` (per-GPU). global batch = 40 x 3 = 120
(기존 24 의 5 배). GPU 0 가 정확히 80.0 % 로 가장 빡빡한 rank;
GPU 1/2 는 78.8 %. fragmentation / cudnn workspace 변동에 약 ~1 GB
여유.

**LR 권장**: ACT 기본
([configuration_act.py:127-129](src/lerobot/policies/act/configuration_act.py#L127-L129))
는 `lr=1e-5`, `lr_backbone=1e-5`, `weight_decay=1e-4`. Global batch
가 5 배가 되었으므로,

- **sqrt scaling (보수적, AdamW + warmup 없음 환경의 기본 권장)**:
  `1e-5 × sqrt(5) ≈ 2.2e-5`
  → `--policy.optimizer_lr=2.2e-5
       --policy.optimizer_lr_backbone=2.2e-5`
- **linear scaling (더 공격적, warmup 함께 쓸 때 권장)**:
  `1e-5 × 5 = 5e-5`

LeRobot ACT preset 은 `get_scheduler_preset()` 이 `None` 이라
warmup 이 없으므로 **sqrt 스케일링 (2.2e-5) 을 우선 추천**.
weight_decay 는 1e-4 그대로 유지.

**참고 (스코프 밖)**: global batch 가 5 배가 되었으므로, 동일한 sample
exposure 를 유지하려면 `--steps` 도 1/5 로 (500K → 100K) 줄일 수
있다. 사용자가 epoch 수를 늘리고 싶다면 그대로 두어도 됨.

## 2026-05-01: 7__train_smovla_adv.sh 의 batch_size 를 VRAM 80% 기준으로 튜닝

### Background
ACT (#56) 와 동일한 절차를 SmolVLA 에 적용. 스크립트
[7__train_smovla_adv.sh](7__train_smovla_adv.sh) 는 현재
`--batch_size=32 --num_processes=3 --mixed_precision=fp16`,
`--policy.freeze_vision_encoder=true --policy.train_expert_only=true`,
`--policy.compile_model=true`. SmolVLA 기본 옵티마이저는
[configuration_smolvla.py:77-85](src/lerobot/policies/smolvla/configuration_smolvla.py#L77-L85):
`optimizer_lr=1e-4`, `betas=(0.9, 0.95)`, `weight_decay=1e-10`,
warmup-cosine scheduler (`warmup=1000`, `decay=30000`,
`decay_lr=2.5e-6`). ACT 와 달리 **warmup 이 있으므로** linear
LR scaling 이 안전.

### Decisions
- ACT probe 와 동일하게 wandb / push 비활성화 wrapper
  (`claude_test/probe_vram_smolvla.sh`).
- `compile_model=true` 의 warmup 시간이 ACT 보다 길 수 있으므로
  probe duration 을 처음부터 ~360 s 로 시작.
- 후보 ladder: `bs ∈ {32, 16, 8, 24, ...}`. SmolVLA 450M backbone +
  expert 라 ACT 보다 VRAM 사용량이 클 가능성 높음. 32 가 OOM 이면
  내려가고, 여유가 많으면 올린다.

### Work items
- [x] `claude_test/probe_vram_smolvla.sh` 작성.
- [x] baseline `bs=32` peak VRAM 채집.
- [x] 80 % 목표 (~19.6 GB) 에 가장 가까운 후보 선정.
- [x] LR 권장값 (warmup 있으므로 linear scaling 우선).
- [x] `claude_test/README.md` 업데이트.
- [x] gh issue create (#59).
- [x] commit + push (60f53131), gh issue close.

## 2026-05-01: 7__train_smovla_adv.sh 에 #59 결과 반영

### Background
#59 의 probe 결과를 [7__train_smovla_adv.sh](7__train_smovla_adv.sh)
에 직접 반영. `--mixed_precision=bf16` 은 사용자가 이미 적용한
상태로 확인됨 (line 71). 나머지 batch / LR / steps / save_freq /
scheduler 만 추가/수정.

### Decisions
- bs 32 → **72** (3 GPU global 216).
- LR 1e-4 default → **2.25e-4** (linear, warmup-cosine 있어 안전).
- steps 20000 → **9000** (1.92M samples exposure 보존).
- scheduler_decay_steps 30000 → **9000** (cosine decay 가 학습
  끝까지 완주하도록 steps 와 정합).
- save_freq 5000 → **3000** (9000 step 동안 3 개 ckpt 유지).
- compile_model / freeze_vision_encoder / train_expert_only /
  num_workers / seed / tolerance_s 는 그대로.

### Work items
- [x] `--batch_size=32` → `--batch_size=72`.
- [x] `--steps=20000` → `--steps=9000`.
- [x] `--save_freq=5000` → `--save_freq=3000`.
- [x] `--policy.optimizer_lr=2.25e-4` 추가.
- [x] `--policy.scheduler_decay_steps=9000` 추가.
- [x] `bash -n 7__train_smovla_adv.sh` 구문 검증.
- [x] gh issue create (#60).
- [x] commit + push (a261cab1), gh issue close.

### Out of scope
- `--mixed_precision` (이미 bf16 으로 사용자가 변경 완료).
- `7__train_smovla.sh` (single / base 변형) 변경.
- compile_model / freeze_vision_encoder / train_expert_only 변경.
- 2.2B SmolVLM2 backbone 옵션.

### Results (2026-05-01)
모든 측정은 `num_processes=3`, `compile_model=true`,
`freeze_vision_encoder=true`, `train_expert_only=true`,
`num_workers=10`. peak 는 nvidia-smi 2 s sampling 의 maximum.

**중요한 발견 (script bug)**: 현재 `7__train_smovla_adv.sh` 의
`--mixed_precision=fp16` 은 SmolVLA 백본에서 crash:
`NotImplementedError: "_amp_foreach_non_finite_check_and_unscale_cuda"
not implemented for 'BFloat16'`. 백본 일부가 bf16 으로 캐스팅되어
GradScaler 가 unscale 할 때 실패. 원본 `7__train_smovla.sh` 와
같이 **`bf16` 으로 되돌려야** 학습이 시작됨. 모든 probe 는 bf16 으로
진행.

| per-GPU bs | global bs | GPU 0 peak | GPU 1 peak | GPU 2 peak | %/GPU |
|-----------:|----------:|-----------:|-----------:|-----------:|------:|
| 32 (현재)  |       96  |  9,886 MiB |  9,331 MiB |  9,331 MiB | 40.2 %|
| **72**     |    **216**| **19,028 MiB** | 18,439 MiB | 18,439 MiB | **77.4 %** |
| 76         |      228  | 20,050 MiB | 19,397 MiB | 19,397 MiB | 81.6 % (cap 초과)|
| 96         |      288  | 24,050 MiB → OOM | – | – | 97.9 % (다음 step OOM)|

**선정**: `--batch_size=72` (per-GPU). global batch = 72 x 3 = 216.
GPU 0 = 77.4 % 로 80 % cap 만족. bs=76 부터 GPU 0 가 81.6 % 로
넘김. ~1.3 GB 안전 마진.

**LR 권장**: SmolVLA 기본은
[configuration_smolvla.py:77-86](src/lerobot/policies/smolvla/configuration_smolvla.py#L77-L86)
의 `lr=1e-4`, warmup-cosine (`warmup_steps=1000`,
`decay_steps=30000`, `decay_lr=2.5e-6`). global batch 가 96 → 216
(2.25 x) 로 늘었고, **warmup 이 있으므로 linear scaling 권장**:

- **Linear (권장)**: `1e-4 × 2.25 = 2.25e-4`
  → `--policy.optimizer_lr=2.25e-4`
- Sqrt (보수): `1e-4 × sqrt(2.25) = 1.5e-4`

`betas`, `weight_decay`, `grad_clip_norm` 그대로 유지.

**필수 수정 (script bug 포함)**:
1. `--mixed_precision=fp16` → `--mixed_precision=bf16` (crash 해결).
2. `--batch_size=32` → `--batch_size=72`.
3. `--policy.optimizer_lr=2.25e-4` 추가 (CLI 에서 override).

**참고 (스코프 밖)**:
- global batch 가 2.25 배 늘었으므로 동일 sample exposure 유지하려면
  `--steps` 를 20000 / 2.25 ≈ 9000 으로 줄여도 됨. 단,
  `scheduler_decay_steps=30000` 이 `steps` 보다 크므로 어느 쪽이든
  cosine decay 끝까지 가지는 않음.
- 2.2B SmolVLM2 backbone 으로 바꾸면 VRAM ~4-5 배가 되므로 별도
  probe 필요.

### Out of scope
- `7__train_smovla_adv.sh` 의 실제 값 자체 수정 (사용자가 권장값을
  받아 직접 반영).
- `7__train_smovla.sh` (single / base 변형) 변경.
- compile_model / freeze_vision_encoder / train_expert_only 변경.
- 2.2B SmolVLM2 backbone 옵션 평가.



### Out of scope
- `7__train_act_adv.sh` 의 실제 `--batch_size` 값 자체 수정
  (사용자가 권장값을 받아 직접 반영).
- LR scheduler / warmup 정책 변경.
- ACT 모델 아키텍처 / num_workers / mixed_precision 변경.
- pi0 / pi05 학습 스크립트 변경.

## 2026-05-01: 7__train_pi0_adv.sh 에 #55 probe 결과 반영

### Background
[#55](https://github.com/coport-uni/FR5ControllerVLA/issues/55)
probe 결과 per-GPU batch=160 (effective 320) 이 H200 NVL 80 %
임계 (≈ 115 GB) 에 위치
([claude_test/probe_logs/SUMMARY.md](claude_test/probe_logs/SUMMARY.md)).
원 스크립트는 per-GPU batch=16 (effective 32) 이므로 해당 값을
바꾸고, effective batch 가 10 배가 된 만큼 lr 도 SQRT 스케일링
(`2.5e-5 * sqrt(10) ≈ 7.9e-5`) 으로 갱신.

### Decisions (사용자 확정)
- per-GPU `--batch_size=16` → `--batch_size=160` (앞선 probe 결과
  2 × H200 NVL 에서 GPU0 peak 79.6 %, GPU1 75.8 %).
- `--policy.optimizer_lr=7.9e-5` 추가 (SQRT 스케일링).
- `scheduler_warmup_steps=1000` 기본값 유지.
- `--resume=true` / `--config_path=...015000...` 는 그대로 두되,
  본 스크립트는 새 `JOB_NAME=fr5_pi0_red_marker_adv` 로 별도
  output 디렉토리에 학습한다 (이미 그렇게 되어 있음).

### Work items
- [x] [claude_test/README.md](claude_test/README.md) 의 #55 ↔ #56
      merge 잔여 conflict 마커 정리 (양쪽 모두 보존).
- [x] [7__train_pi0_adv.sh](7__train_pi0_adv.sh) 에 batch=160 / lr
      반영 + 헤더 주석에 #55 reference 메모 추가.
- [x] `bash -n 7__train_pi0_adv.sh` 구문 검증.
- [x] `gh issue create` 로 follow-up 이슈 등록 (#57).
- [x] commit + push (2f86e112), `gh issue close` 로 클로즈.

### Out of scope
- 30k 본 학습 실행.
- `--steps` / `--save_freq` / scheduler / weight_decay 변경.
- `7__train_pi0.sh` (quickstart) / `7__train_pi05.sh` 변경.
- 새로운 probe 실행 (#55 결과 그대로 사용).

## 2026-05-01: 7__train_pi05_adv.sh per-GPU VRAM ~80% batch 탐색 + 적용

### Background
- 동일 박스 (2× H200 NVL, 각 143.77 GB / 143771 MiB).
  목표 80% ≈ 115 GB / GPU (peak 기준; 두 GPU 중 더 큰 peak ≤ 115 GB).
  최초 90% 목표였으나 pi05 batch=160 (pi0 80% fit 값) 이 OOM 으로
  실패하여, pi0 와 동일한 80% 임계로 변경.
- pi0 probe (#55) 결과: per-GPU batch=160 → GPU0 79.6 % / GPU1 75.8 %,
  batch=176 → OOM. pi05 는 모델 크기가 거의 동일하므로 유사 거동
  예상이지만 normalization_mapping (MEAN_STD) / weight_decay 차이로
  미세 차이 가능 → 실측 필요.
- 현재 [7__train_pi05_adv.sh](7__train_pi05_adv.sh): per-GPU
  batch=160, bf16 + `compile_model=true`
  + `gradient_checkpointing=true`, `freeze_vision_encoder=false`,
  `optimizer_weight_decay=1e-10`, MEAN_STD normalization.

### Decisions (사용자 확정)
- 임계 정의: 두 GPU 중 더 큰 peak ≤ 115 GB (80%).
  (최초 90% 였으나 pi05 batch=160 OOM 확인 후 변경.)
- lr 스케일링: SQRT (`2.5e-5 × sqrt(B'/32)`) — pi0 와 동일 정책.
- 1차 probe 후보 [160, 168, 172, 176, 184] → 모두 OOM 위험으로
  중단 (`pkill`); 2차 probe 후보 [96, 112, 128, 144, 152].

### Method
- [claude_test/probe_pi05_vram.sh](claude_test/probe_pi05_vram.sh)
  작성 — `probe_pi0_vram.sh` 골격 그대로, policy 옵션만 pi05 정합:
  `policy.type=pi05`, `pretrained_path=lerobot/pi05_base`,
  `normalization_mapping=MEAN_STD`,
  `optimizer_weight_decay=1e-10`.
- production 격리: `JOB_NAME=fr5_pi05_vram_probe_b<N>`,
  `--resume=false`, `push_to_hub=false`, `wandb.enable=false`,
  `save_freq` 매우 큼.
- 1 Hz `nvidia-smi` 샘플링, `--steps=50`.

### Hyperparameter recommendation
- 기준 (openpi pi05): effective_batch=32, lr=2.5e-5, warmup=1000,
  decay=30000, weight_decay=1e-10.
- 새 effective batch B' 에 대한 SQRT 스케일 lr 권고
  (`2.5e-5 × sqrt(B'/32)`).
- `--steps=3000` (~8 epoch, openpi sample budget 동일) 유지.
- `weight_decay=1e-10`, `normalization_mapping`, `save_freq`,
  `resume` 그대로 유지.

### Work items
- [x] LP §3/§4/§5 관련 항목 확인 (#55 와 동일 — 해당 없음).
- [x] [claude_test/probe_pi05_vram.sh](claude_test/probe_pi05_vram.sh)
      작성 + [claude_test/README.md](claude_test/README.md) 행 추가.
- [x] `gh issue create` 로 이슈 등록 (#58).
- [x] 1차 probe sweep [160, 168, 172, 176, 184] → batch=160 부터
      OOM 으로 중단 (`pkill`).
- [x] 2차 probe sweep [96, 112, 128, 144, 152] 실행.
- [x] bisect [132, 136, 140] — boundary 확정 (batch=136 fit,
      batch=140 OOM).
- [x] [claude_test/probe_logs/SUMMARY_pi05.md](claude_test/probe_logs/SUMMARY_pi05.md)
      결과 표 정리.
- [x] [7__train_pi05_adv.sh](7__train_pi05_adv.sh) 에 batch=136 +
      lr=7.3e-5 반영, 헤더 주석에 probe 결과 메모.
- [x] `bash -n 7__train_pi05_adv.sh` 구문 검증.
- [x] 사용자 follow-up: 새 batch 에 맞춘 `--steps=3000 → 3500`
      반영 (3500 × 272 ≈ 952 k samples ≈ 7.97 epoch, openpi 960 k
      ≈ 8.04 epoch 정합).
      [claude_test/probe_logs/SUMMARY_pi05.md](claude_test/probe_logs/SUMMARY_pi05.md)
      / 헤더 주석 동기화.
- [x] commit + push (ac69f40b probe + apply, b18635e5 step 조정),
      `gh issue close` (commit message `Closes #58`) 로 클로즈.

### Out of scope
- 30k / 80k 본 학습 실행.
- pi0 추가 probe (이미 #55).
- 정책 코드 / processor / dataset / normalization / weight_decay
  변경.
- `7__train_pi0.sh` / `7__train_pi05.sh` (quickstart) 변경.
- EMA / quantile 증강 구현.

## 2026-05-03: pi0 step/resume 정합 + 8__run_server 충돌 마커 정리

### Background
사용자가 working tree 에서 직접 두 파일을 수정한 상태:
- [7__train_pi0_adv.sh](7__train_pi0_adv.sh): `--steps 30000→3000`,
  `--save_freq 5000→500`, `--resume=true` + `--config_path=...015000`
  제거 → `--resume=false`. 이는 #57 에서 적용한 batch=160 / lr=7.9e-5
  (effective 320, 10× pi0 baseline) 에 맞춰 sample budget 을 openpi
  reference (960 k samples ≈ 8 epoch) 로 맞추는 후속 정렬이며,
  pi05 의 #58 follow-up 과 동일한 패턴.
- [8__run_server.sh](8__run_server.sh): merge commit `eef60bbc` 에
  남아 있던 conflict 마커 (port 17058 vs 17044) 정리, port=17044
  채택.

사용자 명시 요청 ("현재 코드 모두 커밋해줘") 으로 두 변경을
하나의 commit 으로 묶음.

### Decisions (사용자 확정)
- 두 파일 모두 그대로 commit (사용자 의도된 변경, 이미 working
  tree 에 적용됨).
- 단일 commit + 단일 gh issue 로 처리.

### Work items
- [x] `gh issue create` (#61).
- [x] 두 파일 add + commit (248d3140, `Closes #61`).
- [x] push.

### Out of scope
- 추가 학습 파라미터 변경 (lr / batch / weight_decay 등).
- 본 학습 실행.
- 다른 셸 스크립트 / 정책 코드 변경.

## 2026-05-03: outputs/train 학습 로그 GitHub 업로드 + HF 액세스 토큰 마스킹

### Background
[.gitignore](.gitignore) 의 `outputs/train/` 규칙 때문에 학습 로그
(`train_*.log`) 가 GitHub 에 올라가지 않고 있다. 사용자는 체크포인트
(`*/checkpoints/*`, run 당 3.7 GB ~ 5.8 GB, GitHub 100 MB 한도 초과)
는 계속 제외하되, 학습 로그(.log) 는 올리고 싶다.

선행 보안 점검에서 3 개 학습 로그 파일 모두 HuggingFace 의
`X-Xet-Access-Token` (JWT) 이 HTTP DEBUG 트레이스에 평문 기록되어
있는 것이 확인됐다. JWT 안에는 `userId`, `repoId`, `access: WRITE`
가 포함되며 토큰 자체는 만료되었으나 GitHub 공개 업로드 전에
마스킹이 필요하다.

### Decisions (사용자 확정)
- 옵션 1: 토큰 라인의 JWT 값만 `<REDACTED>` 로 마스킹 후 업로드.
  나머지 디버그 라인 / 응답 헤더는 유지.
- `.gitignore` 의 `outputs/train/` 라인을 `outputs/train/*/checkpoints/`
  로 교체. wandb 디렉토리는 전역 `wandb/` 규칙으로 계속 제외됨.

### Work items
- [x] [outputs/train/fr5_act_red_marker_adv_3090/train_20260501-081054.log](outputs/train/fr5_act_red_marker_adv_3090/train_20260501-081054.log) 의 JWT 마스킹.
- [x] [outputs/train/fr5_smolvla_red_marker_adv_3090/train_20260503-123225.log](outputs/train/fr5_smolvla_red_marker_adv_3090/train_20260503-123225.log) 의 JWT 마스킹.
- [x] [outputs/train/fr5_smolvla_red_marker_base_3090/train_20260428-051359.log](outputs/train/fr5_smolvla_red_marker_base_3090/train_20260428-051359.log) 의 JWT 마스킹.
- [x] 마스킹 후 `grep -c X-Xet-Access-Token.*eyJ` 로 잔존 토큰 0 확인.
- [x] [.gitignore](.gitignore) `outputs/train/` → `outputs/train/*/checkpoints/`
      (`!outputs/train/*/train_*.log` re-include 도 추가 — `*.log`
      전역 ignore 가 train 로그를 잡지 않도록).
- [x] 추가 마스킹 (pre-signed URL `X-Amz-Signature` / `X-Amz-Credential`
      / `Policy=eyJ...` / `Signature=` / `Key-Pair-Id=` 및
      `X-Xet-Cas-Uid`). 사용자가 발견 후 옵션 1 추가 적용 확정.
- [x] `gh issue create` 로 follow-up 이슈 등록 (#62).
- [x] commit + push (2b752f2b, rebase onto origin/main).
- [x] `gh issue close`.

### Out of scope
- wandb 디렉토리 업로드 (전역 `wandb/` 규칙 그대로 유지).
- 체크포인트 / `outputs/datasets/` / `outputs/smoke_logs/` 업로드.
- DEBUG 트레이스 라인 전체 제거.
- 향후 학습 스크립트의 로그 레벨 조정 (DEBUG → INFO) — 별도 작업.

## 2026-05-04: h200 학습 로그 GitHub 업로드 + HF 액세스 토큰 마스킹

### Background
2026-05-03 작업 (#62, commit 2b752f2b) 으로 3090 학습 로그 3 건은
마스킹 후 업로드되었으나, 같은 시점에 H200 박스에서 생성된
`outputs/train/fr5_*_h200/` 6 개 학습 로그는 워킹 트리에 untracked
상태로 남아 있었다. `.gitignore` 는 이미 `outputs/train/*/checkpoints/`
+ `!outputs/train/*/train_*.log` 로 갱신돼 있어 추가 수정 불필요.

대상 파일과 사전 스캔 결과 (X-Xet-Access-Token JWT / pre-signed URL):
- [outputs/train/fr5_act_red_marker_base_h200/train_20260423-132434.log](outputs/train/fr5_act_red_marker_base_h200/train_20260423-132434.log) — JWT 1, URL 0.
- [outputs/train/fr5_pi05_red_marker_adv_h200/train_20260503-125626.log](outputs/train/fr5_pi05_red_marker_adv_h200/train_20260503-125626.log) — JWT 1, URL 1.
- [outputs/train/fr5_pi05_red_marker_base_h200/train_20260428-135937.log](outputs/train/fr5_pi05_red_marker_base_h200/train_20260428-135937.log) — JWT 1, URL 1.
- [outputs/train/fr5_pi0_red_marker_adv_h200/train_20260501-083601.log](outputs/train/fr5_pi0_red_marker_adv_h200/train_20260501-083601.log) — JWT 1, URL 1.
- [outputs/train/fr5_pi0_red_marker_base_h200/train_20260427-133929.log](outputs/train/fr5_pi0_red_marker_base_h200/train_20260427-133929.log) — JWT 0, URL 1.
- [outputs/train/fr5_pi0_red_marker_base_h200/train_20260428-082852.log](outputs/train/fr5_pi0_red_marker_base_h200/train_20260428-082852.log) — JWT 1, URL 0.

### Decisions (사용자 확정)
- 마스킹 정책: #62 의 옵션 1 과 동일 (자격증명 값만 `<REDACTED>`,
  DEBUG 라인은 유지).
- `Xserver.sh` 삭제 (워킹 트리에 `D` 로 보이는 항목) 도 이번 커밋에
  함께 포함.

### Work items
- [x] [outputs/train/fr5_act_red_marker_base_h200/train_20260423-132434.log](outputs/train/fr5_act_red_marker_base_h200/train_20260423-132434.log) JWT 마스킹.
- [x] [outputs/train/fr5_pi05_red_marker_adv_h200/train_20260503-125626.log](outputs/train/fr5_pi05_red_marker_adv_h200/train_20260503-125626.log) JWT + pre-signed URL 마스킹.
- [x] [outputs/train/fr5_pi05_red_marker_base_h200/train_20260428-135937.log](outputs/train/fr5_pi05_red_marker_base_h200/train_20260428-135937.log) JWT + pre-signed URL 마스킹.
- [x] [outputs/train/fr5_pi0_red_marker_adv_h200/train_20260501-083601.log](outputs/train/fr5_pi0_red_marker_adv_h200/train_20260501-083601.log) JWT + pre-signed URL 마스킹.
- [x] [outputs/train/fr5_pi0_red_marker_base_h200/train_20260427-133929.log](outputs/train/fr5_pi0_red_marker_base_h200/train_20260427-133929.log) pre-signed URL 마스킹.
- [x] [outputs/train/fr5_pi0_red_marker_base_h200/train_20260428-082852.log](outputs/train/fr5_pi0_red_marker_base_h200/train_20260428-082852.log) JWT 마스킹.
- [x] 마스킹 후 `grep -c 'X-Xet-Access-Token.*eyJ\|X-Amz-Signature=[^<]\|Policy=eyJ\|Key-Pair-Id=[^<]'` 등으로 잔존 토큰 0 확인.
- [x] `gh issue create` 로 follow-up 이슈 등록 (#63).
- [x] commit + push (09fb591e — h200 로그 6 개 + `Xserver.sh` 삭제).
- [x] `gh issue close`.

### Out of scope
- wandb 디렉토리 업로드 (전역 `wandb/` 규칙 그대로 유지).
- 체크포인트 / `outputs/datasets/` / `outputs/smoke_logs/` 업로드.
- DEBUG 트레이스 라인 전체 제거.
- 학습 스크립트의 로그 레벨 조정 (DEBUG → INFO) — 별도 작업.

## 2026-05-04: Hard-reset main to 2542f804

사용자 요청으로 로컬 `main` 을 `2542f804` (merge2) 로 `git reset --hard`.
폐기된 commit 3 개 (origin/main 보다 1 앞서 있던 상태):
- `fa2e8e9e` Align ServoJ cmdT with actual send_action call rate (#65)
- `35a1b8d2` Mark commit checkbox closed for #64 diagnosis entry in ToDo.md
- `ec18ec0f` Diagnose ServoJ mid-motion stop on FR5 ACT async inference (#64)

미커밋 변경사항 5 파일 (`.claude/settings.json`, `9__run_client_act.sh`,
`ToDo.md`, `config_fairino_follower.py`, `fairino_follower.py`) 도 함께
폐기됨.

### Work items
- [x] `git reset --hard 2542f804` 실행, working tree clean 확인.
- [x] `gh issue create` 로 기록 등록 (#69).
- [ ] commit + push (이 ToDo entry).

### Out of scope
- 원격 force-push (사용자가 별도 요청 시에만 진행).
  현재 `origin/main` 은 여전히 `35a1b8d2` 에 있고 로컬은 2 commit 뒤짐.
- 폐기된 변경사항 복구 (`git reflog` 로 단기 복구는 가능).

## 2026-05-07: 2-week rollback to e76ecdf137 with backup branch + tag

사용자 요청: 현재 코드를 백업하고 약 2 주 전 상태(`e76ecdf1`
"merge_fix" 2026-04-29)로 롤백. 단, `main` 은 비파괴적으로 보존하고
새 작업 브랜치에서 롤백 상태를 갖도록 한다.

### Decisions (사용자 확정)
- **롤백 대상**: `e76ecdf137caf9368e52b7da76946fbe49e2f6e3`.
- **백업**: 브랜치 `backup/main-pre-rollback-2026-05-07` + 태그
  `backup-2026-05-07` 둘 다 생성 (옵션 2-C).
- **롤백 방식**: 새 브랜치 `rollback/2-weeks-ago` 를 `e76ecdf1` 에서
  분기. `main` 은 `3b28332a` 그대로 유지 (옵션 3-C, 비파괴).
- **dirty 파일 처리**: 현재 working tree 의 8 개 modified 파일
  (`9__run_client_act.sh`, `9__run_client_smovla.sh`,
  `outputs/captured_images/opencv__dev_video{2,4,18,19,20}.png`,
  `outputs/captured_images/realsense_333422300435.png`) 은 백업
  브랜치에 snapshot 커밋으로 보존 (옵션 4-A).
- **원격**: push 안 함. 로컬 작업으로만 진행 (옵션 5-A).

### Work items
- [x] 현재 HEAD (`3b28332a` → ToDo entry commit `84a4a168`) 에서
  `backup/main-pre-rollback-2026-05-07` 브랜치 생성, 그 위에 dirty
  8 파일 snapshot 커밋 (`b1931a12`).
- [x] 같은 커밋에 `backup-2026-05-07` 태그 부여.
- [x] `main` 으로 복귀 후 working tree clean 확인.
- [x] `rollback/2-weeks-ago` 브랜치를 `e76ecdf1` 에서 생성하고 checkout.
- [x] 최종 상태 검증: `git branch`, `git log --oneline -3`,
  `git status`, `git tag -l 'backup-*'`.
- [x] `gh issue create` 로 본 항목 등록 (#70).
- [ ] commit (이 ToDo entry, push 안 함 — 옵션 5-A).

### Out of scope
- 원격 force-push 및 `origin/main` 변경 (옵션 5-A 명시).
- `main` 자체 hard-reset (옵션 3-C 로 명시적으로 회피).
- 백업 브랜치/태그의 원격 push (사용자 별도 지시 시에만).
- `e76ecdf1` 이전 / 이후 커밋의 cherry-pick 또는 merge.
## 2026-05-04: ServoJ 동작 중 로봇 정지 원인 진단 (diagnosis only)

### Background
사용자가 [9__run_client_act.sh](9__run_client_act.sh) 로 ACT async inference
실행 중 FR5 가 ServoJ 동작 도중 갑자기 멈추는 현상을 보고. 사용자
직감: "컨트롤러 단의 ServoJ 관련 안전장치가 막는 것 같다".
이 entry 는 **진단 단계만** 다루며, 실제 수정(servo_hz/cmdT 조정,
로깅 강화 등)은 사용자 확정 후 별도 ToDo entry 로 진행한다.
관련 LP: §G2 (ServoJ 101 / settle), §Q1 (7-param signature).

### Findings (가장 유력 → 보조)
1. **cmdT 권장범위 위반** (가장 유력):
   - [fairino_follower.py:336](src/lerobot/robots/fairino_follower/fairino_follower.py#L336)
     `period = 1.0 / servo_hz` → `servo_hz=100` 이라 cmdT = **10 ms**.
   - [fairino_follower.py:350-358](src/lerobot/robots/fairino_follower/fairino_follower.py#L350-L358)
     해당 `period` 를 그대로 ServoJ cmdT 인자로 전달.
   - SDK 명시 권장값
     [fairino/Robot.py:1572](src/lerobot/robots/fairino_follower/fairino/Robot.py#L1572):
     `cmdT 建议范围[0.001~0.0016], 默认为0.008` (권장 1~1.6 ms,
     기본 8 ms). 10 ms 는 권장 상한의 6.25 배 + 기본값 초과 →
     컨트롤러 buffer underrun / 안전장치 트리거 가능.
2. **GetSafetyCode() silent 거부**:
   - [fairino/Robot.py:1584-1585](src/lerobot/robots/fairino_follower/fairino/Robot.py#L1584-L1585)
     ServoJ 진입 전 `GetSafetyCode() != 0` 시 즉시 return.
   - [fairino_follower.py:362-365](src/lerobot/robots/fairino_follower/fairino_follower.py#L362-L365)
     현재 `ret != 0` 을 `logger.debug` 로만 기록 → 어떤 안전장치
     코드가 작동했는지 운영 중 확인 어려움.
3. **에러 14 복구 후 ramp 위치 점프**:
   - [fairino_follower.py:418-449](src/lerobot/robots/fairino_follower/fairino_follower.py#L418-L449)
     `_recover_servo_session()` 후 `_commanded` 가 실제 위치와
     동기화되지만, 직후 ramp 재시작 시 추가 settle 없이
     이어지면 컨트롤러 안전장치가 트리거될 수 있음.

### Work items
- [x] `fairino_follower.py` 의 `send_action` / `_recover_servo_session`
  / `_initialise_servo_mode` 코드 인용·라인 확인.
- [x] Fairino SDK `Robot.py` 의 ServoJ 권장 cmdT 범위 / GetSafetyCode
  사전 검사 라인 확인.
- [x] `LearnedPatterns.md` §G2 / §Q1 ServoJ 관련 항목 교차 확인.
- [x] [9__run_client_act.sh](9__run_client_act.sh) async inference
  실행 컨텍스트 확인 (fps=20, chunk_size=100).
- [x] `gh issue create` 로 진단 결과 등록 (#64).
- [x] commit + push (이 entry + 진단 결과 issue link) — ec18ec0f.
- [ ] (사용자 확정 대기) 후속 fix entry 작성:
  - cmdT/servo_hz 보수 조정 (예: cmdT=0.008 또는 0.004).
  - ServoJ 에러 로깅 `debug` → `warning` 승격 + GetSafetyCode 동시 기록.
  - 에러 14 복구 후 짧은 settle (≈100 ms) 추가.

### Out of scope (이번 진단 entry)
- 실제 코드 수정 (별도 ToDo entry + 별도 commit).
- 새 테스트 추가.
- LearnedPatterns.md 업데이트 (fix 후 재현 결과 확인 후).

## 2026-05-07: Add 9__run_client_pi0.sh inference client

근거: `7__train_pi0.sh` 가 `coport-uni/FR5_pick_red_colored_marker_to_box_pi0_model`
로 push 하는 pi0 체크포인트를 PolicyServer (`8__run_server.sh`) 에 띄우고
async-inference 로 FR5 follower 에서 실행하기 위한 client 가 누락되어
`9__run_client_act.sh` / `9__run_client_smovla.sh` 와 동일한 패턴으로 추가.

설계 결정 (사용자 확정):
- 카메라 키: act 스타일 (`top_left` / `top_right` / `hand`).
- server_address: `10.0.12.139:17044` (act/smolvla 동일).
- fps=20, chunk_size_threshold=0.6 (smolvla 동일).
- `actions_per_chunk=50` — pi0 기본 `chunk_size = n_action_steps = 50`
  (`src/lerobot/policies/pi0/configuration_pi0.py`).
- `task="pick red colored marker to box"` — pi0 는 language-conditioned.
- `policy_type=pi0`, `pretrained_name_or_path=
  coport-uni/FR5_pick_red_colored_marker_to_box_pi0_model`.

- [x] `9__run_client_pi0.sh` 작성 (act/smolvla client 패턴 + pi0 파라미터)
- [x] `bash -n` 으로 shell 문법 검증
- [x] `gh issue create` 로 등록 (#71)
- [x] commit + push (issue link 포함) — 033bdb9f
- [x] `gh issue close` 로 종료

## 2026-05-07: Switch 7__train_smovla22_adv.sh to SmolVLM2-2.2B backbone

근거: 현재 스크립트는 `lerobot/smolvla_base` (SmolVLM2-500M + 사전학습된
action expert) 를 로드함. 파일명 `_smovla22_` 와 헤더 주석의 의도(2.2B
백본 사용) 가 실제 동작과 불일치. 2.2B 백본으로 정식 전환하면서
SmolVLA 논문 휴리스틱(VLM 절반 층) 을 24층 백본에 외삽 적용.

설계 결정 (approach (a) — 사용자 확정):
- `--policy.path=lerobot/smolvla_base` 제거 (450M 사전학습 가중치 폐기).
- `--policy.type=smolvla` 추가 (path 없이 정책 지정).
- `--policy.vlm_model_name=HuggingFaceTB/SmolVLM2-2.2B-Instruct` 추가.
- `--policy.load_vlm_weights=true` 추가 (VLM 가중치만 사전학습 사용,
  expert 는 from-scratch).
- `--policy.num_vlm_layers=12` 추가 (논문 휴리스틱 50% × 24층).
- `--batch_size=32` 그대로 유지 (사용자 지시 — VRAM 튜닝 미실시).
- `expert_width_multiplier` default 0.75 유지.
- 공식 2.2B 권장 하이퍼파라미터 없음 (논문/LeRobot docs/커뮤니티 모두).
  → 현 설정은 **외삽** 임을 헤더 주석에 명시.

작업 항목:
- [x] 7__train_smovla22_adv.sh 수정 (위 6개 플래그 변경 + 헤더/하단 주석 업데이트)
- [x] `bash -n` 으로 shell 문법 검증
- [x] `gh issue create` 로 등록 (#73)
- [x] commit + push (issue link 포함) — a81bb703

Out of scope:
- VRAM/batch_size 튜닝 (사용자 지시로 유보).
- 실제 학습 실행 및 loss 검증.
- LearnedPatterns.md 업데이트 (학습 결과 확인 전).
## 2026-05-07: Edge-triggered ServoJ error logging in fairino_follower

근거: 운영 중 ServoJ 가 실패하더라도 현재 코드는
[fairino_follower.py:362-370](src/lerobot/robots/fairino_follower/fairino_follower.py#L362-L370)
에서 `logger.debug` 로만 기록하므로 기본 로그 레벨에서 보이지 않음.
계속 같은 에러를 매 control tick (≈20Hz) 마다 출력하면 로그 스팸이
되므로, **이전 tick 의 에러 코드와 다를 때만** 출력하는
edge-triggered 방식으로 변경 (사용자 요청, see LP §G2 / §Q1).

설계 결정 (사용자 확정 대기):
- 직전 상태를 `self._last_servoj_state` 에 저장 (`"ret:0"` /
  `"ret:14"` / `"exc:ConnectionResetError"` 같은 문자열 키).
- `ret == 0` 으로 복귀: `logger.info("[Fairino] ServoJ recovered (was %s)")`.
- 새 에러 (`ret != 0` 이면서 이전과 다름): `logger.warning("[Fairino] ServoJ err: %d")`.
- exception 도 같은 규칙 (타입 이름이 바뀌면 한 번 출력).
- 에러 14 의 경우 `_recover_servo_session()` 이 이미 INFO 로그를
  남기지만, edge-triggered 라 매 tick 중복되지 않으므로 그대로 둠
  (한 번씩만 짝으로 출력됨 → 의도된 동작).

### Work items
- [x] `__init__` 에 `self._last_servoj_state: str = "ret:0"` 추가.
- [x] `send_action` 의 ServoJ try/except 분기에서 edge-triggered
  로깅 헬퍼 호출로 교체.
- [x] `_log_servoj_state_change(new_state: str, ret: int, exc: Exception | None)`
  헬퍼 메서드 추가 (전이 분기 한 곳에 모음).
- [x] `_recover_servo_session` 호출 후에도 상태 변수가 정확히
  반영되도록 흐름 검토 (recovery 후 다음 tick 이 `ret:0` 이면
  자연스럽게 "recovered" 로그가 한 번 출력됨).
- [x] `ruff check` / `ruff format --check` 통과 확인.
- [x] `gh issue create` 로 등록 (#72).
- [ ] commit + push (issue link 포함).
- [ ] `gh issue close` 로 종료.

## 2026-05-07: Isolate gripper worker from main ServoJ loop

근거: `9__run_client_pi0.sh` 로 pi0 async-inference 를 돌리는 동안
그리퍼 동작 중 ServoJ 가 매 틱 error 14 → `_recover_servo_session()`
무한 루프에 빠져 모션이 멈추는 증상 보고됨. 원인은 그리퍼 워커가
`_run_gripper_cmd` 에서 `ServoMoveEnd → MoveGripper(block=1, maxtime=30000ms)
→ ServoMoveStart` 를 도는 동안 메인 스레드의 ServoJ 가 죽은 서보 세션을
보고 [fairino_follower.py:366-367](src/lerobot/robots/fairino_follower/fairino_follower.py#L366-L367)
의 `_recover_servo_session()` 을 호출하여 워커의 시퀀스와 경쟁하기 때문.
(see LP §G8 / §R2)

설계 결정 (사용자 확정):
- 새 `threading.Event` `_servo_paused`: 워커가 ServoMoveEnd 직전 set,
  ServoMoveStart 완료 후 clear. 메인 스레드는 `is_set()` 동안 ServoJ
  자체와 `_recover_servo_session()` 을 모두 스킵.
- 워커가 unpause 직전 `_resync_needed = True` 로 표시. 메인 루프는
  다음 틱에 실 조인트 값으로 `_commanded` 를 동기화 후 ServoJ 재개
  (큰 ramp 점프 방지).
- 기존 `ramp_in_progress` 기반 그리퍼 드롭 게이트는 제거 (이제 `_servo_paused`
  가 충돌을 막아주므로 불필요).

### Work items
- [x] `__init__` 에 `_servo_paused: threading.Event`, `_resync_needed: bool` 추가.
- [x] `_run_gripper_cmd` 를 try/finally 로 감싸 set → 시퀀스 → resync flag → clear.
- [x] `send_action` 에 paused 분기, `_resync_needed` 처리, recovery 가드 추가.
- [x] `ramp_in_progress` 기반 그리퍼 드롭 제거.
- [x] `ruff check src/lerobot/robots/fairino_follower/fairino_follower.py` / `ruff format --check` 통과.
- [x] `gh issue create` 로 등록 (#74).
- [x] commit + push (issue link 포함) — 94d4bdfe.
- [x] `gh issue close` 로 종료.
- [x] `LearnedPatterns.md` 에 §G9 "MoveGripper(block=1) 중 ServoJ recovery race" 항목 추가.

## 2026-05-08: Disable torch.compile at async-inference policy load

근거: `PolicyServer` 에서 SmolVLA 2.2B 체크포인트
(`coport-uni/...smolvla_model22`) 를 서빙하면
`Error in StreamActions: Offset increment outside graph capture
encountered unexpectedly` 가 발생. HF `config.json` 비교 결과
2.2B 만 `pad_language_to=longest`, `prefix_length=-1` 로 입력
shape 가 가변이라 `torch.compile(mode=max-autotune)` 의 CUDA
Graphs 리플레이가 caching allocator offset 과 어긋나는 것이 원인
([processor_smolvla.py:75](src/lerobot/policies/smolvla/processor_smolvla.py#L75),
[modeling_smolvla.py:711](src/lerobot/policies/smolvla/modeling_smolvla.py#L711)).
450M 은 `pad_language_to=max_length` 라 정적 shape 라 우연히 비껴감.
Pi0 / Pi0.5 체크포인트도 모두 `compile_model=True,
compile_mode=max-autotune` 로 저장돼 있어 동일 위험 존재.
20Hz 원격 제어에서 `torch.compile` 의 추가 가속은 의미가 없으므로
**추론 시점에 한해** 컴파일을 끄는 것이 가장 안전한 최소 변경.
plan: `/home/inno-controller/.claude/plans/synchronous-enchanting-seal.md`.

설계 결정 (사용자 확정):
- 위치는 [policy_server.py:152](src/lerobot/async_inference/policy_server.py#L152)
  의 `policy_class.from_pretrained(...)` 직전.
- `PreTrainedConfig.from_pretrained` 로 config 를 먼저 로드해
  `compile_model=False` 로 덮어쓰고, 그 config 를
  `policy_class.from_pretrained(..., config=policy_cfg)` 로 다시
  넘김. `torch.compile` 은 정책 ctor 안에서 적용되므로 ctor 호출
  전에만 끄면 됨 ([pretrained.py:107](src/lerobot/policies/pretrained.py#L107),
  [modeling_smolvla.py:598-601](src/lerobot/policies/smolvla/modeling_smolvla.py#L598-L601),
  [modeling_pi0.py:590-594](src/lerobot/policies/pi0/modeling_pi0.py#L590-L594),
  [modeling_pi05.py:586-590](src/lerobot/policies/pi05/modeling_pi05.py#L586-L590)).
- 정책 종류 (`smolvla` / `pi0` / `pi05`) 와 무관하게 동일 진입점에서
  처리되므로 분기 없음.
- 학습 파이프라인, 체크포인트, 클라이언트는 손대지 않음.
- 무관 이슈인 카메라 키 mismatch (`'observation.images.camera2'`,
  state shape `[6]` vs `[7]`) 는 별도 ToDo 로 분리.

### Work items
- [x] `src/lerobot/async_inference/policy_server.py` 상단에
  `from lerobot.configs.policies import PreTrainedConfig` 추가.
- [x] [policy_server.py:152](src/lerobot/async_inference/policy_server.py#L152)
  한 줄을 config 사전 로드 + `compile_model=False` 덮어쓰기 +
  `policy_class.from_pretrained(..., config=policy_cfg)` 3 줄
  블록으로 교체. 이유 주석 1 줄 포함.
- [x] `ruff check src/lerobot/async_inference/policy_server.py`
  와 `ruff format --check` 통과 확인.
- [x] `gh issue create` 로 등록하고 issue 번호를 본 항목에 기록 (#75).
- [x] commit + push (issue link 포함, `Closes #N`) — cfe43371.
- [x] `gh issue close` 로 종료 (commit 자동 close 가 안 걸렸을 때만)
  — `Closes #75` 로 자동 종료됨, 별도 호출 불필요.
- [ ] 검증 후 `LearnedPatterns.md` §3 Library Quirks 에 신규 항목
  추가 — "max-autotune CUDA Graphs vs variable-shape VLA inputs".

Out of scope:
- SmolVLA 2.2B 의 카메라 키 (`top_left/top_right/hand`) 와
  클라이언트 alias (`camera1/2/3`) mismatch — 별도 ToDo.
- SmolVLA 450M 이 `state=[6]` 으로 그리퍼 위치를 학습 입력에서
  제외한 점 — 별도 검토.
- 학습 시점에 `compile_model=False` 로 저장하도록 학습 스크립트
  수정 — 이번 변경은 추론 측 한 줄 패치만 다룸.

## CommonClaude/CLAUDE.md 를 Python convention 기반으로 전환 (2026-05-28)

### 배경
upstream `coport-uni/CommonClaude` 의 `CLAUDE.md` 를 현재 C 언어
기준에서 Python 전용으로 전면 교체. 근거: (1) MIT CommLab Coding
and Comment Style 의 Python convention, (2) 본 프로젝트
FR5ControllerVLA `CLAUDE.md` §"Code Style: MIT Code Convention"
의 Python 관련 내용을 reference 로 사용. 이전 시도 (로컬 머지,
우선순위 역전, hook 글로벌화) 는 모두 폐기.

### 결정 사항
- Target: `coport-uni/CommonClaude:main` 에 PR 없이 직접 commit.
- Issue: `coport-uni/CommonClaude` 에 등록, commit message 의
  `Closes #N` 으로 자동 종료.
- Hook 은 건드리지 않음. CommonClaude 의 5 개 hook (그 중 2 개는
  secret scan / env guard) 와 그에 대응하는 settings.json 의
  Bash/Read matcher 는 넓은 보안 footprint 로 그대로 유지.
  프로젝트 hook 3 개와 CommonClaude 공유 hook 3 개는 이미
  byte-identical 임을 diff 로 확인 완료.

### 수정 대상 (CommonClaude/CLAUDE.md 만)
1. `Overview`: 대상 언어를 Python 으로 명시.
2. `§2 Naming` 표: C 기준 → Python 기준
   (Variable/Function/Class/Constant/Module). 본 프로젝트
   §"Code Style: MIT Code Convention > Naming" 표와 동일 포맷.
3. `§2 Comments TODO` 포맷: `/* TODO: ... */`
   → `# TODO: (@owner) action -- reason.`
4. `§2 Documentation`: Doxygen `/** @brief @param @return */`
   → PEP 257 / Google style docstring (`Args:`, `Returns:`,
   `Raises:`).
5. `§13 .gitignore`: C 빌드 산출물 → Python
   (`*.pyc, __pycache__/, *.egg-info/, .venv/, venv/,
   .pytest_cache/, .ruff_cache/, .mypy_cache/, dist/, build/`).
   secrets / editor / OS 블록은 유지.
6. `§16 Git Automation`: clang-format 예시 → ruff + ruff-format
   예시.
7. 잔여 "C" 단어 (예: §13 인트로) → "Python" 교체.

### Work items
- [x] MIT CommLab Python convention 페이지 WebFetch 로 확인,
  본 프로젝트 표와 일치 검증 완료
  (lower_case for vars/funcs/constants/modules, CamelCase for
  classes, 80-col, 4-space indent, PEP 257 docstrings).
- [x] `gh issue create -R coport-uni/CommonClaude ...` 로 등록 —
  https://github.com/coport-uni/CommonClaude/issues/27.
- [x] CommonClaude 를 `/tmp/CommonClaude` 에 HTTPS clone.
- [x] CLAUDE.md 의 §Overview / §2 / §6 / §8 / §11.4 / §13 / §16
  잔여 C 부분을 Python 전용으로 교체 (9 군데).
- [x] `git diff` 검토 후 main 에 commit
  (`refactor(claude-md): replace C-language style with Python
  convention`, `Closes #27` 포함) —
  https://github.com/coport-uni/CommonClaude/commit/e4f6e8d.
- [x] `git push origin main` 직접 push 완료
  (46fc4b8..e4f6e8d).
- [x] commit footer 로 issue #27 자동 close 확인.
- [x] FR5ControllerVLA/ToDo.md 의 본 항목 체크박스 갱신,
  commit + push (CommonClaude commit `e4f6e8d` 링크 포함).

## 2026-06-10: Re-diagnose inactive arrow-key controls in 5__fr5_record.sh

- User re-reported that Right/Left/Esc shortcuts do not work while
  running `5__fr5_record.sh`. Diagnostic only — no code change
  (see LP §G5, prior diagnosis ToDo 2026-04-22, gh issue #36).
- Finding (unchanged plus one new failure path):
  1. GUI terminal path: pynput uses the X11 (`_xorg`) backend; the
     login session is still Wayland (`loginctl` session 832
     Type=wayland), which blocks X11 global key capture. The
     listener starts cleanly but never receives events.
  2. SSH/remote terminal path (new): with no `DISPLAY`, `import
     pynput` raises `DisplayNameError`, so `is_headless()` returns
     True and `init_keyboard_listener()` returns `listener=None` —
     keyboard control is disabled entirely with only a log warning.
- [x] Verify pynput import failure in conda `lerobot` env
      (`failed to acquire X connection: Bad display name ""`).
- [x] Comment findings on existing gh issue #36 (no duplicate issue).
- [ ] User decision: pick remediation (Xorg session / evdev reader /
      termios stdin fallback) before any code change.

## 2026-06-10: Merge CommonClaude global CLAUDE.md sections into project CLAUDE.md

- User request: merge the contents of
  https://github.com/coport-uni/CommonClaude CLAUDE.md into this
  project's CLAUDE.md (see LP §W1, §W2).
- Gap analysis: project CLAUDE.md already mirrors CommonClaude
  §1-§6, §8-§10 (Rule Priority, MIT convention, debug files, task
  management, testing, linting, exceptions, learned patterns).
  Missing: §7 MCP servers (Serena/Context7/Fetch), §11 Commit
  Messages, §12 Branching Strategy, §13 .gitignore, §14 Versioning,
  §15 PR Guidelines, §16 Git Automation, §17 References.
- Merge policy: import missing global sections verbatim where they
  fit; keep project-specific overrides (110-col ruff, Fairino
  examples, direct-to-main task workflow) per Rule Priority and add
  explicit override notes where the global rule is superseded.
- [x] Extend "Research Before Coding" with the §7 MCP server table
      (Serena, Context7, Fetch), rules, and setup commands.
- [x] Add "Commit Messages" (Conventional Commits) section.
- [x] Add "Branching Strategy" (GitHub Flow) section with a project
      override note for the existing direct-to-main task workflow.
- [x] Add ".gitignore", "Versioning" (SemVer), "Pull Request
      Guidelines", "Git Automation" (pre-commit), and "References
      (Git Convention)" sections; also merged the §2 docstring
      example into MIT Code Convention.
- [x] Create GitHub issue via gh, commit, and push —
      https://github.com/coport-uni/FR5ControllerVLA/issues/76.

## 2026-06-10: Replace pynput record shortcuts with evdev reader

- User picked remediation option 2 from the 2026-04-22 / 2026-06-10
  diagnoses (see LP §G5, gh issue #36): read Right/Left/Esc directly
  from `/dev/input/event*` via `evdev` so the lerobot-record
  shortcuts work on Wayland and over SSH, independent of window
  focus.
- Design: new `src/lerobot/utils/evdev_keyboard.py` with an
  `EvdevKeyboardListener` class (device scan + select() reader
  thread, `start()`/`stop()` API). `init_keyboard_listener()` tries
  evdev first and falls back to the existing pynput path when no
  readable keyboard device exists, so upstream behavior is kept on
  machines without input-group access.
- [x] Append ToDo.md entry and create gh issue
- [x] Add `evdev` (linux-only marker) to pyproject dependencies
- [x] Implement `EvdevKeyboardListener` (80-col MIT convention)
- [x] Rewire `init_keyboard_listener()` with evdev-first fallback
- [x] Fix `lerobot_record.py` stop path: `if listener:` instead of
      `if not is_headless() and listener:`
- [x] Grant `/dev/input` access: add user to `input` group
      (re-login needed) and document it (reboot was required, see
      LP §E7; documented via #80)
- [x] ruff check + format on touched files
- [x] Smoke-test device discovery in conda `lerobot` env
- [x] Commit, push, update gh issue (2ec9d12e, #77)

## 2026-06-10: Check and minimize FR5 collision sensitivity

- User request: read the FR5's current collision detection
  sensitivity and set it to the least sensitive value.
- Fairino collision grade is per-joint 1-10 where 1 = most
  sensitive and 10 = least sensitive (`SetAnticollision(mode=0,
  level, config)`); the Python SDK exposes no getter, so the
  current value is probed via XMLRPC introspection first
  (see LP §E1 for network defaults).
- Least-sensitive target: `SetAnticollision(0, [10]*6, 1)` with
  `config=1` so the controller persists it across reboots.
- [x] Append ToDo.md entry and create gh issue (#78)
- [x] Write `claude_test/check_collision_level.py` (query via
      XMLRPC introspection, then set grade 10 on all joints)
- [ ] Run against follower at 192.168.58.2 and verify errcode 0
- [x] Update `claude_test/README.md` index
- [ ] Commit, push, update gh issue

## 2026-06-11: Clean stale dataset dir before hf download in record script

- User request: in `5__fr5_record.sh`, delete the local dataset
  directory (`outputs/datasets/FR5_task1_move_the_brown_colored_
  glass_bottle_to_the_designated_location`) if it exists, before
  the `hf download` step, so the download always starts clean.
- [x] Append ToDo.md entry and create gh issue
- [x] Add `rm -rf <dataset dir>` before `hf download`
- [x] Commit, push, update gh issue (28fe22c4, #79)

## 2026-06-11: Document input-group propagation gotcha in LearnedPatterns

- Follow-up to the evdev shortcut work (see LP §Q11, gh issue #77):
  after `sudo usermod -aG input` the record shortcuts still failed
  because the desktop re-login did not restart the `systemd --user`
  daemon (started before the group change), so VSCode and all
  terminals inherited the stale supplementary-group list. A full
  reboot fixed it; evdev now discovers 4 keyboard devices.
- [x] Append ToDo.md entry and create gh issue (#80)
- [x] Append E7 to LearnedPatterns.md §5 and refresh the header
- [x] Check off the open input-group item in the 2026-06-10 evdev
      section
- [x] Commit and push (2271555b, #80)

## 2026-06-11: Document RealSense xHCI HC-died gotcha in LearnedPatterns

- User asked why the RealSense D455 ignored replugging until a
  reboot. Previous-boot kernel log (18:46) showed the NUC's
  USB4-side xHCI controller `0000:00:0d.0` hung on a
  `stop endpoint` command and was marked dead (`HC died`), taking
  down the whole USB bus — a dead host controller cannot
  enumerate a replugged device; reboot reinitializes it. PCI
  remove/rescan recovers without reboot. Contributing factor:
  the camera hangs off the external dock hub on that controller.
- [x] Append ToDo.md entry and create gh issue (#81)
- [x] Append E8 to LearnedPatterns.md §5 and refresh the header
- [x] Commit and push (70f85e6b, #81)

## 2026-06-12: Diagnose RealSense read-loop crash at stop_event check

- User asked why the RealSense background read thread raised an
  exception at `camera_realsense.py:474`
  (`while not self.stop_event.is_set():`). Diagnosis: a shutdown
  race in `_stop_read_thread()` — it joins the thread with a 2 s
  timeout, then unconditionally sets `self.stop_event = None`
  (line 523). If `_read_from_hardware()` blocks past the join
  timeout (e.g. the xHCI HC-died USB hang, see LP §E8), the
  still-alive thread re-enters the loop and calls
  `None.is_set()` → AttributeError. Harmless at shutdown but it
  masks the underlying USB hang. Fix deferred until requested:
  snapshot `stop_event` locally in `_read_loop`, or only null it
  after a confirmed join.
- [x] Diagnose root cause and report to user (see LP §E8)
- [x] Append ToDo.md entry and create gh issue
- [x] Commit and push (ccaded17, #82)
- [x] Assess dataset impact of the fix (none — shutdown-path only;
      crash occurs after episodes are saved) and record on #82

## 2026-06-12: Fix RealSense stop_event race with minimal change

- User approved the minimal fix for the shutdown race diagnosed in
  the previous entry (gh issue #82, see LP §E8): snapshot
  `self.stop_event` into a local variable at `_read_loop` entry so
  the loop condition no longer dereferences the attribute that
  `_stop_read_thread()` nulls after a timed-out join.
- [x] Append ToDo.md entry (fix tracked on existing issue #82)
- [x] Snapshot stop_event locally in `_read_loop`
- [x] Run ruff check/format and camera tests (41 failures are
      pre-existing, identical on baseline)
- [x] Commit, push, update gh issue #82 (aef21e80, #82 closed)

## 2026-06-12: Diagnose mid-recording crash (D455 drops off USB hub)

- User reported lerobot-record aborting mid-episode with the
  stop_event traceback reappearing. Diagnosis (gh issue #83): the
  D455 sits behind a shared external USB3.2 hub (with an r8152 LAN
  adapter) on the LP §E8 controller; kernel log shows 32x EPROTO
  (-71) errors and three full hub resets in four minutes, so the
  camera physically drops mid-episode, the read thread dies after
  11 consecutive failures, and get_observation kills the session.
- Also found: the working tree carries an uncommitted revert of the
  #82 race fix (aef21e80) — awaiting user confirmation whether it
  was intentional before discarding or keeping it.
- [x] Inspect kernel log and USB topology, identify hub instability
      as root cause (see LP §E8, §E9)
- [x] Create gh issue with findings and hardware-first
      recommendations (#83)
- [x] Append LP §E9 (shared-hub EPROTO pattern)
- [x] Hardware mitigation (move D455 off hub, swap cable) — user
      action, tracked on #83 (verified 2026-06-12: repeater cable
      + dedicated hub, LP §E9 updated)

## 2026-06-12: 7__train_act_task1_h200.sh 의 batch_size 를 H200 141 GB VRAM 80% 기준으로 튜닝

### Background
[7__train_act_task1_h200.sh](7__train_act_task1_h200.sh) 는 헤더 주석상
H200 2x, `--batch_size=8`, `global=16` 가정으로 작성되었으나, 실제 값은
3090 (#56) 결과를 그대로 carry-over 한 `--batch_size=40`,
`--mixed_precision=fp16`, `--policy.optimizer_lr=2.2e-5` 다. 학습 머신은
H200 NVL 141 GB x 2 (143,771 MiB / GPU) 이므로 80 % 라인
(~115,000 MiB) 까지 throughput 을 늘리고, 변경된 (per-GPU, 글로벌)
배치에 맞는 learning rate 권장값을 함께 도출한다.

ACT 의 LeRobot 기본 옵티마이저 설정은
[configuration_act.py:127-129](src/lerobot/policies/act/configuration_act.py#L127-L129):
`optimizer_lr=1e-5`, `optimizer_lr_backbone=1e-5`,
`optimizer_weight_decay=1e-4`. LeRobot 멀티 GPU 문서는 LR auto-scale 을
하지 않으므로 AdamW + no-warmup 환경 권장 경험칙인 sqrt scaling 을 사용
한다 (see #56).

### Decisions (사용자 확정)
- Probe 우선. 추정값 (bs≈240, lr≈4.5e-5) 은 ladder 의 출발점으로만 사용.
- `--steps=100000` 그대로 유지 (epoch 늘려 수렴 여유).
- 스크립트 헤더 주석도 새 setting 에 맞춰 함께 갱신.
- mixed_precision 은 H200 Hopper 권장값인 **bf16** 으로 전환.
- 데이터셋은 스크립트가 가리키는
  `coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_50`
  그대로 사용 (probe 도 동일 데이터셋).
- Probe 동안 wandb / HF Hub push 비활성화 (7__train_act_task1_h200.sh
  는 수정하지 않고 claude_test/ probe 래퍼에서만 override).
- 첫 probe duration 300 s (데이터셋 캐시 미스 여유), 이후 150 s.

### Work items
- [x] `claude_test/probe_vram_batch_h200_task1.sh` 작성: #56 의
      `probe_vram_batch.sh` 를 base 로 (num_processes=2, bf16,
      task1 dataset, 143771 MiB 분모) 만든 H200 전용 wrapper.
- [x] 후보 ladder probe: per-GPU `bs ∈ {64, 128, 192, 240, 280}`
      부터 시작해 ~115,000 MiB peak 에 가장 가까운 후보까지
      필요 시 binary refine. 결과: bs=248 binary-refined
      (GPU0 78.6%, GPU1 80.8%).
- [x] 선정된 글로벌 배치에 대한 LR 권장값 계산. sqrt: 4.5e-5
      (1e-5 × sqrt(496/24) ≈ 4.55). linear: 2.07e-4 (warmup 시).
- [x] [7__train_act_task1_h200.sh](7__train_act_task1_h200.sh)
      값과 헤더 주석 동시 갱신: BATCH_SIZE=496 / GPU_NUMBER=2 →
      per-GPU 248, `mixed_precision=bf16`, `optimizer_lr=4.5e-5`,
      `optimizer_lr_backbone=4.5e-5`, `--steps=100000` 유지.
      추가로 사용자가 추가하던 `${A} / ${B}` 리터럴 표현을
      `$((A / B))` 산술로 교정, pre-existing JOB_NAME 이중
      `coport-uni/` prefix 버그도 동시 정리 (사용자 승인).
- [x] `bash -n 7__train_act_task1_h200.sh` 구문 검증.
- [x] `claude_test/README.md` 업데이트 (새 probe 스크립트 행 추가).
- [x] gh issue create (#84).
- [x] commit + push (1e0880b5), gh issue 코멘트 등록 후 close.

### Out of scope
- 다른 `7__train_*` 스크립트의 H200 튜닝.
- Probe 결과를 LeRobot multi_gpu_training 문서 또는 본 repo 의
  CLAUDE.md / LP 에 추가하는 작업 (별도 task).

## 2026-06-12: Add missing v3.0 git tag to FR5_task1 brown-bottle dataset

### Background
가 #84 probe 를 시작하자 `lerobot-train` 이
`RevisionNotFoundError` 로 실패했다. 원인은 dataset
`coport-uni/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_50`
의 HF Hub repo 에 `v3.0` git tag 가 없었던 것. `meta/info.json` 자체는
`codebase_version: v3.0` 이지만 LeRobot 의 `get_safe_version()`
([utils.py:289](src/lerobot/datasets/utils.py#L289)) 는 git tag 가 없으면
`RevisionNotFoundError` 를 던진다 (그리고 huggingface_hub 의 새 버전
에서는 그 예외 생성 자체가 `response=` 미전달로 다시 죽음). 기존
`FR5_pick_red_colored_marker_to_box` 에는 `v3.0` 태그가 있어 정상이었다.

### Work items
- [x] `hub_api.create_tag(repo, tag='v3.0', revision='main',
      repo_type='dataset')` 로 태그 생성 (사용자 승인).
- [x] 태그 생성 후 #84 probe 재개하여 정상 동작 확인.

## 2026-06-12: Diagnose mid-training stop of 7__train_act_task1_h200.sh at step 148 (diagnosis only)

### Background
사용자가 "왜 학습이 중간에 멈췄나" 질문. `7__train_act_task1_h200.sh`
(#84 에서 튜닝, run zwmmk3ro) 가 `--steps=100000` 목표 중 **step 148**
에서 멈춤. 진단 결과 코드/CUDA 예외가 아니라 외부 신호에 의한 강제
종료로 판단.

### Evidence
- [output.log](outputs/train/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location_50/wandb/run-20260612_080647-zwmmk3ro/files/output.log)
  마지막 줄이 `148/100000 [07:03<29:52:03, 1.08s/step]` 에서 끊김.
- Python traceback / Exception / CUDA OOM 메시지 **없음**.
- wandb debug-internal.log 에 정상 finish/atexit 기록 **없음**
  → atexit 핸들러도 못 돌리고 즉사 = SIGKILL 패턴 (OOM-killer 또는
  `kill -9` / 컨테이너·SSH 세션 종료).
- `checkpoints/` 디렉토리 없음: `save_freq=10000` 이라 첫 체크포인트
  (step 10000) 전에 죽어 복구 지점이 0 → `--resume` 불가.

### Secondary finding (성능)
- H200 2장인데 `1.08s/step` (ETA ~30h) 로 비정상 느림.
- 원인: torch._dynamo `recompile_limit (8)` 도달. `modeling_act.py`
  [L148](src/lerobot/policies/act/modeling_act.py#L148) / L157 의
  `l1_loss.item()` / `mean_kld.item()` 가 매 step graph break +
  재컴파일 유발.

### Recommended remediation (별도 task, 사용자 승인 필요)
- [ ] H200 박스에서 `dmesg -T | grep -iE "killed process|oom"` 로
      OOM-killer 여부 확정. 잡히면 RAM OOM → `--num_workers` 축소.
- [ ] 세션 끊김 방지: `tmux`/`nohup` 안에서 실행.
- [ ] `save_freq` 를 1000 수준으로 낮춰 초반 복구 지점 확보.
- [ ] torch.compile graph break (`.item()`) 완화 검토.

### Out of scope (this task)
- 위 remediation 의 실제 코드/스크립트 수정 (진단만 수행).

## 2026-06-12: Write B200 x4 Pi0 training script from 7__train_pi0_adv.sh

Based on `7__train_act_task1_h200.sh` conventions (GPU_NUMBER +
global BATCH_SIZE divided per-GPU), adapt `7__train_pi0_adv.sh`
(2 x H200) to 4 x B200. (see LP §5 conda roots, §3 NCCL P2P)

- [x] Create `7__train_pi0_adv_b200.sh` (num_processes=4)
- [x] Keep per-GPU batch=160 as proven-safe on B200 180 GB
      (issue #55 probe: 160 -> 79.6% on H200 143.77 GB, ~64% on
      B200; 176 OOM was a transient first-step spike, not steady)
- [x] Re-scale lr: effective batch 320->640 (20x openpi 32),
      sqrt rule 2.5e-5*sqrt(20)=1.1e-4
- [x] Keep bf16, compile, gradient_checkpointing, full fine-tune,
      warmup=1000; suffix JOB_NAME/repo_id with _b200
- [x] Validate `bash -n` syntax + computed values (lr/batch/save_freq)
- [x] Register GitHub issue via `gh issue create` (#86)

## 2026-06-12: Probe B200 VRAM to set Pi0 batch + LR (4 x B200)

Run the issue-#55 VRAM probe ladder on this 4 x B200 box (183359 MiB
= 179.06 GB/GPU) to replace the H200-extrapolated batch=160 in
`7__train_pi0_adv_b200.sh` with an empirically measured ~80%-VRAM
batch, then sqrt-scale the LR from openpi's 2.5e-5 baseline.
(see LP §1 NCCL P2P, §5 conda roots)

Context found during setup:
- conda lives at `/NHNHOME/workspace/sungwoo/miniforge3` (miniforge3),
  NOT in the discovery loop of the existing scripts -> probe driver
  adds it. Env: torch 2.10.0+cu128 (CUDA 12.8, Blackwell sm_100), 4 GPU.
- Dataset coport-uni/FR5_pick_red_colored_marker_to_box is public;
  hf + gh authenticated as coport-uni.

- [x] Write `claude_test/probe_pi0_vram_b200.sh` (num_processes=4,
      sample 4 GPUs, miniforge3 conda path)
- [x] Run ladder per-GPU batch 176/192/208/224/240 (50 steps, bf16 +
      compile + gradient_checkpointing, full fine-tune; 3 attempts --
      v3 valid after P2P-on + compile_mode=default fixes)
- [x] Pick highest batch at <=~80% of 183359 MiB without OOM:
      per-GPU 176 (73.9% peak, 50/50 steps; 192 OOMs)
- [x] sqrt-scale LR: 2.5e-5 * sqrt(704/32) = 1.17e-4 -> 1.2e-4
- [x] Write claude_test/probe_logs/SUMMARY_b200.md + update README
- [x] Update batch/LR in 7__train_pi0_adv_b200.sh with measured values
      (+ compile_mode=default, pinned checkpoint, env fixes)
- [x] Register GitHub issue via gh issue create (#87)

### Update 2026-06-13: probe blocked by pi0_base version drift
First ladder run failed at startup on all batches (not VRAM): the
`lerobot/pi0_base` HEAD (commit 25c379b, 2026-06-03 "Add relative
action processor steps") ships a `policy_preprocessor.json` with step
`relative_actions_processor` (enabled=false) that this v0.5.1 fork's
ProcessorStepRegistry does not have -> ImportError before training.
Fix: pin to the prior revision `26b99b9439ac` (2026-01-22), whose
preprocessor lacks that step (weights identical). Downloaded to
`models/pi0_base_v051compat/`; validating a short run, then rerun
ladder + NCCL probe pointing at the local compatible checkpoint.

### Update 2026-06-13 (cont.): third blocker -- noexec /tmp breaks torch.compile
With the processor + weights issues handled, the probe still crashed at
step 0/50 under `compile_model=true`:
`ImportError: /tmp/torchinductor_*/...cuda_utils...so: failed to map
segment from shared object`. Cause: `/tmp` is mounted `noexec`
(`rw,nosuid,nodev,noexec`), so triton/inductor cannot dlopen the
kernels it writes there. `compile_model=false` (loadcheck) was immune.
This would also crash the real `7__train_pi0_adv_b200.sh`
(compile_model=true).
Fix: redirect caches to an exec-allowed dir
(`TORCHINDUCTOR_CACHE_DIR`, `TRITON_CACHE_DIR` under repo `.cache/`).
Added to both probe drivers; verifying with a compile smoke test, then
relaunching the ladder + NCCL sweep.

### Update 2026-06-13 (cont. 2): NCCL verdict + fourth blocker (max-autotune)
NCCL sweep (batch=96, 40 steps each):
- p2p_off (production default from issue #30): HANG, first allreduce
  never clears (timeout 480 s).
- p2p_on: OK, 4.00 s/step  <- fastest, correct setting for this box.
- p2p_shm_off (pure socket): OK but 6.67 s/step (67 % slower).
=> On bare-metal B200 NVLink, NCCL_P2P_DISABLE=1 must NOT be set
   (exact opposite of the Docker H200 box in issue #30).

Ladder v2 (P2P-on) still died at step 0 for batch 176-240:
CUDA illegal memory access during *Triton GEMM autotune*
(select_algorithm.py). Root cause: pi0's default
compile_mode="max-autotune" (configuration_pi0.py:76) autotunes
triton_mm kernels; at per-GPU batch >= ~176 a candidate kernel
crashes on sm_100 (batch 96/64 runs passed). Fix: probe driver now
passes --policy.compile_mode=default (COMPILE_MODE env knob);
ladder v3 running. Production script must add the same flag.

## 2026-06-13: Fix pi0 vision-tower random init via key remap (option b)

Blocker #2 from the B200 probe (issue #87, SUMMARY_b200.md): under
transformers 5.11.0 the PaliGemma vision tower is stored as
`vision_tower.vision_model.*` in the pi0_base checkpoint but the model
expects `vision_tower.*`; `from_pretrained` loads with `strict=False`,
silently leaving the vision tower randomly initialized. User chose
option (b): add a key remap in the load path instead of pinning
transformers. (see LP Q13 once recorded)

- [x] Record B200 probe lessons in LearnedPatterns.md (Q12-Q14,
      E10-E12)
- [x] Implement vision-tower key remap in the pi0 weight-load path
      (conditional bidirectional remap in _fix_pytorch_state_dict_keys)
- [x] Verify: from_pretrained(models/pi0_base_v051compat) loads with
      zero vision-tower missing/unexpected keys; weights match the
      safetensors content (437/437 tensors,
      claude_test/verify_pi0_vision_load.py: All keys loaded)
- [x] ruff check + format on modified Python files (all clean)
- [x] Register GitHub issue via gh issue create (#88)

## 2026-06-15: Add B200 pi05-adv training script (probe + write + test)

Create a 4 x B200 production training script for the Pi0.5 "adv"
fine-tune, derived from 7__train_pi05_adv_h200.sh (pi05 policy + MEAN_STD
norm + weight_decay=1e-10 + red-marker dataset) and the B200 environment
conventions in 7__train_pi0_task1_b200.sh (miniforge3 conda root, PATH
fix, torchinductor/triton cache redirect for noexec /tmp, P2P stays ON,
compile_mode=default for sm_100, GPU_NUMBER=4). pi05 per-GPU batch on
B200 is not yet probed (only pi0 = 176 @ 73.9 %), so probe it first.
(see LP / SUMMARY_b200.md / SUMMARY_pi05.md)

- [x] Write claude_test/probe_pi05_vram_b200.sh (pi05 policy options +
      B200 env from probe_pi0_vram_b200.sh; 4 GPUs, samples GPUs 0-3)
- [x] Run the VRAM probe ladder (128-168 per GPU); chosen per-GPU
      batch=152 (eff 608) @ 71.5 % VRAM, 5.30 s/step. NOTE: ceiling is
      the Triton compile shared-mem limit (160 fails to compile), not
      VRAM. Also fixed 3 setup blockers: pi05_base v051compat snapshot
      (relative/absolute_actions_processor steps), compile_mode=default
      (sm_100 max-autotune crash), NCCL P2P on + /tmp cache redirect.
- [x] Record results in claude_test/probe_logs/SUMMARY_pi05_b200.md
- [x] Write 7__train_pi05_adv_b200.sh (batch 152/GPU=608 eff,
      steps=1570 ~= 8.00 epoch, lr=1.1e-4, save_freq=157)
- [x] ruff is N/A (bash); run bash -n on both new scripts (OK)
- [x] Smoke-test the production script for 6 steps (push/wandb off):
      exit 0, 6/6 steps, no errors
- [x] Register GitHub issue via gh issue create (#89)
- [x] Commit and push (5a5b692c, closes #89)

## 2026-06-15: Check wandb login status (read-only diagnostic)

Verify whether the workstation is authenticated to Weights & Biases so
that upcoming pi05 B200 training runs log correctly.

- [x] Inspect ~/.netrc for api.wandb.ai credentials (key present,
      stored Jun 13)
- [x] Validate the API key against api.wandb.ai GraphQL viewer query:
      logged in as username=ohsungwoo, entity=ohsungwoo-unist,
      email=ohsungwoo@unist.ac.kr
- [x] Note: no wandb module in /usr/bin/python3 and no project .venv;
      auth travels via ~/.netrc to whichever env runs training
- [ ] GitHub issue: skipped (read-only status check, no repo change)

## 2026-06-15: Add wandb login pre-flight check to B200 pi05 script

Add a wandb authentication check to 7__train_pi05_task1_b200.sh so the
run fails fast if the env is not logged in, instead of stalling on an
interactive prompt mid-training (the script already sets
--wandb.enable=true).

- [x] Add WANDB_USER pre-flight block after the HF_USER check: query
      wandb.Api().viewer.username via the active lerobot env python;
      exit 1 with a clear message if empty (no valid API key)
- [x] bash -n syntax check (OK); ruff N/A (bash script)

## 2026-06-17: Add dataset version-tag pre-flight to ACT H200 script

Training the _100 dataset aborted with FileNotFoundError on
meta/info.json, masking the real cause: LeRobot's get_safe_version
(utils.py:265) needs a git tag matching info.json codebase_version,
and the _100 dataset on the Hub had no tags (the _50 one had v3.0).
The RevisionNotFoundError it tries to raise also crashes on a
huggingface_hub API change (HfHubHTTPError now requires `response`),
hiding the message. Add a pre-flight to 7__train_act_task1_h200.sh
that checks the dataset for a tag matching codebase_version and
creates it if missing (see LP §2 G4, §3 Q12).

- [x] Add tag pre-flight block after HF_USER/JOB_NAME: read
      codebase_version from the Hub info.json, list repo tags, and
      create the tag via HfApi().create_tag if absent
- [x] bash -n syntax check (OK); ruff N/A (bash script)
- [x] Tag the existing _100 dataset to unblock the current run (created v3.0; LeRobotDataset loads 100 ep / 64223 frames)
- [x] Append LearnedPatterns entry for the missing-tag gotcha

## 2026-06-22: Install conda and create lerobot Python 3.12 env

Set up a conda toolchain on a fresh machine (no conda present) and
create the project's runtime environment per CLAUDE.md.

- [x] Confirm no existing conda (which conda/mamba empty; no
      ~/miniconda3, ~/anaconda3, /opt/conda); arch x86_64, Ubuntu 24.04
- [x] Download and silently install Miniconda to ~/miniconda3
      (conda 26.3.2); run conda init bash
- [x] Create env "lerobot" with python=3.12 via -c conda-forge
      --override-channels (defaults channel blocked on Anaconda ToS)
- [x] Verify: env at ~/miniconda3/envs/lerobot, Python 3.12.13
- [x] Transfer ToDo.md ownership to appeal via gcsudo to record this
      entry (file was owned by ykcho, read-only to appeal)
- [x] GitHub issue registered (#94) after gh login + git credential
      setup (coport-uni/FR5ControllerVLA)

## 2026-06-22: Editable-install lerobot into the lerobot conda env

Install the project per CLAUDE.md Build & Test Commands so the FR5
VLA fork is importable and the CLI entry points are available.

- [x] Install pip into the conda env (conda-forge python ships no pip);
      pip 26.1.2
- [x] Run pip install -e ".[test,dev]" in the lerobot env (exit 0);
      pulls torch 2.10.0 + CUDA 12.8 wheels, datasets, accelerate,
      wandb, pytest, mypy, pre-commit
- [x] Verify: import lerobot -> 0.5.1; editable source at src/lerobot;
      CLI lerobot-train / lerobot-teleoperate registered
- [x] GitHub issue registered (#95) — depends on #94
      (coport-uni/FR5ControllerVLA)

## 2026-06-22: Fix pi0/pi05 train crash (missing transformers) on B200 env

`7__train_pi0_task1_b200.sh` crashed at "Creating policy" with
`TypeError: 'NoneType' object is not subscriptable` at
`modeling_pi0.py:359` (`CONFIG_MAPPING["paligemma"]`). Root cause: the
active conda env `/home/appeal/miniconda3/envs/lerobot` had no
`transformers`, so `_transformers_available` was False and
`CONFIG_MAPPING` fell back to None. ACT training is unaffected (no
transformers dependency); only pi0/pi05 need the `[pi]` extra.

- [x] Diagnose crash from wandb output.log traceback (CONFIG_MAPPING None)
- [x] Confirm `transformers` absent in the active lerobot env
- [x] Install pi0/pi05 deps: pip install -e ".[pi]"
      -> transformers 5.12.1, scipy 1.18.0, tokenizers 0.22.2
- [x] Verify pi0 + pi05 modules import and CONFIG_MAPPING is populated
- [x] Update 7__train_pi0_task1_b200.sh header: verify GPU count with
      `nvidia-smi -L` every run and set GPU_NUMBER to match (count is
      not fixed across boxes/sessions)
- [x] GitHub issue registered (#96) — coport-uni/FR5ControllerVLA

## 2026-06-16: Upload FR5_task1 dataset to existing HF repo

- User asked to upload the local LeRobot v3.0 dataset
  `outputs/datasets/FR5_task1_move_the_brown_colored_glass_bottle_to_the_designated_location`
  (80 episodes, 53,717 frames, 3.83 GB) to its existing public
  HuggingFace repo `coport-uni/FR5_task1_...`. Local carried one new
  `file-011` (data + meta/episodes + 3 videos) plus refreshed
  metadata not yet on the Hub; user confirmed "이어서 올려줘"
  (append to the existing repo).
- [x] Confirm repo name and visibility with user (same long name,
      keep existing public repo)
- [x] Verify HF auth (huggingface_hub, account coport-uni) and that
      the remote repo exists (60 files, public)
- [x] upload_folder additive push (ignore .cache); remote 60 -> 65
      files, file-011 data/video present
- [x] Verify uploaded files on the Hub

## 2026-06-22: Fix FR5 teleop crash when a joint hits its motor limit

- gh issue: #93
- User reported the FR5 "connection dying" when a motor reaches its
  limit during leader-follower teleoperation
  (`4__fr5_leader_follower.sh`, follower 192.168.58.2 mirrors leader
  192.168.59.2). The visible symptom is a Rerun gRPC error
  (`re_grpc_client::write ... transport error`), which appears as the
  main `lerobot-teleoperate` process tears down.
- Root cause (confirmed by reading code, no hardware): when a joint
  reaches its limit the controller faults/safety-stops; the SDK TCP
  state thread resets (`reconnect_flag`), so `GetActualJointPosDegree`
  returns nonzero after its 3 retries; `FairinoFollower.get_observation`
  and `FairinoLeader.get_action` then raise `RuntimeError`; `teleop_loop`
  has no per-iteration guard (outer try only catches KeyboardInterrupt),
  so the process exits and Rerun logs the transport error.
- User choices (clarified before work): symptom = process crash behind
  the Rerun error; mode = follower mirroring leader; recovery policy =
  auto-recover and continue.

### A. Stop the crash (root cause of the transport error)
- [x] `FairinoFollower.get_observation`: on joint-read failure return
      last-known joints (track `_last_joints`) with a rate-limited
      warning instead of raising RuntimeError. (folded into B)
- [x] `FairinoLeader.get_action`: same last-known-joints fallback
      instead of raising RuntimeError. (folded into B)

### B. Auto-detect and recover the controller fault (see LP §G2, §G9)
- [x] Add a fault read from the state package (`GetRobotErrorCode` ->
      main_code/sub_code, `GetSafetyCode`). Implemented as `_detect_fault`
      reading main_code/safety_stop0/1_state directly off robot_state_pkg
      (no extra RPC; avoids SDK @log_call spam at 20 Hz).
- [x] Follower: broaden recovery from error-14-only to any fault
      (StopMotion -> ServoMoveEnd -> ResetAllError -> RobotEnable(1)
      -> Mode(0) -> ServoMoveStart -> resync `_commanded` from a live
      read). Gate behind `_servo_paused` so it cannot race the gripper
      worker (LP §G9); rate-limit recovery attempts.
- [x] Leader: on fault, re-run ResetAllError -> RobotEnable(1) ->
      DragTeachSwitch(1), rate-limited.

### C. Prevent reaching the limit (follower clamp margin)
- [ ] Add `limit_margin_deg` (default ~2.0) to FairinoFollowerConfig.
      (DEFERRED -- user chose to proceed with B only this pass.)
- [ ] At connect, read firmware soft limits via `GetJointSoftLimitDeg`,
      intersect with config limits, and clamp `send_action` to the
      margin-adjusted bounds so ServoJ never drives into the fault zone.

### Validation and housekeeping
- [x] Add `claude_test/` debug script to exercise fault detection /
      recovery logic without hardware where possible; document in
      `claude_test/README.md`. (debug_limit_fault_recovery.py, 16/16 pass)
- [x] ruff check + format on all touched files (80-col for new code).
- [x] Create gh issue (#93); append a LearnedPatterns entry (LP §2 G11);
      commit and push.
- [ ] Hardware validation on the robot (limit-fault recovery on
      192.168.58.2 / 192.168.59.2) -- requires the robot, do with user.
