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
