# Cobot1 - DSR2 Task Control System

두산 로봇(DSR2) 모션 제어를 위한 ROS2 패키지.

## 빠른 시작

```bash
# 1. 시뮬레이터 실행
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual

# 2. Task 시스템 실행 (새 터미널)
ros2 launch cobot1 task_system.launch.py

# 3. CLI로 제어 (새 터미널)
ros2 run cobot1 task_cli start pick_and_place
ros2 run cobot1 task_cli pause    # 일시 정지
ros2 run cobot1 task_cli resume   # 재개
ros2 run cobot1 task_cli stop     # 정지
ros2 run cobot1 task_cli status   # 상태 확인
```

## 주요 기능

- **start/stop**: Task 시작/정지
- **pause/resume**: 모션 중간 일시 정지/재개 (DSR2 `move_pause`/`move_resume` 활용)
- **status**: Task 상태 및 로봇 상태 확인

## 핵심 포인트


| 항목           | 권장                           | 비권장                        |
| ------------ | ---------------------------- | -------------------------- |
| 모션 함수        | `amovel()`, `amovej()` (비동기) | `movel()`, `movej()` (블로킹) |
| pause/resume | 서비스 직접 호출                    | DSR_ROBOT2.py에 없음          |
| ROS2 spin    | `spin_once()` 루프             | `executor.spin()`          |


## UI Bridge + Firebase sync

`ui_bridge` 는 웹 UI 명령을 `task_controller` 로 포워딩하고, UI 상태를
`/<ui_ns>/ui/status` (기본 `m0609`) 토픽으로 발행한다.
동시에 Firebase Realtime Database 로 동기화할 수 있다.

```bash
ros2 run cobot1 ui_bridge
```

### 1회 셋업 (Firebase 사용 시)

```bash
pip install firebase-admin      # admin SDK 사용 권장. 미설치 시 REST mode 로 fallback.

mkdir -p ~/.config/cobot1/firebase
chmod 700 ~/.config/cobot1/firebase

# env 파일: 패키지의 example 을 복사
cp $(ros2 pkg prefix cobot1)/share/cobot1/config/firebase_sync.env.example \
   ~/.config/cobot1/firebase/firebase_sync.env

# 서비스 계정 JSON 도 같은 폴더에
cp <your-project>-firebase-adminsdk-*.json ~/.config/cobot1/firebase/
chmod 600 ~/.config/cobot1/firebase/*
```

`ui_bridge` 는 `FIREBASE_SYNC_ENV_FILE` 환경변수 → `~/.config/cobot1/firebase/firebase_sync.env`
순으로 찾고, env 가 없거나 `FIREBASE_DATABASE_URL` 이 비면 sync 를 자동 비활성한다.

### 경로 설계

- 스냅샷: `FIREBASE_STATUS_PATH` (default `robots/m0609/status`) — 매 publish (0.2s) 마다 최신 상태를 `set()`. 동일 시그니처는 `FIREBASE_SYNC_HEARTBEAT_SEC` 주기 내에서 스킵.
- 이벤트: `FIREBASE_EVENTS_PATH` (default `robots/m0609/events`) — 의미 있는 상태 전이 (`task_started`, `task_paused`, `task_completed`, `task_error`, `emergency_stop_*`, `bridge_startup`) 시에만 `push()`.

### 검증

```bash
ros2 topic echo /m0609/ui/status --once
# firebase_sync_enabled / firebase_sync_mode / firebase_sync_status / firebase_sync_last_synced_at 확인
```

## 문서

자세한 아키텍처와 구현 설명: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)