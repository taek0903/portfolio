# Firebase Realtime Database 연동

`ui_bridge` 노드가 UI 상태 스냅샷과 이벤트 로그를 Firebase Realtime Database 로
동기화하는 구조 문서.

## 1. 목적

- 웹/모바일 UI 가 로봇과 같은 네트워크에 없어도 상태를 볼 수 있도록 중앙화.
- Task 완료(`complete`) 및 에러(`error`: task error, SAFE_STOP 진입)를 append-only
로 기록해 사후 분석 가능.
- 브라우저 UI (`references/robot_web_ui`) 가 기대하는 `firebase_sync_*` 필드를
그대로 발행해 UI 동기화 상태 표시까지 지원.

## 2. 데이터 흐름

```
┌──────────────┐ serving_cmd ┌──────────────┐  task/start ┌──────────────────┐
│  Web UI      │ pause_cmd   │              │  task/stop  │                  │
│ (home_ui.js) │ ──────────► │  ui_bridge   │ ──────────► │  task_controller │
│              │  safety_cmd │   Node       │             │                  │
│              │             │              │ ◄───────────│                  │
│              │  ui/status  │              │  task/state │                  │
│              │ ◄────────── │              │             │                  │
└──────────────┘             └──────┬───────┘             └──────────────────┘
   ▲                                │
   │ 상태표시                       │ FirebaseRealtimeSync
   │ (sync 뱃지)                    │   · admin SDK (기본)
   │                                │   · REST fallback
   └─────── Firebase DB ◄───────────┘
              │
              ├── robots/m0609/status   (최신 스냅샷, set())
              └── robots/m0609/events   (이벤트 로그, push())
```

- `ui_bridge` 는 0.2 초 주기로 UI status payload 를 만들고 동일한 payload 를
Firebase `status` 경로에 `set()` 으로 덮어쓴다.
- 상태 전이 콜백(`_task_state_callback`, `_handle_safety_cmd`) 에서만
`events` 경로에 `push()` 로 append 한다. 주기 publish 에서는 push 하지 않아
DB 용량을 보호한다.

## 3. 설정 파일 컨벤션

### 3.1 경로


| 종류           | 위치                                                     | git 관리 | 비고                                            |
| ------------ | ------------------------------------------------------ | ------ | --------------------------------------------- |
| 예시 env (템플릿) | `src/cobot1/config/firebase_sync.env.example`          | 커밋     | `setup.py` 가 `share/cobot1/config/` 로 install |
| 실제 env       | `~/.config/cobot1/firebase/firebase_sync.env`          | 제외     | XDG Base Directory 표준 위치                      |
| 서비스 계정 JSON  | `~/.config/cobot1/firebase/*-firebase-adminsdk-*.json` | 제외     | 같은 폴더에 두면 자동 탐지                               |


`chmod 700 ~/.config/cobot1/firebase && chmod 600 ~/.config/cobot1/firebase/*`
로 권한을 제한한다. 리포지토리 루트 `.gitignore` 에도
`*-firebase-adminsdk-*.json`, `src/cobot1/config/firebase_sync.env`,
`src/cobot1/config/firebase/**` 안전장치가 걸려 있다.

### 3.2 env 스키마

```bash
FIREBASE_DATABASE_URL=https://<project>-default-rtdb.<region>.firebasedatabase.app/
FIREBASE_SERVICE_ACCOUNT_FILE=./<project>-firebase-adminsdk-*.json   # 상대경로면 env 파일과 같은 폴더 기준
FIREBASE_STATUS_PATH=robots/m0609/status     # 기본값과 동일하면 생략 가능
FIREBASE_EVENTS_PATH=robots/m0609/events     # 생략하면 기본값
FIREBASE_SYNC_TIMEOUT_SEC=3                  # REST HTTP 타임아웃
FIREBASE_SYNC_HEARTBEAT_SEC=5                # 동일 시그니처 재전송 간격
# FIREBASE_DATABASE_AUTH=                    # REST fallback 시 DB rule 이 auth 요구할 때만
```

### 3.3 env 파일 탐색 순서

`ui_bridge` 기동 시 `_load_env_file()` 이 아래 순서로 찾아 `os.environ` 에
주입한다. 첫 번째로 존재하는 파일만 사용된다. 이미 설정된 환경변수는
덮어쓰지 않아 shell `export` 가 우선한다.

1. `$FIREBASE_SYNC_ENV_FILE` (override 용 환경변수)
2. `$XDG_CONFIG_HOME/cobot1/firebase/firebase_sync.env`
3. `~/.config/cobot1/firebase/firebase_sync.env`

발견된 env 파일의 폴더가 서비스 계정 JSON 자동 탐지 기준이 된다.
`FIREBASE_SERVICE_ACCOUNT_FILE` 값이 비었거나 찾을 수 없으면 그 폴더의
`*-firebase-adminsdk-*.json` 을 glob 으로 탐색한다.

## 4. `FirebaseRealtimeSync` 내부 구조

`cobot1/ui_bridge.py` 에 정의된 클래스로, 메인 ROS 콜백과 분리된 동기화를
전담한다.

### 4.1 백엔드 선택

`_configure_backend()` 에서 다음 순서로 결정된다:

1. `FIREBASE_DATABASE_URL` 이 비어 있으면 → `mode="disabled"` (이후 호출 무시).
2. `firebase_admin` 모듈이 import 가능하고 서비스 계정 JSON 을 찾으면 →
  `initialize_app()` 후 `mode="admin"`.
3. 위 둘 중 하나라도 실패하면 → `mode="rest"`. `status_endpoint_url`,
  `events_endpoint_url` 을 미리 구성해 둔다.

admin SDK 가 있어도 `initialize_app` 이 실패하면 REST 로 자동 downgrade 되고
로그에 원인이 남는다.

### 4.2 스냅샷 sync (`enqueue_snapshot`)

```
enqueue_snapshot(payload)
    │
    ├─ _make_signature(payload)      # 핵심 필드만 JSON 직렬화
    │
    ├─ if in-flight or
    │   (같은 signature && heartbeat 주기 내) → skip
    │
    └─ threading.Thread(_snapshot_worker, daemon=True)
            │
            ├─ admin 모드 : status_ref.set(payload)
            └─ REST 모드  : urllib PUT <status_url> <json>
```

- `_sync_in_flight` 락으로 네트워크 호출이 겹치지 않게 한다 (0.2초 publish 주기
vs 네트워크 지연 차이 보호).
- 시그니처가 같은 payload 는 `FIREBASE_SYNC_HEARTBEAT_SEC` 간격을 지나야 다시
전송한다. 아무 변화 없을 때도 최소 heartbeat 주기는 갱신돼 살아있음을 표시.
- 시그니처 필드: `system_status / task_status / robot_status / message / mode`.

### 4.3 이벤트 push (`enqueue_event`)

```
enqueue_event(event)
    └─ threading.Thread(_event_worker, daemon=True)
            ├─ admin : events_ref.push(payload)   # Firebase 자동 해시 키
            └─ REST  : urllib POST <events_url>   # {"name":"-NxYz..."} 응답
```

- 중복 억제 없음. 호출자가 "의미 있는 전이" 라고 판단할 때만 호출.
- 실패는 WARN 로그만 남기고 스냅샷 상태에는 영향 주지 않는다. (로그가 너무
도배되지 않게 같은 에러 메시지는 1회만 출력.)

### 4.4 sync 상태 필드

`build_status_fields()` 가 매 publish 마다 호출되어 UI payload 에 아래 필드를
얹는다. 이 스키마는 브라우저 UI (`home_ui.js` `buildFirebaseStatusInfo`) 가
그대로 인식한다.


| 필드                             | 값                                                       | 의미                    |
| ------------------------------ | ------------------------------------------------------- | --------------------- |
| `firebase_sync_enabled`        | bool                                                    | DB URL 이 설정되어 활성 상태인지 |
| `firebase_sync_mode`           | `"admin" / "rest" / "disabled"`                         | 선택된 백엔드               |
| `firebase_sync_path`           | `robots/m0609/status`                                   | 스냅샷 경로                |
| `firebase_sync_events_path`    | `robots/m0609/events`                                   | 이벤트 경로                |
| `firebase_sync_status`         | `"ready" / "syncing" / "synced" / "error" / "disabled"` | 마지막 호출 결과             |
| `firebase_sync_last_synced_at` | ISO8601 UTC                                             | 마지막 성공 시각             |
| `firebase_sync_error`          | string                                                  | 에러 메시지(있을 때)          |


## 5. DB 구조

### 5.1 `robots/m0609/status` (set)

```json
{
  "system_status": {
    "state": 1,
    "name": "WORKING",
    "label": "작업 중"
  },
  "task_status": {
    "state": 1,
    "name": "tong_source",
    "label": "자동 배식",
    "progress": 45
  },
  "robot_status": {
    "state": 2,
    "name": "MOVING",
    "posj": [0.0, -20.5, 90.0, 0.0, 70.0, 0.0],
    "tool_force": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
    "external_joint_torque": [0.5, 0.3, 0.2, 0.1, 0.1, 0.0]
  },
  "message": {
    "text": "돈까스 위로 이동",
    "level": "info"
  },
  "mode": "real",
  "updated_at": "2026-04-20T13:17:32.512831+09:00",
  "firebase_sync_source": "cobot1.ui_bridge"
}
```

- 최신 1개 스냅샷만 유지 (덮어쓰기).
- UI 는 이 노드를 subscribe 하면 실시간 상태 추적 가능.
- `mode`: `"virtual"` (시뮬레이션) 또는 `"real"` (실제 로봇). launch 파라미터로 설정.
- `updated_at`: KST (UTC+09:00) 기준 ISO8601 형식.

### 5.2 `robots/m0609/events` (push)

```json
{
  "id": "20260420_a1b2c3",
  "type": "complete",
  "message": "자동 배식 완료 (3회차)",
  "created_at": "2026-04-20T13:17:32.512831+09:00",
  "mode": "real"
}
```

- 키는 Firebase 가 시간순 정렬 가능한 push-ID 로 자동 생성.
- `id`: 날짜_난수 조합 (예: `20260420_a1b2c3`).
- 기록되는 `type`:
  - `complete` — task 정상 완료 시
  - `error` — task error 또는 로봇 SAFE_OFF/SAFE_STOP 진입 시
- `created_at`: KST (UTC+09:00) 기준 ISO8601 형식.

## 6. 실행 / 1회 셋업

### 6.1 의존성

```bash
sudo apt install python3-pip
pip3 install --user firebase-admin   # 미설치면 REST 모드로 자동 fallback
```

`firebase-admin` 은 rosdep 레지스트리에 없는 순수 pip 패키지라 `package.xml`
의존성으로는 명시하지 않고 주석으로만 안내된다.

### 6.2 인증 파일 배치

```bash
mkdir -p ~/.config/cobot1/firebase
chmod 700 ~/.config/cobot1/firebase

# env 템플릿 복사
cp $(ros2 pkg prefix cobot1)/share/cobot1/config/firebase_sync.env.example \
   ~/.config/cobot1/firebase/firebase_sync.env

# 서비스 계정 JSON 도 같은 폴더에
cp <project>-firebase-adminsdk-*.json ~/.config/cobot1/firebase/
chmod 600 ~/.config/cobot1/firebase/*
```

### 6.3 실행

```bash
source ~/cobot_ws/install/setup.bash
ros2 run cobot1 ui_bridge

# 시뮬레이션 모드로 실행 (mode 파라미터 지정)
ros2 run cobot1 ui_bridge --ros-args -p mode:=virtual
```

정상 로그:

```
[INFO] [ui_bridge]: Loaded firebase env: /home/<user>/.config/cobot1/firebase/firebase_sync.env
[INFO] [ui_bridge]: Mode: real
[INFO] [ui_bridge]: Firebase sync enabled (admin SDK): status=robots/m0609/status, events=robots/m0609/events, cred=...
```

## 7. 검증

```bash
# UI status 토픽에 sync 필드가 채워지는지
ros2 topic echo /m0609/ui/status --once

# DB 내용 직접 덤프 (Console 권한 없이도 가능)
python3 -c "
import firebase_admin, json
from firebase_admin import credentials, db
cred = credentials.Certificate('/home/$(whoami)/.config/cobot1/firebase/rokey-b-1-firebase-adminsdk-fbsvc-605b9577d3.json')
firebase_admin.initialize_app(cred, {'databaseURL': 'https://rokey-b-1-default-rtdb.asia-southeast1.firebasedatabase.app/'})
print(json.dumps(db.reference('robots/m0609').get(), indent=2, ensure_ascii=False))
"
```

Firebase Console 웹:
[https://console.firebase.google.com/project/rokey-b-1/database/rokey-b-1-default-rtdb/data](https://console.firebase.google.com/project/rokey-b-1/database/rokey-b-1-default-rtdb/data)
(접속에는 프로젝트 멤버 Google 계정 권한이 별도로 필요 — 서비스 계정 JSON 과는
별개의 인증.)

## 8. 트러블슈팅


| 증상                                                         | 원인                          | 해결                                                                              |
| ---------------------------------------------------------- | --------------------------- | ------------------------------------------------------------------------------- |
| `Firebase sync disabled (FIREBASE_DATABASE_URL not set).`  | env 파일 못 찾음, 또는 URL 공란      | `~/.config/cobot1/firebase/firebase_sync.env` 존재 및 `FIREBASE_DATABASE_URL` 값 확인 |
| `Running in REST mode (no SDK).` 경고                        | `firebase-admin` 미설치        | `pip3 install --user firebase-admin`                                            |
| `Firebase event push failed: HTTP Error 401: Unauthorized` | REST 모드 + DB rule 이 auth 필수 | admin SDK 로 전환(위 설치) 하거나 `FIREBASE_DATABASE_AUTH` 설정                            |
| `firebase_sync_status: "error"` 지속                         | 네트워크/자격증명/경로 중 하나 문제        | `firebase_sync_error` 필드 값 + WARN 로그 확인                                         |
| Console 에 값이 없는데 `synced` 는 뜸                              | region 또는 path 오탈자          | `FIREBASE_DATABASE_URL` 의 region, `FIREBASE_STATUS_PATH` 대소문자 재확인               |
| Console 접속이 권한 부족                                          | Google 계정이 프로젝트 멤버 아님       | 프로젝트 Owner 에게 Console → 사용자 및 권한 에서 이메일 초대 요청                                   |


## 9. 향후 확장 아이디어

- `FIREBASE_EVENTS_PATH` 를 주석 처리 시 이벤트 push 건너뛰기 옵션 추가
(현재는 경로가 있으면 무조건 시도하고 실패만 로깅).
- heartbeat 외에도 progress 구간(예: 10%마다)별 이벤트 push 모드.
- 다중 로봇 배포 시 `ui_namespace` 를 Firebase 경로에 직접 반영하도록
`FIREBASE_STATUS_PATH` 템플릿(`robots/{ui_ns}/status`) 지원.
- ACL 축소를 위해 서비스 계정에 `Firebase Realtime Database → restricted path` rule 과 대응되는 권한만 부여.

