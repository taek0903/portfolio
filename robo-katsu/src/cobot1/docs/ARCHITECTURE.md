# Cobot1 Task Control System Architecture

## 개요

ROS2 기반 두산 로봇(DSR2) 모션 제어 시스템. 외부 UI(CLI / 웹) 에서 로봇
작업(Task)을 시작/중지/일시정지/재개하고, 로봇의 관절·외력 상태를 실시간으로
모니터링할 수 있는 인터페이스를 제공한다.

현재 구조는 **4 개 프로세스 역할 분리** (+ DSR2 controller, 외부 1 개) 로 구성돼
있으며, 이 형태가 된 배경은 `docs/rclpy-executor-dsr2-troubleshooting.md` 를
참고. 과거에는 `motion_executor` 한 프로세스 안에서 명령·모니터링을 모두
수행했으나, DSR_ROBOT2 Python wrapper 가 내부적으로 임시
`SingleThreadedExecutor.spin_until_future_complete()` 를 사용해 RMW context 를
점유하는 문제로, 모니터링 서비스의 `add_done_callback` 이 firing 하지 못하는
contention 이 발생해 **읽기 전용 모니터링 폴링을 별도 프로세스로 분리**했다.

## 아키텍처

```text
                                        ┌──────────────────────────────────────┐
                                        │  DSR2 controller  (dsr_controller2)  │
                                        │  --------------------------------    │
                                        │  Services (MutuallyExclusive CB):    │
                                        │   /dsr01/system/get_robot_state      │
                                        │   /dsr01/aux_control/get_current_posj│
                                        │   /dsr01/aux_control/get_tool_force  │
                                        │   /dsr01/aux_control/get_external_torque
                                        │   /dsr01/motion/move_joint, move_lin…│
                                        │   … (명령 서비스 다수)                │
                                        └──────────┬──────────────────┬────────┘
                         status services (4)       │                  │  command services
                         polled @ 10 Hz            │                  │  (call_async, block)
                                                   │                  │
          ┌────────────────────────────────────────┴─┐          ┌─────┴──────────────────────────────┐
          │ robot_status_publisher  (process #1)     │          │ motion_executor   (process #2)     │
          │ -----------------------------------------│          │ -----------------------------------│
          │ rclpy node, MultiThreadedExecutor        │          │ 2 노드 같은 프로세스 (분리 spin):    │
          │ ReentrantCallbackGroup                   │          │  ┌──────────────────────────────┐  │
          │ ─ 4 service clients (parallel call_async)│          │  │ control_node (MT executor)   │  │
          │ ─ watchdog + *_valid flags               │          │  │  • /task/execute (start/stop)│  │
          │ ─ aggregates into RobotStatus.msg        │          │  │  • DSR2 명령 client 래퍼     │  │
          │ ─ publishes @ 10 Hz ──────────────────┐  │          │  │  • robot_status sub (fault)  │  │
          └──────────────────────────────────────┼───┘          │  └────────────┬─────────────────┘  │
                                                 │              │               │ spawn task_thread  │
                                                 │              │  ┌────────────▼─────────────────┐  │
                      /dsr01/motion_executor/    │              │  │ dsr_node (DR_init.__dsr__)   │  │
                      robot_status   (topic)   ──┼──────────┐   │  │  • DSR_ROBOT2 wrapper        │  │
                                                 │          │   │  │    movej/movel/… sync calls  │  │
                                                 │          │   │  │  • 임시 SingleThreadedExec 로│  │
                                                 │          │   │  │    spin_until_future_complete│  │
                                                 │          └──►│  └──────────────────────────────┘  │
                                                 │              │   (fault 감지 시 stop_requested)    │
                                                 │              │   publishes:                       │
                                                 │              │     /dsr01/task/execute/state     ─┼──┐
                                                 │              └────────────────────────────────────┘  │
                                                 │                                                      │
                                                 │          ┌──── task/state ──────────────────────────┘
                                                 │          │  (TaskState.msg: state / task_name /
                                                 │          │   progress / message /
                                                 │          │   current_module_{name,label,index,total})
                                                                         ▼              ▼
                                            ┌───────────────────────────────────────────┐
                                            │ task_controller   (process #3)            │
                                            │ ------------------------------------------│
                                            │ subscribes :                              │
                                            │   /dsr01/motion_executor/robot_status     │
                                            │   /dsr01/task/execute/state               │
                                            │ exposes services to UI :                  │
                                            │   /dsr01/task/start  (StartTask)          │
                                            │   /dsr01/task/stop   (StopTask)           │
                                            │ republishes :                             │
                                            │   /dsr01/task/state (TaskState, for UI)   │
                                            └──────────────┬────────────────────────────┘
                                                           │
                                                           │ /dsr01/task/state
                                                           │ /dsr01/task/start  (srv)
                                                           │ /dsr01/task/stop   (srv)
                                                           │
                       ┌───────────────────────────────────┼─────────────────────────────────────────┐
                       │                                   │                                         │
                                  ▼                                                    ▼                                                             ▼
           ┌───────────────────────┐        ┌──────────────────────────────────┐      ┌─────────────────────────┐
           │  task_cli (터미널)    │        │ ui_bridge   (process #4)         │      │  web_ui (브라우저)      │
           │  --------------------│        │ -------------------------------- │      │  --------------------   │
           │ `task_cli status` :  │        │ subscribes :                     │      │ (rosbridge 없이 현재는  │
           │   1회 snapshot 구독  │        │   /dsr01/task/state              │      │  Firebase RTDB 경유)    │
           │   • task/state       │        │   /dsr01/motion_executor/        │      │                         │
           │   • motion_executor/ │        │       robot_status  ← 신규        │      │ ┌─────────────────────┐ │
           │       robot_status   │        │ subscribes (UI 입력) :           │      │ │ home_ui.js / app.js │ │
           │ `task_cli start/stop`│        │   /m0609/serving_cmd             │      │ │ 기대 payload :      │ │
           │  → StartTask/StopTask│        │   /m0609/pause_cmd               │      │ │  • robot_status     │ │
           └──────────┬───────────┘        │   /m0609/safety_cmd              │      │ │  • op_state         │ │
                      │                    │ calls services :                 │      │ │  • progress         │ │
                      │  StartTask/StopTask│   /dsr01/task/start              │      │ │  • active_action    │ │
                      └───────────────────►│   /dsr01/task/stop               │      │ │  • today_errors     │ │
                                           │ publishes :                      │      │ │  • robot.{          │ │
                                           │   /m0609/ui/status  (JSON String)│──┬──►│ │     state_enum,     │ │
                                           │ Firebase RTDB sync :             │  │   │ │     state_name,     │ │
                                           │   robots/<ui_ns>/status (set)    │  │   │ │     posj,           │ │
                                           │   robots/<ui_ns>/events (push)   │  │   │ │     tool_force,     │ │
                                           └───────────────┬──────────────────┘  │   │ │     ext_joint_tq }  │ │
                                                           │                     │   │ └─────────────────────┘ │
                                                           ▼                     │   └──────────┬──────────────┘
                                                 ┌────────────────────┐          │              │
                                                 │ Firebase Realtime  │          └── /m0609/ui/status
                                                 │ Database           │◄────────────────────────┘
                                                 │  (snapshots +      │                (ROS topic, 로컬 환경)
                                                 │   event stream)    │
                                                 └────────────────────┘
```

## 데이터 흐름

**상향 (로봇 → UI)**

1. `DSR2 controller` ─(4 services @ 10 Hz)→ `robot_status_publisher`
2. `robot_status_publisher` ─(`robot_status` 토픽)→ `task_controller` + `ui_bridge` + `task_cli` + `**motion_executor`** (fault 감지용)
3. `motion_executor` (내부 task_thread) ─(`task/execute/state` 토픽)→ `task_controller`
4. `task_controller` ─(`task/state` 토픽)→ `ui_bridge` + `task_cli`
5. `ui_bridge` ─(`/m0609/ui/status` JSON 문자열)→ web_ui / Firebase RTDB

> `motion_executor` 의 `robot_status` 구독은 **읽기 전용**이다. DSR2 polling
> 책임은 여전히 `robot_status_publisher` 단일 포인트이며, motion_executor 는
> 이 토픽을 소비만 한다. 목적은 SAFE_STOP/SAFE_OFF/EMERGENCY 등 하드웨어
> fault 가 발생했을 때 task thread 에 stop 을 전파해 `_gripper()` 반복 실행을
> 차단하는 것이다. 상세: `docs/recovery-gripper-emergency-subtask.md` §6.

**하향 (UI → 로봇)**

1. `web_ui` ─(Firebase 명령 또는 `/m0609/*_cmd` 토픽)→ `ui_bridge`
2. `ui_bridge` ─(`StartTask` / `StopTask` 서비스)→ `task_controller`
3. `task_controller` ─(`/task/execute` 서비스)→ `motion_executor`
4. `motion_executor.task_thread` ─(DSR_ROBOT2 wrapper → 명령 서비스 호출)→ DSR2 controller

## 프로세스 분리 근거

```
┌──────────────────────────────────────────────────────────────────┐
│  왜 robot_status_publisher 가 motion_executor 와 분리되어 있는가 ?│
│                                                                  │
│  motion_executor 한 프로세스 안에 :                                │
│    • MT Executor   (control_node, 10+ 스레드)                    │
│    • DSR_ROBOT2 wrapper 가 쓰는 임시 ST Executor                   │
│    • task_thread   (동기 명령 블록)                              │
│  가 같은 RMW context 를 공유하면 상태 폴링 future 의                 │
│  add_done_callback 이 firing 을 못 해 `ok=0 fail=N` 루프에 빠짐. │
│                                                                  │
│  → 읽기 전용 monitoring 폴링만 별도 프로세스로 분리하면 :        │
│     • RMW 경쟁 제거, 10 Hz 안정                                  │
│     • 명령 실행과 모니터링 장애 격리                             │
│     • 디바이스 분할 배치 옵션 (디바이스1: executor+status,       │
│       디바이스2: controller/UI) 자연스럽게 열림                  │
└──────────────────────────────────────────────────────────────────┘
```

## 노드 구성

### 1. `motion_executor` (`cobot1/nodes/motion_executor.py`)

- **역할**: 실제 로봇 모션 실행 + 외부 명령 진입점
- **내부 구조**: 한 프로세스 안에 두 노드를 **별도 executor 로 분리 spin**
  - `control_node` : `MultiThreadedExecutor` — `/task/execute` 서비스, 상태 publisher, `robot_status` 구독
  - `dsr_node`     : DSR_ROBOT2 wrapper 가 `DR_init.__dsr__node` 로 쓰는 노드
- **Task 실행**: `start_task` 요청 시 별도 `threading.Thread` 에서 task 함수를 호출. DSR_ROBOT2 의 동기 motion 호출은 wrapper 내부에서 `spin_until_future_complete` 로 블록되므로 메인 executor 와 섞지 않는다.
- **Task 합성**: `TASK_REGISTRY` 에 정의된 대로 여러 task 모듈 (`task_gripper_open`/`task_home`/`task_rice`/`task_tong`/`task_sauce`) 을 순차 실행한다. 자세한 내용은 아래 "Task 합성" 섹션 참조.
- **DSR2 폴링 책임은 없음** — DSR2 monitoring 서비스 호출은 `robot_status_publisher` (process #1) 가 단독 수행.
- `**robot_status` 구독 (읽기 전용)**: `motion_executor/robot_status` 토픽을 구독해 `ROBOT_FAULT_STATES ∈ {EMERGENCY_STOP, SAFE_OFF, SAFE_STOP, SAFE_OFF2, SAFE_STOP2}` 진입을 감지하면 `_shared.stop_requested = True` + `state = ERROR` 세팅. task thread 는 다음 `ctx.check_stop()` 에서 즉시 종료되어 잔여 STEPS (특히 `_gripper()`) 가 실행되지 않는다. `move_stop` 은 호출하지 않음 (DSR 이 이미 하드웨어 레벨에서 모션 차단).

### 2. `robot_status_publisher` (`cobot1/nodes/robot_status_publisher.py`)

- **역할**: DSR2 monitoring 서비스를 **병렬 폴링**해 하나의 RobotStatus 메시지로 합성·발행
- **구조**: 단독 프로세스, `MultiThreadedExecutor` + `ReentrantCallbackGroup`
- **폴링 대상** (10 Hz) :
  - `/dsr01/system/get_robot_state`
  - `/dsr01/aux_control/get_current_posj`
  - `/dsr01/aux_control/get_tool_force`
  - `/dsr01/aux_control/get_external_torque`
- **안정성**: 슬롯별 inflight latch + watchdog. 한 서비스가 늦어도 다른 3 개 업데이트는 계속 발행.
- **토픽**: `/dsr01/motion_executor/robot_status` (`cobot_interfaces/RobotStatus.msg`). 각 필드에 `*_valid` 플래그를 두어 stale 상태를 구분.

### 3. `task_controller` (`cobot1/nodes/task_controller.py`)

- **역할**: 상태 관리 + 외부 UI ↔ motion_executor 중계
- **주요 인터페이스**:
  - 외부 요청 : `/dsr01/task/start` (StartTask), `/dsr01/task/stop` (StopTask)
  - 내부 위임 : `/dsr01/task/execute` (motion_executor 호출)
  - 상태 캐시 : `motion_executor/robot_status` · `task/execute/state` 구독 → 통합 상태 머신
  - 상태 발행 : `/dsr01/task/state` (UI 소비용)

### 4. `ui_bridge` (`cobot1/nodes/ui_bridge.py`)

- **역할**: 웹 UI ↔ ROS 브릿지 + Firebase Realtime Database 동기화
- **구독**:
  - `/dsr01/task/state` — task 진행 상황
  - `/dsr01/motion_executor/robot_status` — posj / tool_force / ext_joint_torque / robot_state enum
  - `/m0609/serving_cmd` · `/m0609/pause_cmd` · `/m0609/safety_cmd` — 웹 UI 입력
- **발행**: `/m0609/ui/status` (JSON String) — 웹 UI 가 소비하는 통합 상태
- **Firebase (양방향)**:
  - 스냅샷 `robots/<ui_ns>/status` 에 `set()` (signature 기반 중복 억제 + heartbeat)
  - 의미 있는 전이 시 `robots/<ui_ns>/events` 에 `push()` (`complete`, `error`,
  `safe_stop` 등)
  - **기동 시 pull**: `FirebaseRealtimeSync.fetch_today_events(since=KST 0시, mode_filter=self._mode)` 로 **현재 실행 모드(virtual/real)** 의 당일 에러 이벤트만
  조회해 `today_errors` 로 bootstrap. 가상 에뮬레이터와 실기 세션이 같은 Firebase 를
  공유해도 서로의 에러가 섞이지 않는다.
  - **자정(KST) 롤오버**: `_publish_status` 타이머에서 KST 날짜 변경을 감지하면
  `today_errors` 버퍼 리셋. 새 이벤트 pull 은 하지 않고 비운 상태에서 누적.
- **payload 스키마 (발췌, 2026-04-23)**:
  ```json
  {
    "system_status": { "state": 1, "name": "WORKING", "label": "작업 중" },
    "task_status": {
      "state": 1,
      "name": "auto_serving",
      "label": "자동 배식",
      "progress": 45,
      "modules": [
        { "name": "gripper_open", "label": "그리퍼 해제", "icon": "🖐" },
        { "name": "rice",         "label": "밥",          "icon": "🍚" },
        { "name": "tong",         "label": "샐러드·돈까스",   "icon": "🍱" },
        { "name": "sauce",        "label": "소스",        "icon": "🍯" }
      ],
      "module_name": "tong",
      "module_label": "샐러드·돈까스",
      "module_index": 2,
      "module_total": 4,
      "step_name": "pork1_down",
      "current_step": 23,
      "total_steps": 48
    },
    "robot_status": {
      "state": 2,
      "name": "MOVING",
      "posj": [0.00, 0.00, 90.00, 0.00, 90.00, 0.00],
      "tool_force": [0.012, -0.003, 0.008, 0.000, 0.001, -0.002],
      "external_joint_torque": [0.001, -0.003, 0.002, 0.000, 0.001, 0.000]
    },
    "message": { "text": "...", "level": "info" },
    "today_errors": [
      {
        "id": "20260423_a1b2c3",
        "message": "Task 오류: ...",
        "created_at": "2026-04-23T08:12:34+09:00",
        "task_name": "tong",
        "step_name": "pork1_grip"
      }
    ],
    "mode": "real",
    "updated_at": "2026-04-23T12:34:56+09:00"
  }
  ```

### 5. `task_cli` (`cobot1/nodes/task_cli.py`)

- **역할**: 터미널용 명령줄 클라이언트 (one-shot)
- **명령어**: `start <task>`, `stop`, `pause`, `resume`, `status`
- `status` 는 `task/state` + `motion_executor/robot_status` 각 1회 스냅샷을 받아 표시.

## 상태 머신

### Task State (커스텀)

정의: `cobot_interfaces/msg/TaskState.msg`

```
# 상태 enum
IDLE = 0, RUNNING = 1, STOPPING = 2, ERROR = 3, PAUSED = 4

# 필드
uint8 state
string task_name / task_id
float32 progress                # 0.0 ~ 1.0
uint32 current_step / total_steps
string current_step_name
float64[6] current_posj / current_posx
string message

# 서브태스크(합성 task) 진행 상황 (2026-04-22 추가)
string current_module_name      # TASK_NAME 식별자: "rice"/"tong"/"sauce"/"gripper_open"/"home"
string current_module_label     # UI 표시용 라벨: "밥"/"샐러드·돈까스"/"소스"/...
uint32 module_index             # 0-base
uint32 module_total             # 합성된 모듈 수 (auto_serving = 4)
```

전이:

```
         start
  IDLE ────────► RUNNING
   ▲               │ │
   │      stop     │ │ pause
   │◄──────────────┘ │
   │                 ▼
   │    stop      PAUSED
   │◄────────────── │
   │     resume    │
   │◄──────────────┘
   │
   └── STOPPING ──► IDLE
   └── ERROR ─────► IDLE
```

### Robot State (DSR2 enum)

`cobot_interfaces/msg/RobotStatus.msg` 에 복제 정의 (`dsr_msgs2/srv/GetRobotState.srv` 와 일치).

```
INITIALIZING=0, STANDBY=1, MOVING=2, SAFE_OFF=3, TEACHING=4,
SAFE_STOP=5, EMERGENCY_STOP=6, HOMMING=7, RECOVERY=8,
SAFE_STOP2=9, SAFE_OFF2=10, NOT_READY=15, UNKNOWN=255
```

> ⚠️ DRCF 가상 에뮬레이터 한계: 조인트는 움직여도 `robot_state` 가
> `STANDBY(1)` 에서 전이되지 않는 현상이 관찰된다. 실기에서는 `MOVING(2)` 로
> 정상 전이 예상. 자세한 검증 로그는 트러블슈팅 문서 참고.

### SAFE_STOP 복구 경로

외력/충돌로 DSR 가 `SAFE_STOP(5)` / `SAFE_STOP2(9)` / `SAFE_OFF(3)` /
`SAFE_OFF2(10)` / `EMERGENCY_STOP(6)` 로 전이되면 자체 프로그램 정지가 걸려
일반 `move_resume` / `move_stop` 으로는 해제되지 않는다. UI 에서 2단계
복구 UI 를 통해 아래 시퀀스가 수행된다.

```
Phase 1: 복구 방법 선택 (자동 / 수동)
────────────────────────────────────
web UI
  │  자동 복구: StopTask { stop_type = 6 (TO_STANDBY) }  — SAFE_STOP/SAFE_OFF
  │  수동 복구: StopTask { stop_type = 7 (TO_RECOVERY) } — SAFE_STOP2/SAFE_OFF2
  ▼                                                        (freedrive 후 RECOVERY)
task_controller_node ── proxy ──► motion_executor
                                    │
                                    │  TO_STANDBY:
                                    │    set_safe_stop_reset_type(PROGRAM_STOP)
                                    │    set_robot_mode(AUTONOMOUS)
                                    │    → state 가 STANDBY 로 복귀
                                    │
                                    │  TO_RECOVERY:
                                    │    set_robot_control(RECOVERY_MODE_ON)
                                    │    → 작업자가 수동으로 로봇 이동(freedrive)

Phase 2: 복구 완료 후 동작 선택
────────────────────────────────────
(수동 복구인 경우) UI: StopTask { stop_type = 8 (RECOVERY_DONE) }
  → set_robot_control(RECOVERY_MODE_OFF) → STANDBY 진입

UI: StartTask { task_name }  ← 사용자가 "🏠 홈으로" or "🔁 작업 재시작" 선택
        │
        │  "🏠 홈으로"  → recovery_home           (= gripper_open + home)
        │  "🔁 재시작"  → resolveResumeTask(root, module):
        │      root=auto_serving & module∈{rice,tong,sauce}
        │          → resume_from_<module>        (중단 모듈 ~ sauce 까지 연속)
        │      그 외 (단일 task 중단)
        │          → recovery_<root|module>       (동일 단일 모듈 재실행)
        ▼
motion_executor 가 TASK_REGISTRY 를 통해 합성된 모듈 순차 실행.
첫 모듈은 항상 task_gripper_open → 쥐고 있던 payload 를 먼저 해제한 뒤 홈/작업 이동.
```

- DSR2 서비스 스펙:
  - `dsr_msgs2/srv/SetSafeStopResetType.reset_type` (0=`PROGRAM_STOP`, 1=`PROGRAM_RESUME`)
  - `dsr_msgs2/srv/SetRobotMode.robot_mode`         (0=`MANUAL`, 1=`AUTONOMOUS`, 2=`MEASURE`)
  - `dsr_msgs2/srv/SetRobotControl.robot_control`   (recovery mode on/off 외 다수)
- **UI 재시작의 정교화**: `interruptedTaskRef` 는 SAFE_STOP 진입 시점의
`{ root: task_name, module: module_name }` 페어를 캡처. auto_serving 연속 배식
중단인지, 단일 task 중단인지를 둘 다 구분할 수 있다.
  - auto_serving 중 tong 에서 멈추면 `{root:"auto_serving", module:"tong"}` →
  `resume_from_tong` 호출 → tong → sauce 연속 실행 (rice 는 반복하지 않음).
  - `tong` 단독 실행 중 멈추면 `{root:"tong", module:"tong"}` → `recovery_tong`
  호출 → tong 단일 모듈만 재실행.
  - Phase1 패널에 "중단된 작업 · 🍱 샐러드·돈까스 (자동 배식 중)" 뱃지, Phase2
  재시작 버튼 하단에 "재시작 시: 🍱 집게부터 이어서 (샐러드·돈까스 → 소스)"
  문구를 표시해 사용자가 누르기 전에 범위를 인지할 수 있게 한다.
- `stop_type ∈ {6, 7, 8}` 와 `stop_type = 2 (EMERGENCY)` 는 `task_controller`
에서 `task_state` 제약 없이 통과하도록 별도 분기로 처리된다.
- `stop_type = 5 (SAFE_STOP_RECOVER)` 는 레거시. 내부적으로 `TO_STANDBY(6)`
와 동일한 경로를 탄다. 신규 UI 는 6/7/8 만 사용한다.
- 상세 플로우: `docs/robot-state-recovery.md`,
`docs/recovery-gripper-emergency-subtask.md`.

### 긴급정지 (Emergency Stop)

`stop_type = 2 (STOP_EMERGENCY)` 는 **task_state 와 무관하게 항상 허용**된다
(IDLE/STANDBY 포함). 경로:

```
web UI ─ /m0609/safety_cmd = "EMERGENCY_STOP" ──► ui_bridge
ui_bridge ─ StopTask { stop_type = 2 } (무조건 forward) ──► task_controller
task_controller ─ /task/execute (stop) ──► motion_executor
motion_executor:
    move_stop(QUICK)
    set_robot_control(SERVO_OFF)
    _shared.state = IDLE, task_name = ""
```

2026-04-22 이전에는 `task_state ∈ {RUNNING, PAUSED}` 일 때만 EMERGENCY 를
forward 했기 때문에 STANDBY 상태에서 비상 버튼을 눌러도 servo_off 가 되지
않던 버그가 있었다. 두 게이팅(`ui_bridge`, `task_controller`) 모두 제거됨.

## 핵심 구현 사항

### 0. Task 합성 (`TASK_REGISTRY`)

`motion_executor.TASK_REGISTRY` 는 task 이름 → 모듈 리스트 매핑. 한 task 는
여러 모듈을 연속 실행하며, 각 모듈의 STEPS 가 합산되어 `total_steps` /
`current_step` 이 자동 계산된다.

```python
TASK_REGISTRY = {
    # 단일 진입
    "gripper_open":          [task_gripper_open],
    "home":                  [task_gripper_open, task_home],
    "rice":                  [task_gripper_open, task_rice],
    "tong":                  [task_gripper_open, task_tong],
    "sauce":                 [task_gripper_open, task_sauce],

    # 연속 배식
    "auto_serving":          [task_gripper_open, task_rice, task_tong, task_sauce],

    # 복구 플로우 (UI 2-Phase Recovery 가 선택) — 단일 모듈 재실행
    "recovery_home":         [task_gripper_open, task_home],
    "recovery_rice":         [task_gripper_open, task_rice],
    "recovery_tong":         [task_gripper_open, task_tong],
    "recovery_sauce":        [task_gripper_open, task_sauce],
    "recovery_auto_serving": [task_gripper_open, task_rice, task_tong, task_sauce],

    # auto_serving 중 SAFE_STOP 해제 후 "재시작" — 중단 모듈부터 sauce 까지 연속.
    # 예) auto_serving 중 tong 에서 멈춤 → resume_from_tong 선택 → tong → sauce.
    "resume_from_rice":      [task_gripper_open, task_rice, task_tong, task_sauce],
    "resume_from_tong":      [task_gripper_open, task_tong, task_sauce],
    "resume_from_sauce":     [task_gripper_open, task_sauce],
}
```

`**task_gripper_open` 의 의의**: 모든 진입(home/배식/복구) 의 최선행 모듈.
`task_rice._gripper("RELEASE", ctx)` 에 위임해 하드웨어 DO1=0/DO2=1/DO3=0 으로
그리퍼를 연다. JReady 로의 첫 amovej 이전에 반드시 payload 를 해제하므로, 쥔
상태로 홈 이동하다가 충돌·낙하할 위험을 차단한다.

`**recovery_`* vs 일반 task**: 현재 구성은 동일하지만 키를 분리해 두었다.
향후 복구 전용 점검 스텝 (툴 체크, 좌표 검증 등) 은 `recovery_`* 에만 추가할
수 있다.

`**resume_from_`* vs `recovery_`***: `resume_from_`* 는 **auto_serving 연속
배식이 중간에 끊긴 경우 전용**. 중단된 모듈부터 sauce 까지 남은 모듈을 한 번에
이어서 실행한다. `recovery`_* 는 단일 task (rice/tong/sauce/home) 가 중단되었을
때 동일 모듈을 재실행하는 용도. app.js 의 `resolveResumeTask(root, module)` 이
`root=auto_serving` 여부로 둘을 구분해 선택한다.

`resume_from_` 실행 시 progress scale 보정: `motion_executor._auto_serving_parent_skip_steps(task_name)` 이 건너뛴 auto_serving 모듈의 STEPS 합을 반환하고,
`_on_start` 는 이를 `total_steps` 에 더하고 `_run_task_sequence` 는 초기
`step_offset` 으로 사용한다. 이렇게 하면 퍼블리시되는 `progress` 가 resume task
기준이 아니라 **전체 auto_serving 기준 %** 로 보이고, UI 게이지가 중단 지점
% 에서 이어서 올라간다. `recovery_auto_serving` / 일반 task 는 offset 0 이므로
기존 동작 유지.

### 0-1. 그리퍼 Holding Failsafe

SAFE_STOP 등 fault 진입 시 task 의 `_gripper()` 함수는 **현재 DO 상태를 그대로
유지**한다 (옛날 동작: 안전상 강제 RELEASE → 2026-04-22 제거).

- 산업 표준 fail-hold 패턴. 쥐고 있던 payload 낙하 방지.
- 명시적 해제는 복구 후 사용자가 UI 복구 버튼을 눌렀을 때 `recovery`_* 의
첫 모듈인 `task_gripper_open` 이 담당한다.
- `motion_executor` 의 `robot_status` 구독이 fault 를 감지해 `stop_requested`
를 세팅 → task thread 의 `ctx.check_stop()` 지점에서 즉시 exit → `_gripper()`
가 추가 호출되지 않음.

상세: `docs/recovery-gripper-emergency-subtask.md`.

### 0-2. 서브태스크 UI 표시

`auto_serving` 같은 합성 task 가 현재 어느 모듈을 실행 중인지 UI 에 노출한다.

전파 경로:

```
motion_executor._run_task_sequence
  └─ _SharedState.module_name / module_label / module_index / module_total
     → TaskState (execute/state)
task_controller._executor_state_callback → _publish_state
  └─ TaskState (task/state) — module_* 필드 forward
ui_bridge._task_state_callback → _publish_status
  └─ task_status.module_name/label/index/total + step_name/current_step/total_steps
app.js
  └─ CircularGauge 중앙에 `{icon} {label}` 표시 + tong 내부 phase badge
     (돈까스 1/2 는 "돈까스" 로 통합, getTongPhase() 가 step_name 접두어 분석)
```


| module_name    | TASK_LABEL | UI icon |
| -------------- | ---------- | ------- |
| `gripper_open` | 그리퍼 해제     | 🖐      |
| `home`         | 홈 복귀       | 🏠      |
| `rice`         | 밥          | 🍚      |
| `tong`         | 샐러드·돈까스    | 🍱      |
| `sauce`        | 소스         | 🍯      |


### 0-3. 당일(KST) 카운트 · 에러 로그 pull (2026-04-23)

웹 UI 새로고침 후에도 "당일 배식 횟수" / "오늘의 에러" 가 유지되도록 `ui_bridge`
가 기동 시 Firebase `events` 경로에서 당일(KST 0시 이후) 이벤트를 pull 한다.

```
ui_bridge 기동
  └─ FirebaseRealtimeSync.fetch_today_events(since=KST 0시)
        │  admin 모드 : db.reference(events_path).order_by_key()
        │               .limit_to_last(500).get()
        │  REST 모드  : GET {events_path}.json
        │               ?orderBy="$key"&limitToLast=500
        ▼
     iter events → filter (type=="error" & created_at ≥ boundary)
        └─ self._today_errors.append(...)  (링버퍼 최대 50건)

_publish_status (5 Hz)
  └─ 현재 KST 날짜가 바뀌었으면 today_errors 리셋 (자정 롤오버)
  └─ payload.today_errors 에 노출
```

`complete` 이벤트는 Firebase 에 여전히 enqueue 되지만 ui_bridge 는 복원 시
카운트에 쓰지 않는다 (배식 카운트 feature 는 제거됨).

### 0-4. UI 구성 (2026-04-23)

`web_ui/app.js` 의 메인 레이아웃은 상단 sticky 헤더 + 좌/우 2 컬럼 (`xl:grid-cols-[1.6fr_1fr]`).

- **헤더 (sticky, `top-0 z-40`)**: 연결 dot + `배식 로봇 대시보드` 타이틀 한 줄 +
  (`IDLE + connected` 일 때만) `홈으로` 버튼 + `긴급 정지` / `긴급 정지 해제` 버튼.
  페이지 스크롤 시에도 고정.
  - `홈으로` 는 헤더 우측에서 Emergency 왼쪽. `recovery_home` (그리퍼 해제 후 JReady)
  을 호출. PAUSED/EMERGENCY/SAFE_STOP/RECOVERY 상태에서는 숨겨짐 — 각 전용 컨트롤
  (재개/복구 패널/토스트) 이 대신 노출된다.
  - 기존 `ROBOT CONTROL DASHBOARD` / `System Status` / `Bridge` / `Current Task` /
  `Serving` 카운트는 모두 제거.
- **좌**: CircularGauge (progress + 현재 모듈 아이콘) + Primary Control Panel
(작업 시작 / 일시정지 / 재개 / 복구 패널).
  - Standby 아이콘은 `⋯` (animate-pulse), 복구 완료 아이콘은 🛠 (툴 모양).
  - Primary/Recovery 버튼은 아이콘 + 라벨이 **수평 배치** (`flex-row items-center justify-center gap-3`), 높이 `py-5~py-4` 로 컴팩트.
  - **복구 플로우 UI**:
    - Phase1 (SAFE_STOP/EMERGENCY): `복구 시작하기` (primary blue, 자동 복구),
    2계열이면 `수동 복구` (amber). 상단 안내 라벨은 토스트가 담당해 제거됨.
    `중단된 작업 · 🍱 …` 뱃지만 유지.
    - Manual (RECOVERY): `이동 완료` 버튼만.
    - Phase2 (RECOVERED): 2열 버튼. 좌측 **`마저 진행하기`** (primary emerald,
    `resume_from_<module>` / `recovery_<task>` 호출), 우측 `홈으로` (subtle white,
    헤더 홈 버튼과 동일 톤). 취소 버튼은 제거됨.
    - **Progress freeze**: `SAFE_STOP*` / `EMERGENCY` / `RECOVERY` / `RECOVERED`
    동안 게이지 % 가 freeze 되어 사용자가 중단 지점을 계속 확인할 수 있다.
    다음 task 가 `RUNNING` 으로 전환되면 정상 갱신 재개.
- **우**: Task Steps 패널.
  - 데이터 소스: `task_status.modules` (= `ui_bridge.TASK_MODULES[task_name]`).
  - 각 행: `{marker(✓/▶/○)} {icon} {label} {진행 중|완료}`.
  - `module_index` 기반으로 진행/완료/남음 구분. 완료 행은 strikethrough + dim,
  진행 행은 파란색 + bold, 남음 행은 회색.
  - **resume_from_* 실행 중에는** 앞쪽에 이미 완료된 이전 모듈을 virtualDone 으로
  prepend 해서 `[gripper_open(✓ 완료), rice(✓ 완료), tong(▶ 진행 중), sauce(○)]`
  처럼 auto_serving 전체 흐름이 보인다 (프론트 `SKIPPED_BEFORE` 상수). 이전 모듈은
  SAFE_STOP 이전에 실제로 완료된 것이므로 "건너뜀" 이 아닌 "완료" 로 표기.
  motion_executor 실제 실행에는 영향 없음 (표시만 확장).
  - 작업 없을 때 (task_name="") placeholder "작업이 시작되면 단계가 여기에
  표시됩니다." 노출.
- **상태 안내 Toast (`fixed top-[72px] left-1/2 -translate-x-1/2 z-30 max-w-4xl`)**:
비정상 상태 (`DISCONNECTED` / `EMERGENCY` / `SAFE_STOP`* / `RECOVERY` / `RECOVERED`)
일 때만 헤더 아래 중앙에 고정 노출, 상태 해제 시 자동 소멸. non-blocking
(pointer-events 는 카드 영역만 활성화).
  - 구성: `[icon] [EYEBROW 태그 / description(설명+가이드 합친 한 문장)] [✕]`.
  - 우측 X 버튼으로 토스트를 닫을 수 있다. 닫힘은 현재 `(systemState, recoveryPhase,
  connected)` 조합 동안만 유지되며, 조합이 바뀌면 자동 재노출 (`dismissedToastKey`
  state).
  - 텍스트 수정 위치는 `app.js` 의 `statusGuide = (() => { ... })()` 객체의 각 분기
  (`eyebrow` / `description`).
- **하단**: Activity Logs (세션 in-memory) + **오늘의 에러** 패널
(`payload.today_errors` 렌더, 새로고침해도 Firebase bootstrap 으로 유지) +
**상세 정보 (raw)** 패널 — `system_status` / `task_status` / `robot_status`
세 컬럼에 payload 를 key/value rows 로 그대로 덤프 (`formatRawValue` 로
중첩 값은 한 줄 JSON 요약). 디버깅용.

### 1. 비동기 모션 사용 (Async Motion)

DSR2 의 `movel()`, `movej()` 는 **블로킹 함수** (완료까지 반환 ×, 내부적으로
`rclpy.spin_until_future_complete()` 사용, 중간 pause/stop 불가). 대신
`amovel()`, `amovej()` 를 사용해 즉시 반환 → 루프에서 `check_motion` 으로
완료 확인.

```python
amovel(pos)              # 즉시 반환
while True:
    if self._check_stop():
        return False
    if self._get_motion_status() == 0:   # 0: idle(완료), 1: paused, 2: moving
        return True
    time.sleep(0.05)     # 50ms 폴링
```

### 2. Task 전용 스레드

DSR_ROBOT2 wrapper 의 동기 호출이 메인 executor 를 점유하지 않도록 task 는
별도 스레드에서 실행한다.

```python
self._task_thread = threading.Thread(
    target=self._execute_task, args=(task_name,), daemon=True,
)
self._task_thread.start()
```

### 3. 병렬 상태 폴링

`robot_status_publisher` 는 4 개 서비스를 **동시에** `call_async` 로 쏘고
각 future 의 `add_done_callback` 에서 캐시를 갱신한다. 가장 느린 서비스에
병목되지 않도록 슬롯별로 완전히 독립적.

### 4. Python 래퍼가 없는 DSR2 서비스는 직접 호출

`move_pause`, `move_resume`, `move_stop`, `check_motion` 등은 DSR_ROBOT2.py 에
래퍼가 없어 `create_client` 로 직접 호출한다.

## 주의사항

### DSR2 Python API 제약

1. `movel`/`movej` 는 블로킹 — 반드시 `amovel`/`amovej` 사용
2. DSR2 API 내부 `spin_until_future_complete` 때문에 executor spin 과 충돌하므로 **모니터링 폴링은 별도 프로세스** (`robot_status_publisher`) 에서만 수행
3. 래퍼 누락 서비스는 ROS2 client 로 직접 호출

### 워치독

- `task_controller` : stop 요청 후 3s 내 STOPPING → IDLE 강제 전이
- `robot_status_publisher` : 슬롯별 inflight 요청이 N s 이상 응답 없으면 타임아웃 마킹 + 다음 주기 재시도

### Conda 환경 빌드

`cobot_interfaces` 빌드 시 Conda Python 과 충돌 가능.

```bash
conda deactivate
source /opt/ros/humble/setup.bash
colcon build --packages-select cobot_interfaces
```

## 사용법

### 시뮬레이터 실행

```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual
```

### Task 시스템 실행

```bash
# 전부 한 머신 (개발) — motion_executor + status + task_controller
ros2 launch cobot1 task_system.launch.py
# + UI 스택 (ui_bridge + rosbridge)
ros2 launch cobot1 ui.launch.py

# 디바이스 분할 (운영) — 각 디바이스별 전용 launch
# 디바이스1 (로봇 근접): motion_executor + robot_status_publisher
ros2 launch cobot1 motion.launch.py
# 디바이스2 (UI/원격):   task_controller + ui_bridge + rosbridge 동시 기동
ros2 launch cobot1 ui.launch.py mode:=real
# 가상 에뮬레이터 세션
ros2 launch cobot1 ui.launch.py mode:=virtual
```

launch arg 매트릭스:

`**motion.launch.py**` (디바이스1 · 로봇 근접)


| arg                       | 기본      | 역할                                             |
| ------------------------- | ------- | ---------------------------------------------- |
| `robot_namespace`         | `dsr01` | motion_executor / robot_status_publisher namespace |
| `poll_posj:=true/false`   | `false` | `robot_status_publisher` 의 joint position 폴링 |
| `poll_tool_force:=...`    | `false` | tool force 폴링                                  |
| `poll_ext_torque:=...`    | `false` | external torque 폴링                             |


`**task_system.launch.py**` (단일 머신 개발용 — 3노드 통합)


| arg                      | 기본     | 노드                       |
| ------------------------ | ------ | ------------------------ |
| `executor:=true/false`   | `true` | `motion_executor`        |
| `status:=true/false`     | `true` | `robot_status_publisher` |
| `controller:=true/false` | `true` | `task_controller`        |


`**ui.launch.py**` (디바이스2 · UI/원격)


| arg               | 기본      | 역할                                             |
| ----------------- | ------- | ---------------------------------------------- |
| `robot_namespace` | `dsr01` | `task_controller` namespace (device1 과 동일해야 함) |
| `mode`            | `real`  | `ui_bridge` mode 파라미터 (`real` / `virtual`)     |
| `rosbridge_port`  | `9090`  | rosbridge_websocket 리슨 포트                      |


### UI Bridge 단독 실행

```bash
ros2 run cobot1 ui_bridge --ros-args -p mode:=real
```

### CLI

```bash
ros2 run cobot1 task_cli start pick_and_place
ros2 run cobot1 task_cli pause
ros2 run cobot1 task_cli resume
ros2 run cobot1 task_cli stop
ros2 run cobot1 task_cli status
```

## 파일 구조

```
cobot1/
├── cobot1/
│   ├── __init__.py
│   ├── nodes/                       # ROS2 실행 노드 (setup.py entry_points 대상)
│   │   ├── __init__.py
│   │   ├── motion_executor.py       # 모션 실행 + task_thread + robot_status 구독 (fault)
│   │   ├── robot_status_publisher.py# DSR2 monitoring 폴링 → RobotStatus (단일 poller)
│   │   ├── task_controller.py       # 상태 관리 + UI 서비스 경유
│   │   ├── ui_bridge.py             # 웹 UI ↔ ROS + Firebase sync (pull/push)
│   │   └── task_cli.py              # 터미널 CLI 도구
│   └── tasks/                       # 모션 task 모듈 (노드 아님, 라이브러리)
│       ├── __init__.py
│       ├── _common.py               # initialize_robot / safe_wait
│       ├── task_gripper_open.py     # 공용 그리퍼 해제 (모든 진입의 선행 모듈)
│       ├── task_home.py             # JReady 복귀
│       ├── task_rice.py             # 밥 배식
│       ├── task_tong.py             # 샐러드 + 돈까스 배식 (집게)
│       └── task_sauce.py            # 소스 배식
├── launch/
│   ├── motion.launch.py             # device1: motion_executor + robot_status_publisher
│   ├── task_system.launch.py        # 단일 머신 개발용: executor/status/controller (on/off 인자)
│   └── ui.launch.py                 # device2: task_controller + ui_bridge + rosbridge
├── web_ui/                          # 정적 웹 UI (app.js / home_ui.js / rosClient.js)
├── docs/
│   ├── ARCHITECTURE.md              # 이 문서
│   ├── recovery-gripper-emergency-subtask.md
│   ├── robot-state-recovery.md
│   ├── rclpy-executor-dsr2-troubleshooting.md
│   ├── firebase-integration.md
│   └── TODO.md
├── package.xml
└── setup.py

cobot_interfaces/
├── msg/
│   ├── TaskState.msg       # IDLE/RUNNING/STOPPING/ERROR/PAUSED + module_{name,label,index,total}
│   └── RobotStatus.msg     # robot_state enum + posj/tool_force/ext_torque + *_valid
├── srv/
│   ├── StartTask.srv
│   └── StopTask.srv        # NORMAL/IMMEDIATE/EMERGENCY/PAUSE/RESUME
│                           # + SAFE_STOP_RECOVER(5, legacy)
│                           # + TO_STANDBY(6) / TO_RECOVERY(7) / RECOVERY_DONE(8)
└── action/
    └── ExecuteTask.action  # (미사용, 참고용)
```

## 향후 확장 (pending)

- **web UI 표시 확장**: `payload.robot.{posj, tool_force, external_joint_torque}`
를 대시보드에 렌더. `ui_bridge` 측 payload 확장은 완료, 프런트엔드 미구현.
- **safety 계층**: 외력 임계치 감지 → `SUSPENDED` 상태. 단기엔 `task_controller`
내부 정책으로 구현, 장기엔 `**safety_node` 별도 분리** 권장 (이유:
monitoring 과 유사하게 RMW contention 이 없고 명령 채널과 독립적으로
동작해야 함).
- **실기 검증**: 가상 DRCF 에서 `robot_state` 전이 관찰 불가 — 실제 로봇으로
`MOVING`/`SAFE_STOP` 반응 확인 필요.
- `**recovery_`* 전용 점검 스텝**: 현재 일반 task 와 동일한 구성. 향후 복구
시 툴 체크 / 좌표 검증 / 그리퍼 DO readback 등 점검 단계를 추가할 수 있도록
키를 분리해 둠.
- `**gripper_open` 대기시간 단축**: 현재 rice `_gripper` 에 위임해 1.5s 대기.
DO 응답이 매우 빠르면 단축 가능.

## 참고 자료

- [Doosan Robotics API Manual - move_pause](https://doosanrobotics.github.io/doosan-robotics-api-manual/GL013303/mode/auto/motion_control_utilities/move_pause.html)
- [DSR2 Programming Manual - motion_pause](https://v2-manual.scroll.site/ko/v2-programming-manual/2.12.1/publish/motion_pause)
- `dsr_tests/test/test_cli_dsr_system.py` (move_pause/resume 사용 예시)
- `docs/rclpy-executor-dsr2-troubleshooting.md` (3-프로세스 분리 결정 경위)
- `docs/robot-state-recovery.md` (SAFE_STOP/SAFE_OFF 복구 서비스 플로우 상세)
- `docs/recovery-gripper-emergency-subtask.md` (2026-04-22 작업: 복구 그리퍼
해제 / 긴급정지 / 서브태스크 UI / holding failsafe / fault-aware task thread)
- `docs/firebase-integration.md` (Firebase sync 상세)

