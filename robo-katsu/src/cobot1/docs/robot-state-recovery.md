# 로봇 상태 관리 및 복구 시스템

> 작성일: 2026-04-21  
> 관련: `motion_executor.py`, `task_controller_node.py`, `ui_bridge.py`, `StopTask.srv`

## 개요

외력/충돌 등으로 DSR 로봇이 안전 정지(SAFE_STOP/SAFE_OFF 계열) 상태에 진입했을 때,
**티칭펜던트 없이 UI(소프트웨어)만으로 복구**할 수 있도록 구현한 시스템.

DSR2의 `set_robot_control` 서비스를 활용해 상태별로 적절한 복구 경로를 제공한다.

---

## DSR2 로봇 상태 (GetRobotState)


| 값   | 상태명              | 설명                   |
| --- | ---------------- | -------------------- |
| 0   | `INITIALIZING`   | 초기화 중                |
| 1   | `STANDBY`        | 대기 (정상 상태, 작업 실행 가능) |
| 2   | `MOVING`         | 이동 중                 |
| 3   | `SAFE_OFF`       | 서보 꺼짐 (1계열)          |
| 4   | `TEACHING`       | 직접 교시 모드             |
| 5   | `SAFE_STOP`      | 안전 정지 (1계열)          |
| 6   | `EMERGENCY_STOP` | 비상 정지                |
| 7   | `HOMMING`        | 원점 복귀 중              |
| 8   | `RECOVERY`       | 복구 모드 (손으로 이동 가능)    |
| 9   | `SAFE_STOP2`     | 보호 정지 (2계열)          |
| 10  | `SAFE_OFF2`      | 서보 꺼짐 (2계열)          |
| 15  | `NOT_READY`      | 준비 안 됨               |
| 255 | `UNKNOWN`        | 상태 수신 전              |


### 안전 정지 계열 구분

- **1계열** (`SAFE_STOP`, `SAFE_OFF`): 덜 심각, 직접 STANDBY로 복구 가능
- **2계열** (`SAFE_STOP2`, `SAFE_OFF2`): 더 심각, RECOVERY 경유 필요

---

## DSR 안전 정지 체계 (IEC 61800-5-2)

### Stop Category 기준


| 명칭                        | Stop Category | 동작                       | 서보 상태 |
| ------------------------- | ------------- | ------------------------ | ----- |
| **STO** (Safe Torque Off) | Category 0    | 즉시 전원 차단                 | OFF   |
| **SS1** (Safe Stop 1)     | Category 1    | 최대 감속 → 서보 OFF           | OFF   |
| **SS2** (Safe Stop 2)     | Category 2    | 최대 감속 → 서보 ON (SOS 모니터링) | ON    |
| **RS1** (Reflex Stop)     | Category 2    | 충돌 방향 반대로 이동 후 대기        | ON    |


### 안전 정지 트리거 조건

DSR 안전 컨트롤러(하드웨어)가 **자동으로** 감지하고 정지시킴:


| 트리거                             | 선택 가능한 Stop Mode   | 기본값 |
| ------------------------------- | ------------------ | --- |
| **Emergency Stop** (비상 정지 버튼)   | STO, SS1 만         | SS1 |
| **Protective Stop** (외부 안전 장치)  | STO, SS1, SS2      | SS2 |
| **Collision Detection** (충돌 감지) | STO, SS1, SS2, RS1 | SS2 |
| Joint Angle/Speed 제한 위반         | STO, SS1, SS2      | SS2 |
| Joint Torque 제한 위반              | STO 만              | STO |
| TCP Position/Speed 제한 위반        | STO, SS1, SS2      | SS2 |


### 기본 설정으로 동작

**외력 감지 시 자동으로 멈추는 이유:**

DSR에는 **기본 안전 설정**이 적용되어 있어서, 별도 설정 없이도:

- 충돌 감지 활성화 (기본 민감도)
- 관절 토크/속도 제한 모니터링
- TCP 위치/속도 제한 모니터링

이 조건들이 위반되면 **DSR 안전 컨트롤러가 자동으로 SAFE_STOP 계열로 진입**시킴.

### Stop Mode 설정 위치

Stop Mode를 변경하려면 **티칭펜던트**에서 설정:

```
티칭펜던트 → 설정 → 안전 → Safety Stop Mode
```


| 항목                    | 설정 가능 값            |
| --------------------- | ------------------ |
| Emergency Stop        | STO, SS1           |
| Protective Stop       | STO, SS1, SS2      |
| Collision Detection   | STO, SS1, SS2, RS1 |
| Joint Limit Violation | STO, SS1, SS2      |
| TCP Limit Violation   | STO, SS1, SS2      |


> **참고**: 안전 관련 설정은 티칭펜던트에서만 변경 가능. 소프트웨어 API로는 변경 불가.

### 소프트웨어에서 할 수 있는 것 / 없는 것


| 가능                          | 불가능                 |
| --------------------------- | ------------------- |
| `move_stop()` — 일반 정지       | SAFE_STOP 모드로 강제 진입 |
| `set_robot_control()` — 복구  | Stop Mode 설정 변경     |
| 상태 모니터링 (`get_robot_state`) | 안전 민감도 변경           |


```python
# ❌ 이런 API 없음
dsr.enter_safe_stop()
dsr.set_collision_sensitivity(0.5)  # 티칭펜던트에서만 가능

# ✅ 소프트웨어에서 할 수 있는 것
move_stop()                          # 일반 정지
set_robot_control(2)                 # SAFE_STOP → STANDBY 복구
get_robot_state()                    # 상태 확인
```

### 1계열 vs 2계열 차이 (추정)

공식 문서에 명확한 구분이 없지만, 복구 경로 차이로 추정:


| 계열      | 상태                    | 추정 원인                  | 복구          |
| ------- | --------------------- | ---------------------- | ----------- |
| **1계열** | SAFE_STOP, SAFE_OFF   | SS2/SS1 정지, 일반적인 안전 위반 | 직접 STANDBY  |
| **2계열** | SAFE_STOP2, SAFE_OFF2 | 더 심각한 위반 (외부 충격 등)     | RECOVERY 경유 |


**2계열이 RECOVERY를 경유하는 이유**: 로봇이 예상치 못한 위치에 있을 수 있어서,
사용자가 손으로 안전한 위치로 이동시킬 기회를 주는 것으로 추정.

### 실제 운영 시 권장

1. **기본 설정 그대로 사용** — 대부분의 경우 충분함
2. **민감도 조정 필요 시** — 티칭펜던트에서 Collision Sensitivity 조정
3. **소프트웨어에서는 복구만 담당** — `set_robot_control` 서비스 활용

---

## SetRobotControl 서비스

DSR2가 제공하는 `/{ns}/system/set_robot_control` 서비스로 상태 전환을 수행한다.


| 값     | 상수명                          | 전환                        | 비고             |
| ----- | ---------------------------- | ------------------------- | -------------- |
| 0     | `CONTROL_INIT_CONFIG`        | NOT_READY → INITIALIZING  | T/P 전용         |
| 1     | `CONTROL_ENABLE_OPERATION`   | INITIALIZING → STANDBY    | T/P 전용         |
| **2** | `CONTROL_RESET_SAFET_STOP`   | **SAFE_STOP → STANDBY**   | ✅ S/W 가능       |
| **3** | `CONTROL_RESET_SAFET_OFF`    | **SAFE_OFF → STANDBY**    | ✅ S/W 가능       |
| **4** | `CONTROL_RECOVERY_SAFE_STOP` | **SAFE_STOP2 → RECOVERY** | ✅ S/W 가능       |
| **5** | `CONTROL_RECOVERY_SAFE_OFF`  | **SAFE_OFF2 → RECOVERY**  | ✅ S/W 가능       |
| 6     | `CONTROL_RECOVERY_BACKDRIVE` | SAFE_OFF2 → RECOVERY      | H/W 기반, 리부팅 필요 |
| **7** | `CONTROL_RESET_RECOVERY`     | **RECOVERY → STANDBY**    | ✅ S/W 가능       |


---

## 복구 경로

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         상태 전환 다이어그램                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────────┐                              ┌──────────────┐         │
│  │  SAFE_STOP   │──── control(2) ────────────► │   STANDBY    │         │
│  │     (5)      │                              │     (1)      │         │
│  └──────────────┘                              └──────────────┘         │
│                                                       ▲                 │
│  ┌──────────────┐                                     │                 │
│  │   SAFE_OFF   │──── control(3) ─────────────────────┘                 │
│  │     (3)      │                                                       │
│  └──────────────┘                                                       │
│                                                                         │
│  ┌──────────────┐         ┌──────────────┐         ┌──────────────┐     │
│  │ SAFE_STOP2   │─ ctrl ─►│   RECOVERY   │─ ctrl ─►│   STANDBY    │     │
│  │     (9)      │  (4)    │     (8)      │  (7)    │     (1)      │     │
│  └──────────────┘         └──────────────┘         └──────────────┘     │
│                                  ▲                                      │
│  ┌──────────────┐                │                                      │
│  │  SAFE_OFF2   │──── control(5) ┘                                      │
│  │    (10)      │                                                       │
│  └──────────────┘                                                       │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### 자동 복구 (TO_STANDBY)

위험물이 제거된 경우, UI에서 "자동 복구" 버튼을 누르면:

1. 현재 상태에 맞는 `set_robot_control` 호출
2. 2계열이면 RECOVERY 경유
3. 최종적으로 STANDBY 도달
4. `set_robot_mode(AUTONOMOUS)` 호출
5. 후속 task 실행 가능

### 수동 복구 (TO_RECOVERY → RECOVERY_DONE)

로봇을 직접 손으로 이동해야 하는 경우:

1. "수동 복구" 버튼 → `set_robot_control(4 or 5)` → RECOVERY 모드
2. 사용자가 로봇을 손으로 안전한 위치로 이동
3. "이동 완료" 버튼 → `set_robot_control(7)` → STANDBY
4. 후속 task 실행 가능

> **주의**: 수동 복구(RECOVERY 모드)는 **SAFE_STOP2/SAFE_OFF2(2계열)에서만** 직접 진입 가능.
> 1계열(SAFE_STOP/SAFE_OFF)에서는 자동 복구를 사용해야 함.

---

## StopTask.srv stop_type 상수

`cobot_interfaces/srv/StopTask.srv`에 정의된 복구 관련 상수:


| 값     | 상수명                 | 설명                                 |
| ----- | ------------------- | ---------------------------------- |
| 0     | `STOP_NORMAL`       | 일반 정지 (감속)                         |
| 1     | `STOP_IMMEDIATE`    | 즉시 정지                              |
| 2     | `STOP_EMERGENCY`    | 비상 정지                              |
| 3     | `PAUSE`             | 일시 정지                              |
| 4     | `RESUME`            | 재개                                 |
| 5     | `SAFE_STOP_RECOVER` | (레거시) SAFE_STOP 복구, TO_STANDBY와 동일 |
| **6** | `TO_STANDBY`        | 자동 복구: SAFE_* → STANDBY            |
| **7** | `TO_RECOVERY`       | 수동 복구 진입: SAFE_*2 → RECOVERY       |
| **8** | `RECOVERY_DONE`     | 수동 복구 완료: RECOVERY → STANDBY       |


---

## 구현 위치

### motion_executor.py

- `_call_set_robot_control()`: DSR `set_robot_control` 서비스 호출 헬퍼
- `_handle_to_standby()`: 자동 복구 (stop_type 5, 6)
- `_handle_to_recovery()`: 수동 복구 진입 (stop_type 7)
- `_handle_recovery_done()`: 수동 복구 완료 (stop_type 8)

### task_controller_node.py

- `_stop_callback()`: stop_type 5~8 처리, executor에 전달
- `_robot_status_callback()`: 안전 정지 감지 시 TaskState를 ERROR로 전환

### ui_bridge.py

- `DSR_SAFETY_ROBOT_STATES`: 안전 정지 상태 집합 (3, 5, 6, 9, 10)
- 안전 정지 시 진행률 동결, `robot_status` 문구 변경
- 명령(serving_cmd, pause_cmd) 차단

---

## CLI 테스트 명령

```bash
# 빌드
cd ~/cobot_ws
colcon build --packages-select cobot_interfaces cobot1 --symlink-install
source install/setup.bash

# DSR set_robot_control 직접 호출
ros2 service call /dsr01/system/set_robot_control \
  dsr_msgs2/srv/SetRobotControl "{robot_control: 2}"

# 자동 복구 (stop_type=6)
ros2 service call /dsr01/task/stop \
  cobot_interfaces/srv/StopTask "{stop_type: 6}"

# 수동 복구 진입 (stop_type=7)
ros2 service call /dsr01/task/stop \
  cobot_interfaces/srv/StopTask "{stop_type: 7}"

# 수동 복구 완료 (stop_type=8)
ros2 service call /dsr01/task/stop \
  cobot_interfaces/srv/StopTask "{stop_type: 8}"

# 상태 확인
ros2 run cobot1 task_cli status
ros2 topic echo /dsr01/motion_executor/robot_status --field robot_state
```

---

## UI 플로우 (예정)

```
Phase 1: 위험 감지
├── 안전 정지 감지 (SAFE_*)
├── "충돌 또는 경로 상의 이유로 작업이 중단되었습니다"
└── 복구 방법 선택:
    ├── [자동 복구] → stop_type=6
    └── [수동 복구] → stop_type=7 (2계열만)

Phase 2: 복구 완료 (STANDBY 도달 후)
├── [홈으로 이동] → StartTask('home')
└── [작업 재시작] → StartTask('tong_source')
```

---

## 통합 시스템 상태 (SystemState)

Task 상태와 Robot 상태를 조합해 **일관된 통합 상태**를 제공한다.
UI에서 복잡한 파생 로직 없이 이 값만 보고 렌더링을 결정할 수 있다.

### 상태 정의


| 값                  | 상수명            | 설명          | UI 동작                       |
| ------------------ | -------------- | ----------- | --------------------------- |
| **정상 상태 (0-9)**    |                |             |                             |
| 0                  | `IDLE`         | 대기          | 작업 요청 가능                    |
| 1                  | `WORKING`      | 작업 수행 중     | 일시정지만 가능                    |
| 2                  | `PAUSED`       | 일시 정지       | 재개만 가능                      |
| **복구 필요 (10-19)**  |                |             |                             |
| 10                 | `SAFE_STOP`    | 안전 정지 (1계열) | 복구 패널 Phase 1               |
| 11                 | `SAFE_STOP_2`  | 보호 정지 (2계열) | 복구 패널 Phase 1 (수동 복구 옵션 포함) |
| 12                 | `EMERGENCY`    | 비상 정지       | 모든 요청 차단                    |
| 13                 | `RECOVERY`     | 수동 복구 모드    | "이동 완료" 버튼 대기               |
| **복구 완료 (20-29)**  |                |             |                             |
| 20                 | `RECOVERED`    | 복구 완료       | 복구 패널 Phase 2 (후속 동작 선택)    |
| **에러/비정상 (30-39)** |                |             |                             |
| 30                 | `ERROR`        | 일반 에러       | 에러 메시지 표시                   |
| 31                 | `NOT_READY`    | 준비 안 됨      | 대기 중 표시                     |
| 32                 | `DISCONNECTED` | 연결 안 됨      | 연결 오류 표시                    |


### 상태 전이 다이어그램

```
                              SystemState 전이 흐름

                                 ┌────────┐
                     ┌──────────►│  IDLE  │◄──────────────────────────┐
                     │           └───┬────┘                           │
                     │               │ StartTask                      │
                     │               ▼                                │
                     │         ┌──────────┐                           │
                     │    ┌───►│ WORKING  │◄────┐                     │
                     │    │    └────┬─────┘     │                     │
                     │    │         │ Pause     │ Resume              │
                     │    │         ▼           │                     │
                     │    │    ┌──────────┐     │                     │
                     │    │    │  PAUSED  │─────┘                     │
                     │    │    └────┬─────┘                           │
                     │    │         │ 완료/정지                         │
                     │    │         ▼                                 │
                     │    └───────────────────────────────────────────┘
                     │
    ╔════════════════╧═══════════════════════════════════════════════════╗
    ║           안전 정지 발생 (어떤 상태에서든 진입 가능)                       ║
    ╚═══════════════╤════════════════════════════════════════════════════╝
                    │
          ┌─────────┴─────────┐
          ▼                   ▼
    ┌───────────┐       ┌─────────────┐
    │ SAFE_STOP │       │ SAFE_STOP_2 │
    │   (10)    │       │    (11)     │
    └─────┬─────┘       └──────┬──────┘
          │                    │
          │ 자동복구             │ 자동복구  │ 수동복구
          │                    │         ▼
          │                    │   ┌──────────┐
          │                    │   │ RECOVERY │
          │                    │   │   (13)   │
          │                    │   └────┬─────┘
          │                    │        │ 이동완료
          ▼                    ▼        ▼
    ┌─────────────────────────────────────────┐
    │              RECOVERED (20)             │
    │         (후속 동작 선택 대기)               │
    └──────────────────┬──────────────────────┘
                       │
                       │ 홈이동 / 재시작 / 취소
                       ▼
                 ┌───────────┐
                 │   IDLE    │
                 │ / WORKING │
                 └───────────┘
```

### 구현 위치

- **ui_bridge.py**: `_compute_system_state()` — 통합 상태 계산
- **app.js**: `SYSTEM_STATE` 상수, `STATUS_CONFIG` — UI 렌더링 매핑

### UI payload

```json
{
  "system_status": {
    "state": 0,           // int (0-32) — 통합 시스템 상태 코드
    "name": "IDLE",       // 영문 상수명
    "label": "대기"        // 한글 표시용 라벨
  },
  "task_status": {
    "state": 0,           // int (0-4) — TaskState enum
    "name": "",           // task 이름 (예: "tong_source")
    "label": "",          // 한글 라벨 (예: "자동 배식")
    "progress": 0         // 0-100 정수
  },
  "robot_status": {
    "state": 1,           // DSR robot_state enum (1=STANDBY)
    "name": "STANDBY",    // DSR 상태 이름
    "posj": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],           // 관절 각도 (deg)
    "tool_force": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],     // 툴 힘/토크
    "external_joint_torque": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  // 외부 토크
  },
  "message": {
    "text": "UI Bridge가 시작되었습니다.",
    "level": "success"    // "info" | "warning" | "error" | "success"
  },
  "mode": "real",         // "real" | "virtual"
  "updated_at": "2026-04-21T15:30:00+09:00"  // ISO8601 KST
}
```

- `system_state`: 숫자 코드 (0-32)
- `system_state_name`: 영문 상수명
- `system_state_label`: 한글 표시 라벨
- `op_state`, `robot_status`: 하위 호환용 기존 필드

---

## 제한사항

1. **EMERGENCY_STOP(6)**: SW로 해제 불가, H/W 버튼 필요
2. **CONTROL_RECOVERY_BACKDRIVE(6)**: H/W 기반, 리부팅 필요
3. **가상 환경**: `set_robot_control` 서비스는 성공해도 상태가 안 바뀔 수 있음 (시뮬레이터 한계)
4. **1계열에서 RECOVERY 모드 직접 진입 불가**: STANDBY 거쳐야 함

---

## 관련 문서

- [ARCHITECTURE.md](./ARCHITECTURE.md): 전체 시스템 아키텍처
- [SetRobotControl.srv](../../doosan-robot2/dsr_msgs2/srv/system/SetRobotControl.srv): DSR2 서비스 정의
- [StopTask.srv](../../cobot_interfaces/srv/StopTask.srv): 복구 stop_type 정의

