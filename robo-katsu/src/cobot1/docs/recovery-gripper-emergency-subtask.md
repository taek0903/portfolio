# 복구 플로우 그리퍼 자동 해제 + Standby 긴급정지 + 서브태스크 UI 표시

작업일: 2026-04-22

3가지 이슈를 한 번에 처리한 기록. ARCHITECTURE / 복구 시스템 문서와 함께 읽는다.

## 1. 배경

운영 중 발견된 3가지 문제:

1. **SAFE_OFF / SAFE_STOP 복구 후 로봇이 물건을 쥔 채 JReady 로 이동** — 집게/주걱/
  소스통/돈까스/샐러드 가 그대로 물려 있는 상태에서 홈 또는 재시작을 누르면
   첫 amovej 가 실행되어 충돌·낙하 위험.
2. **STANDBY 상태에서 긴급정지 버튼을 눌러도 servo_off 가 안 됨** — task 가
  RUNNING/PAUSED 일 때만 EMERGENCY 가 forward 되도록 게이팅되어 있었음.
3. **UI 에 현재 진행 중 서브태스크 가 보이지 않음** — auto_serving 중에
  rice/tong/sauce 중 어느 것이 실행 중인지 표시 없음. tong 안의 샐러드·돈까스
   구분도 어려움.

## 2. 해결 개요

### 2.1 그리퍼 자동 해제

새 얇은 task 모듈 `task_gripper_open.py` 를 도입하고, `TASK_REGISTRY` 의
**모든 외부 진입 task** 첫 모듈로 합성한다.

```python
# motion_executor.py
TASK_REGISTRY = {
    "gripper_open":          [task_gripper_open],
    "home":                  [task_gripper_open, task_home],
    "rice":                  [task_gripper_open, task_rice],
    "tong":                  [task_gripper_open, task_tong],
    "sauce":                 [task_gripper_open, task_sauce],
    "auto_serving":          [task_gripper_open, task_rice, task_tong, task_sauce],
    # 복구 플로우 (UI 가 명시적으로 호출)
    "recovery_home":         [task_gripper_open, task_home],
    "recovery_rice":         [task_gripper_open, task_rice],
    "recovery_tong":         [task_gripper_open, task_tong],
    "recovery_sauce":        [task_gripper_open, task_sauce],
    "recovery_auto_serving": [task_gripper_open, task_rice, task_tong, task_sauce],
}
```

DO 매핑은 새로 정의하지 않고 **rice 의 기존 `_gripper("RELEASE", ctx)`** 에 위임한다
(rice 의 `release_pre` 스텝과 완전히 동일한 로직). 이렇게 하면 하드웨어 DO 세팅을
여러 곳에서 재정의할 필요가 없고, 이미 현장 검증된 경로를 그대로 사용할 수 있다:

```python
# tasks/task_gripper_open.py
from . import task_rice

def run(ctx):
    if not task_rice._gripper("RELEASE", ctx):
        return False, "Task stopped by user"
    return True, "Gripper released"
```

실 하드웨어는 DO1~DO3 만 사용한다. rice 원본은 DO4 도 0 리셋하지만 하드웨어에
존재하지 않는 채널이라 무해하게 무시된다 (rice 가 현재 현장에서 정상 동작 중).

평상시 "🏠 홈으로" 버튼도 `recovery_home` 을 호출해 동일한 안전 경로를 탄다.

### 2.2 UI 재시작은 중단된 모듈부터

SAFE_STOP/SAFE_STOP_2 진입 시점에 `task_status.module_name` 을 캡처하고,
"🔁 작업 재시작" 버튼이 다음 매핑으로 task 를 시작한다:

```js
// app.js
const RECOVERY_TASK_FOR = {
  auto_serving: 'recovery_auto_serving',
  rice:         'recovery_rice',
  tong:         'recovery_tong',
  sauce:        'recovery_sauce',
  home:         'recovery_home',
};
```

→ auto_serving 중 tong 에서 멈춘 경우 재시작은 `recovery_tong` (그리퍼 해제 +
tong) 만 실행되어 rice/sauce 는 반복하지 않는다.

### 2.3 Standby 긴급정지

두 곳에서 `STATE ∈ {RUNNING, PAUSED}` 게이팅을 제거:

- `ui_bridge._handle_safety_cmd`: EMERGENCY_STOP 명령을 받으면 task_state 와
무관하게 `StopTask(stop_type=STOP_EMERGENCY)` 를 항상 forward.
- `task_controller._stop_callback`: `stop_type == 2` (EMERGENCY) 는 분기를 앞으로
빼 무조건 허용. 기존 `else` 블록의 "No task running" 리젝트 경로 밖으로 이동.

`motion_executor._handle_emergency_stop` 은 이미 task_state 와 무관하게
`move_stop(QUICK) + set_robot_control(SERVO_OFF)` 를 실행하므로 그대로 둠.

### 2.4 서브태스크 UI 표시

`cobot_interfaces/msg/TaskState.msg` 확장:

```
string current_module_name     # 기계 식별자 (TASK_NAME): "rice"/"tong"/"sauce"/...
string current_module_label    # UI 표시용: "밥" / "샐러드·돈까스" / "소스"
uint32 module_index            # 0-base
uint32 module_total            # 합성 모듈 수 (auto_serving 는 4 = gripper_open+rice+tong+sauce)
```

각 task 모듈에 `TASK_LABEL` 상수 추가:


| 모듈             | TASK_LABEL | 아이콘 (UI) |
| -------------- | ---------- | -------- |
| `gripper_open` | 그리퍼 해제     | 🖐       |
| `home`         | 홈 복귀       | 🏠       |
| `rice`         | 밥          | 🍚       |
| `tong`         | 샐러드·돈까스    | 🍱       |
| `sauce`        | 소스         | 🍯       |


**전파 경로**

```
motion_executor._run_task_sequence
  └─ shared.module_name / module_label / module_index / module_total
     → TaskState (execute_task/state)
task_controller._executor_state_callback → _publish_state
  └─ TaskState (task/state) — task_name / current_step / module_* 모두 forward
     (기존에 task_controller 가 이 필드들을 forward 하지 않고 있던 버그도 함께 수정)
ui_bridge._task_state_callback → _publish_status
  └─ task_status.module_name / module_label / module_index / module_total
     / step_name / current_step / total_steps 추가
app.js
  └─ describeSubTask() 가 module_name → icon 매핑 + tong 내부 phase 계산
     (step_name 에 "salad" 포함 → 샐러드, "pork" 포함 → 돈까스)
```

UI 표시 위치:

- **헤더 Current Task** 아래에 파란색 서브라벨 한 줄: `🍱 샐러드·돈까스 · 돈까스`
- **CircularGauge 중앙**: WORKING 상태일 때 아이콘/라벨을 서브태스크로 덮어쓰고,
tong 내부 phase (샐러드/돈까스) 는 하단의 작은 rounded badge 로 표시.

사용자 요청대로 `pork1_`* / `pork2_`* 는 모두 "돈까스" 로 통합 (1/2 구분 X).
집게 집기/반납/ready 등의 준비 스텝은 phase = null 로 두고 module_label 만 표시
(복잡도 증가 방지).

## 3. 변경 파일


| 파일                                                     | 핵심 변경                                                                                                                                                                                                                                                                                                                                   |
| ------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `src/cobot_interfaces/msg/TaskState.msg`               | 4개 module_* 필드 추가                                                                                                                                                                                                                                                                                                                       |
| `src/cobot1/cobot1/tasks/task_gripper_open.py`         | 신규 — 공용 그리퍼 해제 모듈                                                                                                                                                                                                                                                                                                                       |
| `src/cobot1/cobot1/tasks/task_home.py`                 | `TASK_LABEL = "홈 복귀"`                                                                                                                                                                                                                                                                                                                   |
| `src/cobot1/cobot1/tasks/task_rice.py`                 | `TASK_LABEL = "밥"`                                                                                                                                                                                                                                                                                                                      |
| `src/cobot1/cobot1/tasks/task_tong.py`                 | `TASK_LABEL = "샐러드·돈까스"`                                                                                                                                                                                                                                                                                                                |
| `src/cobot1/cobot1/tasks/task_sauce.py`                | `TASK_LABEL = "소스"`                                                                                                                                                                                                                                                                                                                     |
| `src/cobot1/cobot1/tasks/__init__.py`                  | `task_gripper_open` 노출                                                                                                                                                                                                                                                                                                                  |
| `src/cobot1/cobot1/nodes/motion_executor.py`           | TASK_REGISTRY 재구성, `_SharedState` 에 module_name/label 추가, `_publish_task_state` / `_on_start` / `_run_task_sequence` / `_reset_executor_state` / `_finish_idle`* 에서 module 필드 세팅                                                                                                                                                        |
| `src/cobot1/cobot1/nodes/task_controller.py`            | `_stop_callback` 에 `stop_type == 2` 분기 추가 (task_state 무관 허용), `_executor_state_callback` 에서 task_name/current_step/module_* 캐싱, `_publish_state` 에서 forward                                                                                                                                                                             |
| `src/cobot1/cobot1/nodes/ui_bridge.py`                  | `_handle_safety_cmd` EMERGENCY 무조건 forward, `_task_state_callback` 에 module 필드 캐싱, `_publish_status` payload 에 module_name/label/index/total + step_name + current/total_steps 추가, `ACTION_TO_TASK` / `ACTION_LABELS` 에 `recovery`_*, `gripper_open` 추가                                                                                 |
| `src/cobot1/web_ui/app.js`                             | `RECOVERY_TASK_FOR`, `MODULE_ICONS`, `MODULE_LABELS`, `getTongPhase`, `describeSubTask` 추가. SAFE_STOP 캡처 시 module_name 우선. `handleGoHome` → `recovery_home`, `handleRestartTask` → `recovery_<module>`. "🏠 홈으로" 일반 버튼도 `recovery_home` 호출. `CircularGauge` 에 `subTask` prop 추가 — 중앙 서브태스크 아이콘/라벨/phase 표시. 헤더 Current Task 아래 서브라벨 추가. |


## 4. 빌드 / 실행

`TaskState.msg` 가 바뀌었으므로 `**cobot_interfaces` 재빌드 필수**:

```bash
cd ~/cobot_ws
rm -rf build/cobot_interfaces install/cobot_interfaces
source /opt/ros/humble/setup.bash
colcon build --packages-select cobot_interfaces
source install/setup.bash
colcon build --packages-select cobot1 --symlink-install
```

노드 재시작:

```bash
ros2 launch cobot1 task_system.launch.py
ros2 run cobot1 ui_bridge
# 웹 UI 브라우저 새로고침 (Firebase / rosbridge 재구독)
```

## 5. 검증 시나리오

1. **Standby 긴급정지**: 아무 task 도 돌지 않는 상태에서 UI 비상정지 버튼 →
  실선반 빨간 불(서보 차단) 확인. `ros2 topic echo /dsr01/motion_executor/robot_status`
   에 `EMERGENCY_STOP` 진입 확인.
2. **정상 홈 복귀**: WORKING 이 아닐 때 "🏠 홈으로" 버튼 → gauge 에 🖐 그리퍼 해제
  → 🏠 홈 복귀 순서로 서브태스크가 바뀌고 완료 시 IDLE.
3. **auto_serving → SAFE_STOP → 재시작**:
  - tong 의 돈까스 단계에서 외력/안전정지 트리거 → UI 에 Phase1 (자동/수동 복구)
  - 자동 복구 완료 → Phase2 (홈으로 / 재시작)
  - "🔁 작업 재시작" → `recovery_tong` 실행 (그리퍼 해제 후 tong 처음부터) →
  rice/sauce 는 반복하지 않음.
4. **서브태스크 표시**: auto_serving 실행 중 gauge 중앙이
  `🖐 그리퍼 해제` → `🍚 밥` → `🍱 샐러드·돈까스` (하단에 "샐러드" → "돈까스"
   phase) → `🍯 소스` 로 전환되는지 확인.

## 6. SAFE_STOP 진입 시 그리퍼 진동 방지

**증상**: 외력 감지 등으로 SAFE_STOP/SAFE_OFF 에 진입하면 아무것도 누르지
않아도 그리퍼가 반복적으로 열렸다 닫혔다 하다가 마지막 `RELEASE` 로 "확" 열림.

**원인**:

- `motion_executor` 가 `robot_status` 를 구독하지 않아 task thread 가 하드웨어
SAFE_STOP 을 모름.
- DSR 은 SAFE_STOP 중 모션 명령(amovej/amovel)을 차단하지만 `set_digital_output`
은 계속 통과시킨다. 또한 `check_motion()` 이 즉시 idle(0) 로 반환되어
`ctx.wait_motion()` 이 "완료"로 오인하고 다음 STEP 으로 진행.
- 결과적으로 task 의 STEPS 리스트가 빠르게 소비되며 각 `_gripper(...)` 호출의
DO 세트 패턴이 연속적으로 실행되어 진동처럼 보임.

**수정**: `motion_executor` 에 `RobotStatus` 구독 추가. 하드웨어 상태가
`ROBOT_FAULT_STATES = {EMERGENCY_STOP, SAFE_OFF, SAFE_STOP, SAFE_OFF2, SAFE_STOP2}`
중 하나로 진입하면 `_shared.stop_requested = True` + `state = ERROR` 로 세팅.
task thread 는 다음 `ctx.check_stop()` 지점 (보통 다음 `_gripper`/`wait_motion`
진입부) 에서 즉시 리턴되어 후속 `_gripper(...)` 호출이 더 이상 발생하지 않는다.

`move_stop` 은 호출하지 않는다 — DSR 이 이미 하드웨어 레벨에서 모션을 막은
상태이고, 정식 복구는 UI "자동/수동 복구" 버튼 (stop_type=6/7/8) 을 통해 진행.

## 7. Holding failsafe — SAFE_STOP 진입 시 그리퍼 상태 유지

**증상 (6번 수정 이후에도 남아있던 동작)**: SAFE_OFF / SAFE_STOP 에 진입하면
사용자가 아무것도 누르지 않아도 그리퍼가 자동으로 열림.

**원인**: 각 `_gripper()` 함수의 `safe_wait` 실패 분기에 "안전을 위해 RELEASE 로
DO 재설정" 로직이 있었음. 6번 fix 덕분에 진동은 사라졌지만, 그래도 `_gripper`
실행 중에 stop 이 감지되면 마지막에 한 번 강제로 RELEASE DO 를 세팅하고 반환.

**정책 결정 (2026-04-22)**: "holding failsafe" 로 변경.


| 항목                          | 변경 전             | 변경 후                         |
| --------------------------- | ---------------- | ---------------------------- |
| SAFE_STOP 중 `_gripper` 종료 시 | RELEASE DO 강제 세팅 | 현재 DO 상태 그대로 유지              |
| payload 유지                  | 낙하 가능 (열림)       | 계속 쥐고 있음                     |
| 작업자 주변 안전                   | 열려서 비켜주는 이점은 있음  | `task_gripper_open` 으로 명시 해제 |
| 산업 관행                       | 비표준              | 표준 ("fail-hold")             |


**수정**: `task_rice._gripper` / `task_tong._gripper` / `task_sauce._gripper` 의
`safe_wait` 실패 분기에서 DO 재설정 호출 제거. 즉, stop 감지 시 **현재 DO 상태를
그대로 두고 False 만 반환**한다.

- rice 는 reference 정책상 액션 직전에 DO1~DO4=0 리셋을 하므로 reset 직후
0.1s 구간에 stop 이 들어오면 DO 가 "all zero" 상태로 남을 수 있는데, 이는
rice 의 원래 액션 간 전이 상태와 동일해 새로운 위험을 만들지 않는다.
- tong / sauce 는 reset 단계가 없으므로 stop 시점 직전의 목표 DO 상태가 그대로
유지됨 (예: GRIP_TIGHT 중 stop 이면 GRIP_TIGHT 유지).

**명시적 해제 경로**: 복구 플로우는 항상 `recovery`_* task 로 들어오고, 그
첫 모듈이 `task_gripper_open` (= rice `_gripper("RELEASE")`) 이므로 사용자가
"🏠 홈으로" / "🔁 작업 재시작" / "🏠 홈으로" (일반) 중 어느 것을 누르든 복귀
모션 시작 전에 반드시 한 번 그리퍼가 열린다.

## 8. 주의 / 향후 개선 아이디어

- `gripper_open` 의 대기 시간 1.5s 는 rice/tong 의 기본 wait 와 동일. DO 응답이
매우 빠르면 단축 가능.
- tong 의 phase 분리는 step_name 접두어에 의존. 향후 step_name 네이밍 리팩토링
시 `getTongPhase` 도 같이 점검.
- task_controller 가 기존에 task_name / step 필드 를 forward 하지 않고 있어
UI 에서 활동 라벨 일부가 empty 로 표시되던 잠재 버그가 있었음. 이번에 같이 수정.
- `home` 과 `recovery_home` 은 현재 동일한 합성. 향후 복구 전용 점검(툴 체크 등) 이
필요해지면 `recovery_home` 만 확장해 분기 가능.

