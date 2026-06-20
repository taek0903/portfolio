# Pause/Gripper Safety Bugfixes (2026-04)

이 문서는 2026년 4월 발견된 일시정지(pause) 및 그리퍼 안전 관련 버그와 수정 내용을 기록합니다.

## Issue 1: 일시정지 버튼 간헐적 동작 문제

### 증상

- 일시정지 버튼을 눌렀는데 task가 계속 진행됨
- 버튼을 두 번 눌러야 멈추는 경우가 있음
- 간헐적으로 재현됨

### 원인 분석

#### 1. MovePause 콜백에서 실패 케이스 무시

`motion_executor.py`의 `_on_move_pause_done` 콜백에서 `res.success=False`인 경우 아무런 처리(로깅, 상태 변경)가 없었습니다.

```python
# 수정 전 (버그)
def _on_move_pause_done(self, future):
    res = future.result()
    if res and res.success:
        # 성공 처리만 있음
        ...
    # else: 아무것도 안 함!
```

#### 2. 중복 pause 요청 방지 메커니즘 없음 (Race Condition)

빠르게 두 번 클릭하면 첫 번째 MovePause 서비스 응답이 오기 전에 두 번째 요청이 상태 체크를 통과할 수 있었습니다.

**타임라인 예시:**

1. T0: 사용자 pause 클릭 → state=RUNNING 체크 통과 → MovePause 호출
2. T1: MovePause 서비스 처리 중... (state는 아직 RUNNING)
3. T2: 사용자 pause 또 클릭 → state=RUNNING 체크 통과 → MovePause 또 호출
4. T3: 첫 번째 MovePause 콜백 → state=PAUSED
5. T4: 두 번째 MovePause 콜백 → state가 PAUSED라서 무시됨

### 수정 내용

#### 1. 실패 케이스 로깅 추가

```python
def _on_move_pause_done(self, future):
    res = future.result()
    if res and res.success:
        # 성공 처리
        ...
    elif res:
        self.get_logger().warn(f"MovePause returned success=False: ...")
    else:
        self.get_logger().warn("MovePause returned None result")
```

#### 2. In-flight 가드 추가

`_SharedState`에 `pause_in_flight` 플래그를 추가하여 pause 요청이 진행 중일 때 중복 요청을 차단합니다.

```python
class _SharedState:
    def __init__(self):
        ...
        self.pause_in_flight = False  # 추가

def _handle_pause(self, response):
    with self._shared.lock:
        if self._shared.pause_in_flight:
            response.success = False
            response.message = "Pause already in progress"
            return response
        self._shared.pause_in_flight = True
    ...
```

#### 3. MovePause 실패 시 안전 중단 (MoveStop fallback)

MovePause가 실패하면 단순 로깅이 아니라 **task 전체를 안전하게 중단**합니다.

```python
def _on_move_pause_done(self, future):
    res = future.result()
    if res and res.success:
        # 성공: state → PAUSED
        ...
    else:
        # 실패: task 전체 중단 (안전 조치)
        self._abort_task_on_pause_failure("일시정지 실패 - 작업 중단 중...")

def _abort_task_on_pause_failure(self, message: str):
    """MovePause 실패 시 안전하게 task를 중단."""
    with self._shared.lock:
        self._shared.pause_in_flight = False
        self._shared.stop_requested = True  # task thread에 중단 신호
        self._shared.state = ExecutorState.STOPPING
        self._shared.message = message
    self._send_move_stop(0)  # 로봇 모션 정지
    # 3초 watchdog: STOPPING 상태가 풀리지 않으면 강제 IDLE
```

**왜 이렇게 해야 하는가?**

- 사용자가 pause를 눌렀다는 것은 "멈추고 싶다"는 의도
- MovePause가 실패하면 로봇은 계속 움직임
- 단순 로깅만 하면 사용자 의도와 다르게 작업이 계속 진행됨
- 안전을 위해 task 자체를 중단하는 것이 올바른 대응

### 관련 파일

- `cobot1/motion_executor.py`

### 테스트

- `test/test_pause_resume_race.py`

---

## Issue 1-B: Pause 후에도 현재 모션이 끝까지 진행되는 문제 (심층 분석, 2026-04-21)

### 증상

- UI에서 pause를 눌렀는데 `state=PAUSED`로 바뀌었음에도:
  - `message`에 "Paused"가 아니라 `실행 중: JReady로 이동` 같은 **다음 step**의 task 정보가 표시됨
  - `progress`가 한 칸 더 올라감
  - 로봇이 현재 진행 중이던 모션을 **끝까지 수행**한 뒤에야 멈춤
- 그 이후의 모션은 제대로 멈춤 (즉, pause 자체는 "늦게" 먹히지만 먹히긴 함)

### 근본 원인: Step 경계(step→amovel) gap race

Task의 각 step은 세 줄로 구성됩니다.

```python
if not step(N, "name", "desc"): return ...   # ① check_stop + update_progress
amovel(pos, vel, acc)                         # ② 새 비동기 모션 전송
if not ctx.wait_motion(): return ...          # ③ 모션 완료 대기
```

기존 Issue 1 수정으로 `wait_motion` 은 `pause_in_flight` / `PAUSED` 를 체크해 대기하지만, **step N 의 `wait_motion` 이 반환된 직후 ~ step N+1 의 `amovel` 호출 사이**의 μs 단위 gap 에 pause 요청이 들어오면 다음 일이 벌어집니다.

```
T0: Step N의 wait_motion() → motion_done=True, return True
T1: Task thread가 Python 다음 줄로 이동 (gap)
T2: [USER PAUSE]
     - pause_in_flight = True
     - MovePause → DSR 전송
     - DSR: 진행 중인 모션 없음 → success=True 반환 (pause 할 게 없음!)
T3: Task thread: step(N+1) → update_progress
     - message = "실행 중: step N+1"     ← "Paused" 를 덮어씀
     - current_step +=1                  ← progress 가 한 칸 올라감
T4: Task thread: amovel(N+1) → DSR 에 새 모션 전송 → 로봇 물리적 이동 시작!
T5: MovePause 콜백 도착 → state=PAUSED, message="Paused"
T6: Task thread: wait_motion(N+1) → state=PAUSED 감지 → 대기
     - 하지만 DSR 은 새 모션을 진행 중 (T4 에서 시작)
     - DSR 은 이미 완료된 이전 모션 기준으로 "pause 할 게 없었음"
T7: 로봇이 step N+1 모션 끝까지 수행 후 정지
     - UI 상으로는 이미 state=PAUSED, message="Paused"
     - wait_motion 은 계속 대기 → 그 뒤 step 들은 제대로 멈춤
```

**핵심 결함 2 가지:**

1. `update_progress` 가 pause 상태를 전혀 체크하지 않고 `message` / `current_step` 을 덮어씀
2. Step 경계에 pause 방어벽이 없음 → `wait_motion` 은 이미 시작된 모션만 지킬 수 있고, 아직 issue 되지 않은 다음 `amovel` 은 막지 못함

### 수정 내용: `update_progress` 를 pause gate 로 활용

`update_progress` 는 각 step 의 시작 지점(= `amovel` 직전)에서 호출됩니다. 여기서 pause 를 흡수하면 다음 `amovel` 자체가 issue 되지 않아 위 race 가 완전히 닫힙니다.

```python
def update_progress(self, step, total, name, message=""):
    """Step 시작 지점에서 호출. Pause 중이면 Resume 까지 block."""
    while True:
        with self._shared.lock:
            if self._generation != self._shared.task_generation:
                return
            if self._shared.stop_requested:
                return
            should_wait = (
                self._shared.pause_in_flight or
                self._shared.state == ExecutorState.PAUSED
            )
            if not should_wait:
                # 평상시: 진행 정보 업데이트 후 반환
                self._shared.current_step = self._shared.module_step_offset + step
                self._shared.step_name = name
                if message:
                    self._shared.message = message
                return
        time.sleep(0.05)
```

아울러 `wait_motion` 도 `pause_in_flight` 를 체크하도록 보강해, pause 콜백이 도착하기 전(상태 전이 전)이더라도 task 가 즉시 대기에 들어가도록 했습니다.

```python
def wait_motion(self) -> bool:
    motion_done = False
    while True:
        if self.check_stop():
            return False
        with self._shared.lock:
            should_wait = (
                self._shared.pause_in_flight or                 # ← 추가
                self._shared.state == ExecutorState.PAUSED
            )
        if should_wait:
            time.sleep(0.05)
            continue
        if motion_done:
            return True
        try:
            status = check_motion()
        except Exception:
            status = 1
        if status == 0:
            motion_done = True
            continue
        time.sleep(0.05)
```

### 수정 후 동작

```
T0: Step N wait_motion → return True
T1: [USER PAUSE] pause_in_flight = True, MovePause → DSR
T2: Task thread: step(N+1) → update_progress
     └── pause_in_flight=True 감지 → sleep 0.05 loop (block)
T3: MovePause 콜백 → state=PAUSED, pause_in_flight=False, message="Paused"
T4: update_progress: state=PAUSED → 계속 block (amovel 호출 안 됨!)
T5: [USER RESUME] → state=RUNNING
T6: update_progress: should_wait=False → 진행 정보 업데이트 후 반환
T7: amovel(N+1) → 이제 정상 시작
```

### 이 수정이 커버하는 범위


| 시점  | pause 수신 위치                                        | 기존 동작                           | 수정 후                                                                     |
| --- | -------------------------------------------------- | ------------------------------- | ------------------------------------------------------------------------ |
| A   | Step N의 `wait_motion` 루프 중                         | ✓ 정상 (DSR MovePause 가 현재 모션 정지) | ✓ 동일                                                                     |
| B   | Step N의 `wait_motion` 반환 ~ Step N+1 `step()` 호출 사이 | ✗ 다음 모션이 끝까지 진행                 | ✓ `update_progress` 가 block 하여 `amovel` 호출 방지                            |
| C   | Step N+1의 `update_progress` 실행 중                   | ✗ (위와 동일)                       | ✓ block 내에서 pause 감지                                                     |
| D   | `update_progress` 반환 ~ `amovel` 호출 사이 (μs)         | —                               | 거의 불가능한 gap. 설령 발생해도 `amovel` 직후 `MovePause` 가 DSR 에 도달 → DSR 이 정상 pause |
| E   | `amovel` 호출 직후 ~ `wait_motion` 진입 전                | ✓ 정상                            | ✓ 동일                                                                     |


B, C 가 사용자가 보고한 증상의 원인이고, 이번 수정으로 해결됩니다.

### 왜 "현재 모션은 끝까지, 다음 모션부터는 멈춤" 현상인가 (사용자 관찰과의 정합성)

- Pause 가 늦게 (step 경계에) 들어오면:
  - 기존: "진행 중인 step (= step N+1) 이 끝까지 진행" (사실은 방금 issue 된 새 모션)
  - 그다음 step 들은 pause 상태가 이미 설정되어 있으므로 `wait_motion` 이 잡음 → 안 돌아감
- Pause 가 일찍 (모션 진행 중) 들어오면:
  - DSR MovePause 가 현재 모션을 중간에 정지 → "진행 중 모션도 즉시 멈춤" (정상 동작)
- 사용자가 관찰한 현상은 전자에 해당합니다. 이번 수정은 전자의 gap 을 제거합니다.

### 관련 파일

- `cobot1/motion_executor.py` (`wait_motion`, `update_progress`)

---

## Issue 3: 그리퍼 안전 동작 문제 (중요!)

### 증상

- SAFE_STOP 또는 EMERGENCY 상태에서 그리퍼가 자동으로 여닫힘
- 서보가 꺼진 상태(SAFE_OFF)인데 그리퍼가 움직임

### 원인 분석

`task_tong.py`, `task_source.py`의 `_gripper()` 함수에 stop/fault 체크가 없었습니다.

```python
# 수정 전 (버그)
def _gripper(action: str):
    from DSR_ROBOT2 import set_digital_output, wait
    for ch in (1, 2, 3, 4):
        set_digital_output(ch, 0)  # stop 체크 없이 실행!
    wait(0.1)
    ch = _TONG_DO_MAP[action]
    set_digital_output(ch, 1)  # stop 체크 없이 실행!
    wait(1.5)
```

**위험 시나리오:**

1. Task 실행 중 모션 완료
2. `_gripper("GRIP_TIGHT")` 시작
3. 외력 감지 → SAFE_STOP 진입
4. `_gripper()`는 stop 체크 없이 계속 실행
5. 그리퍼가 세게 닫힘 → **작업자 부상 위험!**

### 수정 내용

#### 1. 안전 유틸리티 함수 추가 (`_common.py`)

```python
def safe_wait(ctx, duration_sec: float, interval: float = 0.05) -> bool:
    """인터럽트 가능한 대기."""
    elapsed = 0.0
    while elapsed < duration_sec:
        if ctx.check_stop():
            return False
        time.sleep(min(interval, duration_sec - elapsed))
        elapsed += interval
    return True

def safe_digital_output(ctx, channel: int, value: int) -> bool:
    """안전한 digital output 설정."""
    if ctx.check_stop():
        return False
    from DSR_ROBOT2 import set_digital_output
    set_digital_output(channel, value)
    return True
```

#### 2. `_gripper()` 함수 수정

- `ctx` 파라미터 추가 (하위 호환을 위해 optional)
- 각 단계에서 stop 체크
- stop 요청 시 그리퍼를 열린 상태로 두고 반환 (안전)

```python
def _gripper(action: str, ctx=None):
    if ctx is None:
        # 기존 동작 (하위 호환)
        ...
        return True

    # Step 1: 모든 채널 리셋 (stop 체크)
    for ch in (1, 2, 3, 4):
        if not safe_digital_output(ctx, ch, 0):
            return False

    # Step 2: 짧은 대기 (인터럽트 가능)
    if not safe_wait(ctx, 0.1):
        return False

    # Step 3: 목표 채널 설정 (stop 체크)
    if not safe_digital_output(ctx, ch, 1):
        set_digital_output(RELEASE_CH, 1)  # 비상 열기
        return False

    # Step 4: 대기 (인터럽트 가능)
    if not safe_wait(ctx, 1.5):
        set_digital_output(RELEASE_CH, 1)  # 비상 열기
        return False

    return True
```

### 관련 파일

- `cobot1/tasks/_common.py` - `safe_wait()`, `safe_digital_output()` 추가
- `cobot1/tasks/task_tong.py` - `_gripper()` 수정
- `cobot1/tasks/task_source.py` - `_gripper()` 수정

### 테스트

- `test/test_gripper_safety.py`

---

## 추가 개선: 서비스 폴링 기본값 변경 (Issue 4 관련)

### 배경

`ros2 launch cobot1 task_system.launch.py` 실행 시 초기에 서비스 통신이 불안정한 경우가 있었습니다. 4개 서비스(robot_state, posj, tool_force, ext_torque)를 동시에 10Hz로 폴링하면서 ROS2 discovery 시간 동안 부하가 생길 수 있습니다.

### 수정 내용

- `poll_posj`, `poll_tool_force`, `poll_ext_torque` 기본값을 `false`로 변경
- 필수인 `robot_state`만 기본 폴링
- 추가 기능(pos_log 등) 구현 시 필요한 서비스를 `true`로 설정

### 사용 방법

```bash
# 기본 (robot_state만 폴링)
ros2 launch cobot1 task_system.launch.py

# 모든 서비스 폴링 (추가 기능용)
ros2 launch cobot1 task_system.launch.py poll_posj:=true poll_tool_force:=true poll_ext_torque:=true
```

---

## 테스트 실행 방법

```bash
cd /home/woody/cobot_ws/src/cobot1/test
python3 test_pause_resume_race.py -v
python3 test_gripper_safety.py -v
```

