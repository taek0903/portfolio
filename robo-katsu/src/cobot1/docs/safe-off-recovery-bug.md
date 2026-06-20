# SAFE_OFF(state=3) 복구 실패 + 긴급 정지 차단 버그

> 상태: **수정 적용 완료 (2026-04-24)**  
> 관련 커밋: `motion_executor._handle_to_standby` SAFE_OFF fallback,
> `ui_bridge._handle_safety_cmd` DSR 직접 호출.  
> 아래 "수정 사항" 은 원래 설계 메모. 실제 반영 코드는 "적용 내역" 절 참조.

## 재현 조건

1. 로봇이 2계열 안전 정지(SAFE_STOP2=9)로 진입 (노란색)
2. DSR 펌웨어가 자동으로 SAFE_STOP2(9) → SAFE_OFF(3) 전이 (빨간색)
3. UI에서 "복구 시작하기" 클릭 (stop_type=6, TO_STANDBY)
4. **에러**: "STANDBY 전환 실패 (현재 state=3)"
5. 이 상태에서 "긴급 정지" 버튼도 작동 안 함

## 근본 원인

### 버그 A — SAFE_OFF(3)에서 1계열 복구만 시도

`motion_executor._handle_to_standby()` 루프 (motion_executor.py L854-858) 에서
`state == ROBOT_STATE_SAFE_OFF(3)` 이면 무조건 `CONTROL_RESET_SAFET_OFF(3)` 만 시도.

그런데 DSR 펌웨어는 **진입 경로를 기억**한다.
2계열(SAFE_STOP2)에서 자동 전이된 SAFE_OFF(3)은 1계열 해제 명령을 **거부**하고
2계열 해제 명령(`CONTROL_RECOVERY_SAFE_OFF=5`)을 요구한다.

코드는 `robot_state` 숫자만 보고 진입 경로를 모르므로 → 계속 틀린 명령 →
같은 state 반복 → `same_state_count >= 2` → 실패.

### 버그 B — 복구 요청이 서비스 콜백을 20초간 점유

`_handle_to_standby`가 서비스 콜백(`_on_stop`) 안에서 최대 20초간
`time.sleep` 루프를 돌며 블로킹.

긴급 정지 요청(stop_type=2)도 같은 서비스(`/dsr01/task/stop`)를 쓰므로:
- rosbridge 레벨: 같은 서비스에 대한 두 번째 호출이 첫 번째 응답까지 큐잉 (ROSLIB 제약)
- motion_executor 레벨: `execute_task/stop` 서비스가 복구 요청에 점유돼 디스패치 지연

결과: 복구 실패 후 최소 20-25초가 지나야 긴급 정지가 도달할 수 있고,
사용자에게는 "아무 버튼도 안 먹힘"으로 보인다.

## 수정 사항

### 수정 1 — SAFE_OFF에서 1계열 실패 시 2계열 fallback (motion_executor.py)

`_handle_to_standby` 루프에서 `state == SAFE_OFF(3)` 일 때,
`CONTROL_RESET_SAFET_OFF(3)` 시도 후 같은 state가 유지되면
다음 iteration에서 `CONTROL_RECOVERY_SAFE_OFF(5)` 를 시도하는 fallback 분기 추가.

```python
# 수정 방향 (의사코드) — motion_executor.py _handle_to_standby 루프 내
elif state == ROBOT_STATE_SAFE_OFF:
    if same_state_count == 0:
        # 첫 시도: 1계열 해제
        control = CONTROL_RESET_SAFET_OFF       # 3
    else:
        # 1계열 실패 → 2계열 경유 시도
        control = CONTROL_RECOVERY_SAFE_OFF     # 5
```

### 수정 2 — 긴급 정지 경로를 복구 파이프라인에서 독립 (ui_bridge.py)

ui_bridge의 `_handle_safety_cmd`에서 EMERGENCY_STOP 시,
task_controller 경유(stop_type=2) 대신 motion_executor의 DSR 서비스를 **직접 호출**.

```python
# 수정 방향 — ui_bridge.py
# __init__에 client 추가:
self._servo_off_cli = self.create_client(ServoOff, f"/{ROBOT_ID}/system/servo_off")
self._move_stop_cli = self.create_client(MoveStop, f"/{ROBOT_ID}/motion/move_stop")

# _handle_safety_cmd EMERGENCY_STOP 분기에서:
# 1) move_stop(QUICK) 직접 호출
# 2) servo_off 직접 호출
# 3) 기존 task_controller 경유도 유지 (상태 정리용, fire-and-forget)
```

이렇게 하면 복구 요청이 서비스를 점유 중이어도 긴급 정지는 즉시 실행됨.

### 수정 우선순위

| 순위 | 수정 | 효과 | 난이도 |
|---|---|---|---|
| 1 | 수정 1: SAFE_OFF fallback | state=3 복구 실패 해결 | 낮음 |
| 2 | 수정 2: servo_off 직접 호출 | 긴급 정지 차단 해결 | 중간 |

## 적용 내역 (2026-04-24)

### motion_executor.py — `_handle_to_standby` SAFE_OFF 분기

`same_state_count` 를 이용한 1계열 → 2계열 fallback 적용.
루프는 기존대로 `same_state_count >= 2` 에서 실패 처리되므로, 두 계열을 모두
시도한 뒤에만 실제로 실패가 반환된다.

```python
elif state == ROBOT_STATE_SAFE_OFF:
    if same_state_count == 0:
        control, label = (
            CONTROL_RESET_SAFET_OFF,
            "SAFE_OFF → STANDBY (1계열 해제)",
        )
    else:
        control, label = (
            CONTROL_RECOVERY_SAFE_OFF,
            "SAFE_OFF → RECOVERY (2계열 fallback)",
        )
```

`changed_states` 는 원래부터 `ROBOT_STATE_RECOVERY` 를 포함하므로 2계열 fallback
호출 후 RECOVERY(8) 로 전이되는 경우도 정상적으로 관측된다. 이어지는 iteration
에서 기존 RECOVERY 분기(`CONTROL_RESET_RECOVERY=7`)가 STANDBY 까지 마무리한다.

진입 경로별 전이 정리:

```
1계열: SAFE_STOP(5) ──(2)──► SAFE_OFF(3) ──(3)──► STANDBY(1)
2계열: SAFE_STOP2(9) ──(자동)──► SAFE_OFF(3) ──(5)──► RECOVERY(8) ──(7)──► STANDBY(1)
```

### ui_bridge.py — EMERGENCY_STOP 독립 경로

`dsr_msgs2.srv` 의 `MoveStop` / `ServoOff` 를 임포트하고 ui_bridge 에서 직접 DSR
서비스를 호출하는 클라이언트를 추가. EMERGENCY_STOP 수신 시 task_controller
경유 호출은 그대로 두되(태스크 상태 정리용), DSR 서비스에 동시 쏘아 복구
점유와 무관하게 즉시 반응하도록 함.

```python
from dsr_msgs2.srv import MoveStop, ServoOff

# __init__
self._dsr_move_stop_cli = self.create_client(
    MoveStop, f'/{robot_ns}/motion/move_stop', callback_group=self._cb_group,
)
self._dsr_servo_off_cli = self.create_client(
    ServoOff, f'/{robot_ns}/system/servo_off', callback_group=self._cb_group,
)

# _handle_safety_cmd EMERGENCY_STOP 분기
if self._dsr_move_stop_cli.service_is_ready():
    mv_req = MoveStop.Request(); mv_req.stop_mode = 1  # ST_QUICK
    self._dsr_move_stop_cli.call_async(mv_req)
if self._dsr_servo_off_cli.service_is_ready():
    so_req = ServoOff.Request(); so_req.stop_type = 1  # QUICK
    self._dsr_servo_off_cli.call_async(so_req)
# task_controller 경유 호출(IDLE 전이)도 fire-and-forget 유지
```

의존성: `cobot1/package.xml` 은 이미 `<depend>dsr_msgs2</depend>` 를 선언하고
있어 추가 변경 없이 빌드된다.

### 검증 시나리오

1. **2계열 SAFE_OFF 복구**: 외력/보호정지 등으로 SAFE_STOP2(9) → SAFE_OFF(3)
   자동 전이 상태에서 UI "자동 복구" 클릭. 로그에
   `SAFE_OFF → STANDBY (1계열 해제)` (실패) → `SAFE_OFF → RECOVERY (2계열 fallback)` →
   `RECOVERY → STANDBY` 순서로 찍히고 최종 STANDBY(1) 도달.
2. **복구 중 긴급 정지**: 복구(stop_type=6) 가 진행되는 도중(최대 ~20s 루프)
   UI 비상정지 버튼 누름. 기다림 없이 즉시 서보 OFF(빨간불)로 전환. ui_bridge
   로그에 `EMERGENCY: move_stop(QUICK) sent directly to DSR` /
   `EMERGENCY: servo_off(QUICK) sent directly to DSR` 확인.
3. **STANDBY 긴급 정지 회귀 미발생**: 작업이 돌지 않는 STANDBY 에서도 비상정지
   버튼이 계속 작동 (기존 `_handle_emergency_subtask` 기능 유지).

## 관련 코드 위치

- `motion_executor.py` L854-858: SAFE_OFF 분기 (수정 1 대상)
- `motion_executor.py` L815-904: `_handle_to_standby` 전체 루프
- `motion_executor.py` L605-635: `_handle_emergency_stop`
- `task_controller.py` L358-454: `_stop_callback` (서비스 프록시)
- `ui_bridge.py` L1102-1142: `_handle_safety_cmd`
- `app.js` L861-866: `handleAutoRecovery`

## 참고: DSR 상태 전이 (실측)

```
1계열:
  SAFE_STOP(5) ──(2)──► SAFE_OFF(3) ──(3)──► STANDBY(1)

2계열:
  SAFE_STOP2(9) ──(자동 전이)──► SAFE_OFF(3)
                                    │
                                    ├─ (3) CONTROL_RESET_SAFET_OFF → ❌ DSR 거부
                                    └─ (5) CONTROL_RECOVERY_SAFE_OFF → RECOVERY(8) ──(7)──► STANDBY(1)
```

## 참고: DSR SAFE_STOP vs SAFE_OFF 차이

- **SAFE_STOP (노란색)**: 소프트웨어 브레이크. 서보 ON, 위치 유지.
- **SAFE_OFF (빨간색)**: 서보 전원 차단. 모터 힘 풀림.
- SAFE_STOP2(9)에서 SAFE_OFF2(10)가 아닌 SAFE_OFF(3)으로 전이되는 이유:
  DSR 펌웨어가 "서보 차단" 상태를 SAFE_OFF(3)으로 통합하기 때문.
  SAFE_OFF2(10)는 별도의 보호 정지 서보 오프 경로(Category 0 Stop 등)에서만 사용.
  2계열 에스컬레이션은 일반 SAFE_OFF(3)으로 내려가지만, DSR 내부적으로는
  진입 경로를 기억해 해제 명령을 구분한다.
