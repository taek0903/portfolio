# TODO

## 원칙 (유지)

> 전체 시스템 아키텍처 다이어그램(데이터 흐름·노드 역할·payload 스키마 포함)은
> `docs/ARCHITECTURE.md` 참조.

- 디바이스1: `motion_executor` + `robot_status_publisher` 실행 / 디바이스2:
`task_controller` 실행. 개발 편의용으로 `task_system.launch.py` 단일 실행도
지원 (launch arg `executor` / `status` / `controller` 로 on/off).
- 역할 분리 (프로세스 3개):
  - **motion_executor**: DSR2 명령·태스크 실행 (DSR_ROBOT2 wrapper + task thread + start/stop 서비스)
  - **robot_status_publisher**: DSR2 monitoring 서비스 4종 폴링 → `robot_status` 토픽
  - **task_controller**: 외부 UI ↔ motion_executor 중계

## 완료

- **복구 패널 뱃지 이동 · resume progress auto_serving scale · raw status 패널 (2026-04-23)**
  - **복구 Phase1 패널 뱃지 위치 이동**: "중단된 작업 · 🍱 …" 뱃지를 `복구 시작하기`
  버튼 위 → **아래** 로 이동. 뱃지 유무에 따라 버튼 세로 좌표가 흔들리던 문제 해결.
  - **resume_from_* progress 를 auto_serving scale 로 보정**: `motion_executor` 에
  `_auto_serving_parent_skip_steps(task_name)` 헬퍼 추가. `resume_from_<mod>` 시
  건너뛴 auto_serving 모듈 (rice / rice+tong) 의 STEPS 합을 `total_steps` 와
  초기 `step_offset` 에 반영 → 퍼블리시되는 progress 가 자동으로 auto_serving
  전체 scale. 예) resume_from_tong 시작 시 rice 분량 % 로 출발, 완료 시 100%.
  `recovery_auto_serving` / 일반 task 는 offset 0 으로 동작 유지.
  - **Task Steps 우측 뱃지 "건너뜀" → "완료"**: resume_from_* 의 virtualDone
  모듈(이미 완료된 이전 모듈) 은 실제로는 "건너뛴" 것이 아니라 SAFE_STOP 이전에
  끝난 모듈이므로 뱃지를 "완료" 로 통일. emerald 체크 + strikethrough 는 기존
  완료 행과 동일.
  - **하단 "상세 정보" 패널 개편**: 기존 4칸 요약 카드(진행 중 작업 / 시스템 상태
  코드 / 마지막 완료 / 최근 에러) 를 제거하고, payload 의 `system_status` /
  `task_status` / `robot_status` 를 각각 한 컬럼씩 key/value rows 로 그대로 덤프.
  중첩 object/array 는 `formatRawValue` 로 한 줄 요약(JSON). 디버깅 편의성 향상.
  `rawStatus` state 가 5Hz 로 갱신.
- **재시작 sub-label · Toast X · serving count 제거 · resume_from 연속 표시 (2026-04-23)**
  - `app.js RESUME_DESC_FOR` 문구를 "마저 진행하기를 누르면 🍚 밥부터 이어서…"
  톤으로 재작성. 버튼 하단 안내 박스의 `재시작 시:` 접두어 제거. 단일 모듈 복귀
  (`recovery_<task>`) fallback 문구도 동일 톤으로 통일.
  - 상태 안내 Toast 우측에 **✕ 버튼** 추가. `dismissedToastKey` state 가
  `(systemState, recoveryPhase, connected)` 를 key 로 저장해, 같은 상태 동안에는
  숨김 유지하고 상태가 바뀌면 자동 재노출.
  - **배식 카운트(serving_count) feature 제거**: `ui_bridge.SERVING_COUNT_TASKS`
  집합, `_serving_count_today` 필드, payload `task_status.serving_count`*  필드,
  Firebase bootstrap 의 serving 집계, 자정 롤오버의 serving 리셋, complete 이벤트
  message 의 `({회차})` 접미사 전부 삭제. `today_errors` 는 유지. UI 우측 패널
  상단의 `오늘 배식 N회` 뱃지와 `servingCount` state 도 삭제.
  - *resume_from_ 실행 중 Task Steps 연속 표시**: `app.js` 에 `SKIPPED_BEFORE`
  상수 추가. `resume_from_tong` / `resume_from_sauce` 일 때 payload `taskModules`
  앞에 "건너뛴" 모듈(rice / rice·tong) 을 `__virtualDone` 플래그와 함께 prepend
  해 `[gripper_open(✓), rice(✓ 건너뜀), tong(▶), sauce(○)]` 처럼 auto_serving
  전체 흐름이 보인다. `activeModuleIndex` 는 `virtualOffset` 만큼 밀어 표시. 백엔드
  스키마/motion_executor 변경 없음.
  - 문서 반영: `docs/ARCHITECTURE.md` payload 스키마 / Firebase pull 경로 / UI
  구성 / Toast 섹션.
- **motion.launch + UI 헤더/토스트/복구 정비 (2026-04-23)**
  - `launch/motion.launch.py` 신설 — 디바이스1(로봇 근접)에서 `motion_executor` +
  `robot_status_publisher` 만 전용으로 기동. 기존 `task_system.launch.py controller:=false` 우회를 대체. Launch args: `robot_namespace`, `poll_posj`,
  `poll_tool_force`, `poll_ext_torque`.
  - **홈 버튼 헤더 이관**: Primary 영역 하단의 홈 버튼을 헤더 우측(Emergency 좌측)
  으로 이동. 컴팩트 스타일(`px-4 py-2.5`, 화이트 subtle). 노출 조건 `isIdle && connected` 유지 — PAUSED/EMERGENCY/SAFE_STOP/RECOVERY 에서는 숨김.
  - **Toast 레이아웃 간소화**: `title` + 우측 `hint` 박스 제거. `[icon] [EYEBROW / description(설명+가이드 합친 한 문장)]` 2단 구조로 단순화. `statusGuide` 객체 8개
  분기의 문구를 "발생 원인 + 다음 행동" 한 문장으로 재작성 (예: Safety Stop →
  "충돌이 감지되어 안전 모드가 활성화되었습니다. 좌측 복구 시작하기 버튼을 눌러
  복구해주세요.").
  - **복구 플로우 UI 정비**:
    - phase1: 상단 "안전 정지 / 복구 방법을 선택하세요" 라벨 제거 (토스트 담당).
    자동 복구 버튼 "자동 복구" → **"복구 시작하기"**.
    - manualPanel: 상단 "수동 복구 / 로봇을 안전한 위치로…" 라벨 제거.
    - phase2: 상단 "복구 완료 / 다음 동작을 선택하세요" 라벨 제거. **"취소" 버튼
    삭제**. 2열 버튼 순서 [`**마저 진행하기`**(emerald primary) | `홈으로`(white
    subtle, 헤더 홈 버튼과 동일 톤)]. `restartSubLabel` 뱃지는 유지.
    - `theme.RECOVERED` 아이콘 ✅ → **🛠** (툴 모양). CircularGauge 중앙에 표시.
  - **복구 구간 progress freeze 확대**: `PROGRESS_FREEZE_SYSTEM_STATES` Set 신설 —
  `SAFE_STOP` / `SAFE_STOP_2` / `EMERGENCY` / `RECOVERY` / `RECOVERED` 동안
  `setProgress` 차단. 복구 완료 후 "마저 진행하기/홈으로" 선택 화면에서도 중단 당시
  % 가 게이지에 그대로 유지. 다음 task 가 RUNNING 으로 전환되면 자동 재개.
  - 문서 반영: `docs/ARCHITECTURE.md` UI 구성(0-4) / Launch 매트릭스 · 파일 구조,
  `scripts/SETUP.md` 3.2(motion.launch.py) / 3.3 섹션 갱신.
- **UI Controller 정리 · Firebase 모드 필터 · device2 launch (2026-04-23)**
  - `web_ui/app.js` 헤더 재작성 — `ROBOT CONTROL DASHBOARD / System Status / Bridge / Current Task / Serving` 블록 전부 제거, `배식 로봇 대시보드` 타이틀
  한 줄만 유지. `sticky top-0 z-40 py-3` 로 스크롤에도 고정되는 얇은 헤더.
  Safety 버튼도 컴팩트(`min-w-[180px] py-2.5`).
  - Serving count 를 헤더 → 우측 **Task Steps** 패널 상단으로 이동. `오늘 배식 N회`
  뱃지 + `KST` 표기.
  - 홈 버튼 노출 조건을 `isIdle && connected` 로 좁힘. PAUSED/EMERGENCY/
  SAFE_STOP/RECOVERY 에서 disabled 홈 버튼이 중복 노출되던 문제 해결.
  - Primary/Recovery 버튼(시작·재개·일시정지·자동복구·수동복구·이동완료·홈으로·재시작)
  모두 `flex flex-row items-center justify-center gap-3` 수평 레이아웃 + 높이
  축소(`py-8 → py-5` / `py-6 → py-4`).
  - 보호모드·EMERGENCY·복구완료·연결끊김 안내 카드(우측 고정 카드)를 **상단 우측
  고정 Toast** 로 이관. 상태 지속 시 유지 → 해제되면 자동 사라짐(non-blocking).
  안내 문구 수정은 `statusGuide` 객체(=`app.js` `statusGuide = (() => ...)`)에서.
  - Hero 섹션 좌/우 비중 `xl:grid-cols-[1.15fr_0.85fr]` → `xl:grid-cols-[1.6fr_1fr]`
  로 좌측 우선 확대.
  - `ui_bridge.FirebaseRealtimeSync.fetch_today_events()` 에 `mode_filter` 파라미터
  추가. `_bootstrap_from_firebase` 가 `self._mode` 로 필터링 → virtual 실행
  중에는 virtual 이벤트만, real 실행 중에는 real 이벤트만 bootstrap.
  - `launch/ui.launch.py` 신설 — 디바이스2(UI/원격)에서 `task_controller` +
  `ui_bridge` + `rosbridge_websocket` 을 한 번에 기동. 기존 device1 은 여전히
  `task_system.launch.py executor:=true status:=true controller:=false` 로
  motion_executor + robot_status_publisher 만 실행.
  - 문서 반영: `docs/ARCHITECTURE.md` Launch 시나리오·UI 레이아웃 섹션, `scripts/SETUP.md`
  단일 launch 안내 갱신.
- **파일 구조 정리 · Firebase 당일 pull · UI 개편 (2026-04-23)**
  - `cobot1/cobot1/nodes/` 디렉토리 신설 — `motion_executor`, `robot_status_publisher`,
  `task_controller`, `ui_bridge`, `task_cli` 모두 하위로 이동. 기존
  `controller/` 디렉토리 제거. 레거시 `move.py` / `move_periodic.py` / `rice.py`
  삭제 (tasks/task_rice.py 로 대체됨). `setup.py` `entry_points` 일괄
  `cobot1.nodes.`* 로 갱신.
  - `ui_bridge.FirebaseRealtimeSync.fetch_today_events(since, type_filter, fetch_limit)` 추가 — admin SDK / REST 모두 지원. 기동 시 KST 0시 이후 이벤트를
  pull 해 `serving_count_today` / `today_errors` bootstrap. 자정(KST) 롤오버는
  `_publish_status` 에서 날짜 변경 감지 후 리셋.
  - 배식 카운트 의미 변경: 누적 카운터 → **당일(KST) 카운터**. payload 에
  `task_status.serving_count_today` 필드 추가, 기존 `serving_count` 는 같은
  값으로 유지(호환).
  - payload 에 `task_status.modules` / `today_errors` 신설. `TASK_MODULES` 매핑은
  `motion_executor.TASK_REGISTRY` 와 수동 동기.
  - `web_ui/app.js`:
    - Standby 아이콘을 체크(✅) → `⋯` + animate-pulse 로 교체.
    - 우측 Status Guide + Info Cards 2x2 제거, **Task Steps** 불렛 리스트로
    대체 (완료/진행/남음 마커·스타일 차등). 비정상 상태에서만 Status Guide
    보조 카드 노출.
    - 하단에 **오늘의 에러** 패널 추가 — `payload.today_errors` 기반. 새로고침
    후에도 ui_bridge 가 Firebase 에서 다시 bootstrap.
  - Progress bar 검토 완료: `total_steps` 합산 / `module_step_offset` 누적 로직이
  모듈 전환·pause/resume·resume_from_* 재시작 모두에서 기대대로 동작 → 코드
  수정 없음.
  - `docs/ARCHITECTURE.md` 에 새 파일 구조, Firebase pull 경로(0-3), UI 구성
  (0-4), 갱신된 payload 스키마 반영.
- **auto_serving 중단 후 재시작 — 이후 모듈까지 연속 실행 + UI 표시 (2026-04-22)**
  - 증상: `auto_serving` 중 tong/sauce 에서 SAFE_STOP 발생 후 "재시작" 을 누르면
  `recovery_tong` / `recovery_sauce` 가 호출되어 해당 단일 모듈만 실행하고 끝나버림
  (남은 sauce 가 진행되지 않음).
  - 원인: `motion_executor.TASK_REGISTRY` 의 `recovery_<module>` 이 `[gripper_open, <module>]` 단일 모듈만 포함. 단독 실행된 rice/tong/sauce 복귀에는 맞지만, 
  auto_serving 연속 배식 중단 복귀에는 부족.
  - 수정:
    - `motion_executor.TASK_REGISTRY` 에 `resume_from_rice / resume_from_tong / resume_from_sauce` 추가. 중단 모듈부터 sauce 까지 연속 실행.
    - `ui_bridge.ACTION_TO_TASK` / `ACTION_LABELS` / `SERVING_COUNT_TASKS` 에
    신규 task 등록. resume_from_* 완료 시 배식 1회로 카운트.
    - `app.js`: SAFE_STOP 진입 시 `interruptedTaskRef` 를 `{root, module}` 객체로
    캡처 (root=auto_serving/tong/rice/sauce/home, module=실제 중단 서브모듈).
    `resolveResumeTask(root, module)` 헬퍼로 실행할 task 결정:
      - root=auto_serving & module∈{rice,tong,sauce} → `resume_from_<module>`
      - 그 외 (단일 task 중단) → `recovery_<root|module>`
    - Phase1 안전 정지 패널에 "중단된 작업 · 🍱 샐러드·돈까스 (자동 배식 중)" 뱃지.
    - Phase2 복구 완료 패널의 재시작 버튼 하단에 "재시작 시: 🍱 집게부터 이어서
    (샐러드·돈까스 → 소스)" 문구 표시 — 사용자가 누르기 전에 어떤 범위가 실행될지
    명확히 인지 가능.
- **복구 플로우 그리퍼 자동 해제 + Standby 긴급정지 + 서브태스크 UI 표시 +
SAFE_STOP 그리퍼 진동/자동열림 수정 (2026-04-22)**
  - 자세한 내용: [recovery-gripper-emergency-subtask.md](./recovery-gripper-emergency-subtask.md)
  - `task_gripper_open.py` 신규 — DO 로직을 새로 정의하지 않고 `task_rice._gripper("RELEASE", ctx)`
  에 위임 (하드웨어에 DO4 없음; rice 의 초기 release 로직을 단일 진실점으로 사용).
  모든 외부 진입 task (home / rice / tong / sauce / auto_serving) 의 첫 모듈로 합성.
  - **SAFE_STOP 중 그리퍼 진동 방지**: `motion_executor` 가 `motion_executor/robot_status`
  를 구독하도록 추가. 하드웨어가 fault state(EMERGENCY/SAFE_OFF/SAFE_STOP/SAFE_OFF2/
  SAFE_STOP2) 로 진입하면 `stop_requested=True` 세팅해 task thread 의 잔여 STEPS
  (특히 `_gripper`) 실행을 즉시 차단. DSR 은 SAFE_STOP 중 모션만 막고 DO 는 그대로
  통과시키며 check_motion 이 즉시 idle 반환해서 STEPS 루프가 폭주하던 버그.
  - **Holding failsafe**: `_gripper()` (rice/tong/sauce) 의 safe_wait 실패 분기에서
  "안전을 위해 RELEASE DO 강제 세팅" 로직 제거. SAFE_STOP 진입 시 그리퍼는 현재
  상태를 그대로 유지 (payload 낙하 방지 = 산업 표준 fail-hold). 명시적 해제는
  복구 시 `task_gripper_open` (recovery_* 의 첫 모듈) 이 담당.
  - `TASK_REGISTRY` 확장: `recovery_home`, `recovery_rice`, `recovery_tong`,
  `recovery_sauce`, `recovery_auto_serving` — 2단계 복구 UI 와 1:1 매핑.
  - UI 재시작 버튼: 중단된 서브모듈(`module_name`) 기반 `recovery_<module>` 로
  복귀. 예) auto_serving 중 tong 에서 멈추면 recovery_tong 만 실행 (rice/sauce skip).
  - **Standby 긴급정지 버그 수정**: `ui_bridge._handle_safety_cmd` 및
  `task_controller._stop_callback` 이 `STATE ∈ {RUNNING, PAUSED}` 일 때만
  EMERGENCY 를 forward 하던 것을 task_state 무관하게 항상 forward 하도록 수정.
  STANDBY/IDLE 상태에서도 비상 버튼 → 실제 servo_off 동작.
  - **서브태스크 UI 표시**: `TaskState.msg` 에 `current_module_name` /
  `current_module_label` / `module_index` / `module_total` 추가. 각 task 모듈에
  `TASK_LABEL` 상수 추가. motion_executor → task_controller → ui_bridge → UI
  까지 forward. CircularGauge 중앙에 현재 서브태스크 아이콘/라벨 표시 (밥 🍚 /
  샐러드·돈까스 🍱 / 소스 🍯 / 그리퍼 해제 🖐). tong 내부는 step_name 접두어로
  "샐러드" / "돈까스" phase tag 표시 (돈까스 1/2 는 "돈까스" 하나로 통합).
- **외력 감지 시 UI 대응 (2026-04-21)**
  - `task_controller`: `RobotStateEnum` 확장 (SAFE_STOP2, SAFE_OFF2 등), 
  안전 정지 진입 시 TaskState → ERROR 전환, 진행률 동결
  - `ui_bridge`: 안전 정지 감지 시 `op_state=IDLE`, `robot_status` 문구 변경,
  명령 차단, 진행률 동결
  - `app.js`: `robot.state_name` null 처리, 서비스 경로 `dsr01` 분리,
  `ACTION_TO_TASK` 매핑 추가
- **SAFE_STOP/SAFE_OFF SW 복구 시스템 (2026-04-21)**
  - DSR2 `set_robot_control` 서비스 기반 상태별 복구 경로 구현
  - `StopTask.srv`: `TO_STANDBY(6)`, `TO_RECOVERY(7)`, `RECOVERY_DONE(8)` 추가
  - `motion_executor`: `_handle_to_standby`, `_handle_to_recovery`, 
  `_handle_recovery_done` 핸들러 구현
  - 상세 문서: [robot-state-recovery.md](./robot-state-recovery.md)
- **2단계 복구 UI (2026-04-21)**
  - `app.js v4.2`: 2-Phase Recovery UI 구현
    - Phase 1: 복구 방법 선택 (자동 복구 / 수동 복구)
    - Phase 2: 복구 완료 후 후속 동작 선택 (홈으로 / 작업 재시작)
    - RECOVERY(8) 상태 감지 시 "이동 완료" 버튼 표시
  - `ui_bridge`: RECOVERY 상태 감지 시 별도 UI 상태로 구분
  - CLI 테스트 완료 (ros2 service call)
- **통합 시스템 상태 (SystemState) (2026-04-21)**
  - Task 상태 + Robot 상태를 조합한 통합 상태 정의 (0~32)
  - `ui_bridge.py`: `SystemState` 클래스, `_compute_system_state()` 메서드
  - `app.js v4.3`: `SYSTEM_STATE` 상수, `STATUS_CONFIG` 기반 UI 단순화
  - payload에 `system_status`, `system_state_name`, `system_state_label` 필드 추가
  - 상태 구분: IDLE(0), WORKING(1), PAUSED(2), SAFE_STOP(10), SAFE_STOP_2(11),
  EMERGENCY(12), RECOVERY(13), RECOVERED(20), ERROR(30), NOT_READY(31), DISCONNECTED(32)
  - 상세 문서: [robot-state-recovery.md](./robot-state-recovery.md#통합-시스템-상태-systemstate)
- **UI 긴급 정지 → 실제 서보 차단 (2026-04-21)**
  - UI "비상 정지" 버튼 → task 종료 + servo off 구현
  - `motion_executor.py`: `_handle_emergency_stop()` 핸들러 추가
    - `move_stop(stop_mode=1, QUICK)` — 현재 모션 즉시 정지
    - `ServoOff(stop_type=1, QUICK)` — 실제 서보 OFF (빨간 불)
  - 복구 시 `TO_STANDBY(6)` 사용 (set_robot_control + set_robot_mode(AUTONOMOUS))

## 남은 작업

*계획된 작업은 모두 "완료" 섹션으로 이동. 새 요구사항은 여기 추가.*

- **우측 TASK STEPS 세부 스텝 (추후 검토)**
  - 밥/돈까스·샐러드/소스 하위의 더 세부 스텝을 진행 중/남은 것까지 표시.
  - 현재 payload 에는 `current_step_name` 만 있고 모듈별 STEPS 리스트 없음 →
  motion_executor 에서 active 모듈의 STEPS 를 payload 로 내려줘야 함.
  scope/UX 결정 후 진행.
- **실기 검증 (DRCF)**: 가상 에뮬레이터에서는 `robot_state` 가 `STANDBY(1)`
에서 전이되지 않아 `MOVING` / `SAFE_STOP` 반응 검증 불가. 실제 로봇에서
Firebase pull / today_errors / progress bar / mode 필터 흐름을 한 번 관찰 필요.

## 실행/검증 가이드

### 빌드

```bash
cd ~/cobot_ws
colcon build --packages-select cobot_interfaces cobot1 --symlink-install
source install/setup.bash
```

### 복구 시스템 CLI 테스트

```bash
# 자동 복구 (SAFE_* → STANDBY)
ros2 service call /dsr01/task/stop cobot_interfaces/srv/StopTask "{stop_type: 6}"

# 수동 복구 진입 (SAFE_*2 → RECOVERY)
ros2 service call /dsr01/task/stop cobot_interfaces/srv/StopTask "{stop_type: 7}"

# 수동 복구 완료 (RECOVERY → STANDBY)
ros2 service call /dsr01/task/stop cobot_interfaces/srv/StopTask "{stop_type: 8}"

# 상태 확인
ros2 run cobot1 task_cli status
```

### 단일 머신 (전체 실행)

```bash
# 터미널 1: motion_executor + robot_status_publisher + task_controller
ros2 launch cobot1 task_system.launch.py

# 터미널 2: ui_bridge + rosbridge_websocket (task_controller 는 위에서 이미 기동)
ros2 run cobot1 ui_bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 분산 실행 (디바이스1 = 로봇, 디바이스2 = UI)

```bash
# 디바이스1 (로봇 근접) — motion_executor + robot_status_publisher
ros2 launch cobot1 motion.launch.py

# 디바이스2 (UI/원격) — task_controller + ui_bridge + rosbridge 를 한 번에
ros2 launch cobot1 ui.launch.py mode:=real      # 실기
ros2 launch cobot1 ui.launch.py mode:=virtual   # 가상 에뮬레이터
```

### RobotStatus 토픽 확인

```bash
ros2 topic echo /dsr01/motion_executor/robot_status --field robot_state
ros2 topic echo /dsr01/motion_executor/robot_status --once
```

