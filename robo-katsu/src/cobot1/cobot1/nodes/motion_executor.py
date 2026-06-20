"""
Motion Executor Node (Two-node architecture, slim)

rclpy Executor와 DSR_ROBOT2 Python wrapper의 공존 문제 해결을 위해 **프로세스
하나에 노드 두 개**를 운용한다. 배경·원인·설계 근거는
docs/rclpy-executor-dsr2-troubleshooting.md 참조.

- control_node (MultiThreadedExecutor에 add):
    ROS 외부 I/F (StartTask/StopTask 서비스, TaskState 토픽) +
    DSR2 명령용 클라이언트 (MoveStop/Pause/Resume).
- dsr_node (어떤 executor에도 add하지 않음):
    DR_init.__dsr__node로 등록. DSR_ROBOT2 함수가 호출될 때만 글로벌
    SingleThreadedExecutor가 잠깐 가져간다.
- task thread (단 하나):
    DSR_ROBOT2 함수를 직접 호출. control_node의 shared state를 Lock으로 공유.

## 상태 폴링은 별도 프로세스로 분리

`GetRobotState` / `GetCurrentPosj` / `GetToolForce` / `GetExternalTorque` 를
폴링해 `motion_executor/robot_status` 토픽을 발행하는 역할은 **별도 프로세스**
`robot_status_publisher` 가 담당한다 (cobot1/robot_status_publisher.py). 이 프로세스
안에서 폴링하면 DSR_ROBOT2 wrapper + MT executor + task thread 조합과 동일
rmw context 에서 경합해 응답을 못 받는 증상이 있었다. Troubleshooting 문서
§10 참조.
"""

import threading
import time
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import DR_init

from cobot_interfaces.msg import RobotStatus, TaskState
from cobot_interfaces.srv import StartTask, StopTask
from dsr_msgs2.srv import (
    MovePause,
    MoveResume,
    MoveStop,
    ServoOff,
    SetRobotControl,
    SetRobotMode,
    SetSafeStopResetType,
)


# SAFE_STOP 해제용 DSR2 상수
# SetRobotMode: 0=MANUAL, 1=AUTONOMOUS, 2=MEASURE
ROBOT_MODE_AUTONOMOUS = 1
# SetSafeStopResetType: 0=PROGRAM_STOP(=DEFAULT), 1=PROGRAM_RESUME
SAFE_STOP_RESET_TYPE_PROGRAM_STOP = 0

# SetRobotControl 상수 (SetRobotControl.srv 참조)
# 0: CONTROL_INIT_CONFIG (T/P only)
# 1: CONTROL_ENABLE_OPERATION (T/P only)
# 2: CONTROL_RESET_SAFET_STOP — SAFE_STOP(5) → STANDBY
# 3: CONTROL_RESET_SAFET_OFF — SAFE_OFF(3) → STANDBY
# 4: CONTROL_RECOVERY_SAFE_STOP — SAFE_STOP2(9) → RECOVERY
# 5: CONTROL_RECOVERY_SAFE_OFF — SAFE_OFF2(10) → RECOVERY
# 6: CONTROL_RECOVERY_BACKDRIVE — SAFE_OFF2 → RECOVERY (H/W, reboot 필요)
# 7: CONTROL_RESET_RECOVERY — RECOVERY(8) → STANDBY
CONTROL_RESET_SAFET_STOP = 2
CONTROL_RESET_SAFET_OFF = 3
CONTROL_RECOVERY_SAFE_STOP = 4
CONTROL_RECOVERY_SAFE_OFF = 5
CONTROL_RESET_RECOVERY = 7

# DSR robot_state enum (GetRobotState.srv / RobotStatus.msg 와 동일 매핑)
# ⚠️ 숫자 매핑 주의: 2 = MOVING, 6 = EMERGENCY_STOP. 과거 여기서 2 를
# EMERGENCY_STOP 으로 오표기해 로봇이 움직이기 시작하면(MOVING=2) task 가
# "EMERGENCY" 로 오인되어 즉시 중단되는 버그가 있었다.
ROBOT_STATE_INITIALIZING   = 0
ROBOT_STATE_STANDBY        = 1
ROBOT_STATE_MOVING         = 2
ROBOT_STATE_SAFE_OFF       = 3
ROBOT_STATE_TEACHING       = 4
ROBOT_STATE_SAFE_STOP      = 5
ROBOT_STATE_EMERGENCY_STOP = 6
ROBOT_STATE_HOMMING        = 7
ROBOT_STATE_RECOVERY       = 8
ROBOT_STATE_SAFE_STOP2     = 9
ROBOT_STATE_SAFE_OFF2      = 10
ROBOT_STATE_NOT_READY      = 15
ROBOT_STATE_UNKNOWN        = 255

# 하드웨어 레벨 안전 정지 상태. 이들 중 하나로 진입하면 task thread 를 즉시
# 중단시켜야 한다 (DSR 이 모션은 막지만 set_digital_output 은 계속 통과되어
# STEPS 리스트 잔여 _gripper() 호출이 연속 실행되면서 그리퍼가 진동하는 현상 방지).
ROBOT_FAULT_STATES = frozenset({
    ROBOT_STATE_EMERGENCY_STOP,
    ROBOT_STATE_SAFE_OFF,
    ROBOT_STATE_SAFE_STOP,
    ROBOT_STATE_SAFE_STOP2,
    ROBOT_STATE_SAFE_OFF2,
})

# ServoOff stop_type 상수 (ServoOff.srv 참조)
SERVO_OFF_STOP_TYPE_QUICK_STO = 0  # Safe Torque Off (가장 강력)
SERVO_OFF_STOP_TYPE_QUICK = 1
SERVO_OFF_STOP_TYPE_SLOW = 2
SERVO_OFF_STOP_TYPE_HOLD = 3  # = EMERGENCY


# ───────────────────── Robot Config ─────────────────────
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class ExecutorState(IntEnum):
    IDLE = 0
    RUNNING = 1
    STOPPING = 2
    ERROR = 3
    PAUSED = 4


class _SharedState:
    """control_node와 task_thread가 공유하는 상태. Lock으로 보호."""

    def __init__(self):
        self.lock = threading.Lock()
        self.state = ExecutorState.IDLE
        self.task_name = ""
        self.task_generation = 0
        self.module_index = 0
        self.module_total = 0
        self.module_step_offset = 0
        # 현재 실행 중인 서브태스크 모듈 (auto_serving 내부에서 rice/tong/sauce 전환 추적).
        # module_name 은 기계적 식별자(TASK_NAME), module_label 은 UI 표시용.
        self.module_name = ""
        self.module_label = ""
        self.current_step = 0
        self.total_steps = 0
        self.step_name = ""
        self.message = ""
        self.stop_requested = False
        self.pause_in_flight = False  # 중복 pause 요청 방지


class _TaskContext:
    """task 모듈의 run(ctx)에 주입되는 얇은 어댑터."""

    def __init__(self, shared: _SharedState, module_index: int, generation: int):
        self._shared = shared
        self._module_index = module_index
        self._generation = generation

    def _is_stale(self) -> bool:
        with self._shared.lock:
            return self._generation != self._shared.task_generation

    def check_stop(self) -> bool:
        with self._shared.lock:
            if self._generation != self._shared.task_generation:
                return True
            return self._shared.stop_requested

    def is_paused(self) -> bool:
        """현재 pause 요청 중(pause_in_flight) 이거나 PAUSED 인지 반환.

        task 내부에서 pause 를 명시적으로 흡수해야 하는 구간(예: DSR 의
        MovePause 로 제어되지 않는 force-control compliance loop)에서 사용한다.
        일반적인 trajectory motion 은 wait_motion() 이 pause 를 자동으로 처리하므로
        이 메서드를 직접 호출할 필요가 없다.
        """
        with self._shared.lock:
            return (
                self._shared.pause_in_flight
                or self._shared.state == ExecutorState.PAUSED
            )

    def wait_motion(self) -> bool:
        """
        비동기 모션(amovej/amovel) 완료 대기.

        DSR_ROBOT2.check_motion()은 dsr_node에서 글로벌 SingleThreadedExecutor로
        spin되는 경로라, 우리 control_node의 MultiThreadedExecutor와 간섭하지
        않는다. 폴링 주기는 20ms. stop 요청 시 즉시 False 반환.

        pause_in_flight 또는 PAUSED 상태에서는 모션이 완료되어도 대기한다.
        이를 통해 pause 요청 직후(콜백 완료 전)에도 task가 진행되지 않도록 보장.
        """
        from DSR_ROBOT2 import check_motion
        motion_done = False

        while True:
            if self.check_stop():
                return False

            # pause 요청 중(pause_in_flight)이거나 이미 PAUSED면 대기
            # pause_in_flight 체크로 MovePause 콜백 완료 전에도 즉시 대기 시작
            with self._shared.lock:
                should_wait = (
                    self._shared.pause_in_flight or
                    self._shared.state == ExecutorState.PAUSED
                )

            if should_wait:
                time.sleep(0.02)
                continue

            # Resume 되었고 모션이 이미 완료되었으면 다음 step으로
            if motion_done:
                return True

            # 모션 상태 폴링
            try:
                status = check_motion()
            except Exception:
                # check_motion 내부 에러는 "아직 진행 중"으로 간주해 재시도.
                status = 1

            if status == 0:
                motion_done = True
                # 다음 루프에서 pause 체크 후 return
                continue

            time.sleep(0.02)

    def update_progress(self, step: int, total: int, name: str, message: str = ""):
        """Step 시작 지점에서 호출. Pause 중이면 Resume 까지 block.

        이 함수는 각 step의 시작(=amovel/amovej 직전)에서 호출되므로, 여기서
        pause 를 흡수해 새 모션이 시작되지 않도록 한다. 이렇게 하지 않으면
        step N 의 wait_motion 반환 후 step N+1 의 amovel 호출 사이의 gap 에
        pause 가 들어왔을 때, MovePause 가 DSR 에 도달해봤자 "paused 할 모션이
        없는" 상태라 무시되고 직후 amovel 로 시작한 새 모션이 끝까지 진행되는
        버그가 발생한다.

        Pause 중 block 하는 동안 stop/stale 이 오면 조용히 return (업데이트만
        skip). 후속 amovel 은 곧바로 wait_motion 에서 stop 으로 걸러진다.
        """
        while True:
            with self._shared.lock:
                # stale generation or stop requested → 업데이트 skip
                if self._generation != self._shared.task_generation:
                    return
                if self._shared.stop_requested:
                    return
                # pause_in_flight 또는 PAUSED 면 block
                should_wait = (
                    self._shared.pause_in_flight or
                    self._shared.state == ExecutorState.PAUSED
                )
                if not should_wait:
                    self._shared.current_step = self._shared.module_step_offset + step
                    self._shared.step_name = name
                    if message:
                        self._shared.message = message
                    return
            time.sleep(0.02)


# ───────────────────── Task Registry ─────────────────────
# sequential.launch.py 의 OnProcessExit 순차 실행 패턴을 리스트로 표현.
# 조합 task (예: auto_serving) 는 module 리스트로 정의하면 motion_executor 가
# 각 모듈 STEPS 길이를 합산해 total_steps / current_step 오프셋을 자동 계산한다.
from cobot1.tasks import (  # noqa: E402
    task_gripper_open, task_home, task_rice, task_tong, task_sauce,
)

# TASK_REGISTRY
#
# 모든 진입(home/배식/복구) 은 JReady 로의 첫 amovej 전에 그리퍼를 열어 안전을 확보한다.
# 이를 위해 모든 외부 task 는 `task_gripper_open` 을 최선행 모듈로 합성한다.
# (rice/tong/sauce 각 모듈 안에도 release_pre/release_1 단계가 있지만 그 이전에
#  `amovej(JReady)` 가 먼저 실행되므로, 쥔 상태로 JReady 이동하는 위험이 남아 있음)
#
# - "home": 정상 운영 중 홈 버튼도 안전을 위해 그리퍼 먼저 열고 홈으로.
# - "recovery_*": SAFE_STOP/SAFE_OFF 복구 후 홈/원래 작업 재개 시 사용. UI 가 선택.
# - "auto_serving": 자동 배식. rice → tong → sauce 연속 실행 (앞에 gripper_open 선행).
# - "gripper_open": 단독 실행 (디버그/수동 트리거용).
TASK_REGISTRY: dict[str, list] = {
    # 단일 모듈
    task_gripper_open.TASK_NAME: [task_gripper_open],
    task_home.TASK_NAME:         [task_gripper_open, task_home],
    task_rice.TASK_NAME:         [task_gripper_open, task_rice],
    task_tong.TASK_NAME:         [task_gripper_open, task_tong],
    task_sauce.TASK_NAME:        [task_gripper_open, task_sauce],

    # 자동 배식: gripper_open → rice → tong → sauce
    "auto_serving":              [task_gripper_open, task_rice, task_tong, task_sauce],

    # 복구 플로우 — SAFE_STOP/SAFE_OFF 해제 후 홈/재시작.
    # 현재는 home/rice/tong/sauce/auto_serving 정의와 동일하지만, 향후 복구 전용
    # 점검 스텝(툴 체크 등)을 추가할 수 있도록 별도 키를 유지한다.
    # `recovery_<single>` 은 단일 모듈만 재개 — 예) 그냥 tong 만 실행하던 중 중단
    # 된 경우 복구 후 동일하게 tong 만 재시작.
    "recovery_home":             [task_gripper_open, task_home],
    "recovery_rice":             [task_gripper_open, task_rice],
    "recovery_tong":             [task_gripper_open, task_tong],
    "recovery_sauce":            [task_gripper_open, task_sauce],
    "recovery_auto_serving":     [task_gripper_open, task_rice, task_tong, task_sauce],

    # auto_serving 중단 → 해당 모듈부터 **남은 모듈까지 연속 실행**.
    # 예) auto_serving 중 tong 에서 SAFE_STOP → 복구 후 재시작 = tong → sauce
    #     auto_serving 중 rice 에서 SAFE_STOP → 복구 후 재시작 = rice → tong → sauce (=auto_serving 과 동일)
    #     auto_serving 중 sauce 에서 SAFE_STOP → 복구 후 재시작 = sauce
    "resume_from_rice":          [task_gripper_open, task_rice, task_tong, task_sauce],
    "resume_from_tong":          [task_gripper_open, task_tong, task_sauce],
    "resume_from_sauce":         [task_gripper_open, task_sauce],
}


def _auto_serving_parent_skip_steps(task_name: str) -> int:
    """resume_from_<mod> 에서 auto_serving 중 "건너뛴" 모듈들의 STEPS 합.

    progress 를 auto_serving 전체 scale 로 보이게 하려고, 건너뛴 모듈의 step 수만큼
    total_steps / 초기 step_offset 에 더해준다. gripper_open 은 resume task 앞쪽에도
    포함돼 있으므로 중복 카운트 방지를 위해 제외한다.
    """
    if not task_name.startswith("resume_from_"):
        return 0
    pivot = task_name[len("resume_from_"):]
    parent = TASK_REGISTRY.get("auto_serving", [])
    skipped: list = []
    for m in parent:
        mname = getattr(m, "TASK_NAME", "")
        if mname == pivot:
            break
        if mname == "gripper_open":
            continue
        skipped.append(m)
    return sum(len(m.STEPS) for m in skipped)


class ControlNode(Node):
    """외부 I/F 전용 노드. MultiThreadedExecutor에서 spin됨."""

    def __init__(self, shared: _SharedState):
        super().__init__("motion_executor", namespace=ROBOT_ID)
        self._shared = shared
        self._cb_group = ReentrantCallbackGroup()

        self._task_thread: threading.Thread | None = None
        self._stop_watchdog_timer = None

        self.create_service(
            StartTask, "execute_task/start", self._on_start,
            callback_group=self._cb_group,
        )
        self.create_service(
            StopTask, "execute_task/stop", self._on_stop,
            callback_group=self._cb_group,
        )

        self._move_stop_cli = self.create_client(
            MoveStop, f"/{ROBOT_ID}/motion/move_stop", callback_group=self._cb_group,
        )
        self._move_pause_cli = self.create_client(
            MovePause, f"/{ROBOT_ID}/motion/move_pause", callback_group=self._cb_group,
        )
        self._move_resume_cli = self.create_client(
            MoveResume, f"/{ROBOT_ID}/motion/move_resume", callback_group=self._cb_group,
        )

        # SAFE_STOP 복구용 DSR2 system 서비스.
        # set_safe_stop_reset_type(0) 후 set_robot_mode(1=AUTONOMOUS) 호출이
        # 실제 SAFE_STOP 해제 트리거 시퀀스다. docs/ARCHITECTURE.md SAFE_STOP
        # 복구 절차 참고.
        self._set_safe_stop_reset_type_cli = self.create_client(
            SetSafeStopResetType,
            f"/{ROBOT_ID}/system/set_safe_stop_reset_type",
            callback_group=self._cb_group,
        )
        self._set_robot_mode_cli = self.create_client(
            SetRobotMode,
            f"/{ROBOT_ID}/system/set_robot_mode",
            callback_group=self._cb_group,
        )
        self._set_robot_control_cli = self.create_client(
            SetRobotControl,
            f"/{ROBOT_ID}/system/set_robot_control",
            callback_group=self._cb_group,
        )
        self._servo_off_cli = self.create_client(
            ServoOff,
            f"/{ROBOT_ID}/system/servo_off",
            callback_group=self._cb_group,
        )

        self._task_state_pub = self.create_publisher(TaskState, "execute_task/state", 10)
        self.create_timer(0.1, self._publish_task_state)

        # 로봇 상태 폴링은 별도 프로세스 `robot_status_publisher` 가 담당한다
        # (cobot1/robot_status_publisher.py). 이 프로세스 안에서는 명령/태스크만.
        #
        # 단 안전 정지(SAFE_STOP/SAFE_OFF 등) 진입 시 task thread 가 계속 STEPS 를
        # 소비하며 _gripper() 호출이 반복되는 문제를 막기 위해, motion_executor 도
        # robot_status 를 구독해 fault 진입 시 stop_requested 를 세팅한다.
        # (DSR 이 모션은 차단하지만 set_digital_output 은 그대로 통과되어,
        # check_motion() 이 idle 로 즉시 반환되면서 STEPS 루프가 폭주함.)
        self._robot_state_available = False
        self._robot_state_in_fault = False
        # 최근 관측된 robot_state enum 값. _handle_to_standby 등 복구 서비스가
        # DSR 상태 전환을 검증할 때 이 값을 폴링한다.
        self._latest_robot_state = ROBOT_STATE_UNKNOWN
        self._robot_status_sub = self.create_subscription(
            RobotStatus,
            "motion_executor/robot_status",
            self._on_robot_status,
            10,
        )

        self.get_logger().info(
            f"MotionExecutor ready. tasks={list(TASK_REGISTRY)}"
        )

    # ═══════════════════ Robot status ═══════════════════
    def _on_robot_status(self, msg: RobotStatus):
        """robot_status_publisher 가 발행하는 DSR robot_state 를 감시.

        하드웨어가 SAFE_STOP/SAFE_OFF/EMERGENCY 로 진입하면 task thread 의
        stop_requested 를 세팅해 잔여 STEPS(특히 _gripper) 실행을 즉시 차단한다.
        이미 STOPPING/ERROR/IDLE 상태면 추가 액션 없음.

        주의:
        - move_stop 은 여기서 호출하지 않는다. DSR 이 이미 하드웨어 레벨에서
          모션을 차단한 상태고, SAFE_STOP 복구(TO_STANDBY)는 UI 버튼을 통해서
          별도 플로우(stop_type=6/7/8) 로 진행된다.
        - 복구 해제 시(prev_fault && !new_fault) 에는 별도 처리 없이 IDLE 대기.
          사용자가 UI "🏠 홈으로"/"🔁 재시작" 을 눌러야 새 task 가 시작된다.
        """
        if not msg.robot_state_valid:
            return
        self._robot_state_available = True
        self._latest_robot_state = msg.robot_state
        now_in_fault = msg.robot_state in ROBOT_FAULT_STATES

        if now_in_fault and not self._robot_state_in_fault:
            self._robot_state_in_fault = True
            with self._shared.lock:
                running = self._shared.state in (
                    ExecutorState.RUNNING, ExecutorState.PAUSED, ExecutorState.STOPPING,
                )
                if running:
                    self._shared.stop_requested = True
                    self._shared.state = ExecutorState.ERROR
                    self._shared.message = (
                        f"로봇 안전 정지 (robot_state={msg.robot_state}) — "
                        "task thread 중단"
                    )
            if running:
                self.get_logger().error(
                    f"Robot entered fault state {msg.robot_state}; task thread will exit"
                )
        elif not now_in_fault and self._robot_state_in_fault:
            self._robot_state_in_fault = False
            self.get_logger().info(
                f"Robot left fault state (now {msg.robot_state}); awaiting new task"
            )

    # ═══════════════════ Task state publishing ═══════════════════
    def _publish_task_state(self):
        msg = TaskState()
        with self._shared.lock:
            msg.state = int(self._shared.state)
            msg.task_name = self._shared.task_name
            msg.current_step = self._shared.current_step
            msg.total_steps = self._shared.total_steps
            msg.current_step_name = self._shared.step_name
            msg.progress = (
                self._shared.current_step / self._shared.total_steps
                if self._shared.total_steps > 0 else 0.0
            )
            msg.message = self._shared.message
            # 합성 task 의 현재 서브모듈 정보 (task_controller / ui_bridge / UI 로 전파).
            msg.current_module_name = self._shared.module_name
            msg.current_module_label = self._shared.module_label
            msg.module_index = int(self._shared.module_index)
            msg.module_total = int(self._shared.module_total)
        msg.stamp = self.get_clock().now().to_msg()
        self._task_state_pub.publish(msg)

    # ═══════════════════ Task start/stop services ═══════════════════
    def _on_start(self, request, response):
        task_name = request.task_name
        self.get_logger().info(f"Start request: {task_name}")

        if task_name not in TASK_REGISTRY:
            response.success = False
            response.message = (
                f"Unknown task: {task_name}. Available: {list(TASK_REGISTRY)}"
            )
            return response

        with self._shared.lock:
            if self._shared.state in (ExecutorState.RUNNING, ExecutorState.PAUSED):
                response.success = False
                response.message = "Task already running or paused"
                return response

            if self._task_thread is not None and self._task_thread.is_alive():
                self.get_logger().warn(
                    "Previous task thread still alive; bumping generation to detach"
                )

            modules = TASK_REGISTRY[task_name]
            first_module = modules[0] if modules else None
            self._shared.task_generation += 1
            self._shared.state = ExecutorState.RUNNING
            self._shared.task_name = task_name
            self._shared.module_index = 0
            self._shared.module_total = len(modules)
            self._shared.module_step_offset = 0
            self._shared.module_name = (
                getattr(first_module, "TASK_NAME", "") if first_module else ""
            )
            self._shared.module_label = (
                getattr(first_module, "TASK_LABEL", self._shared.module_name)
                if first_module else ""
            )
            self._shared.current_step = 0
            self._shared.total_steps = (
                sum(len(m.STEPS) for m in modules)
                + _auto_serving_parent_skip_steps(task_name)
            )
            self._shared.step_name = ""
            self._shared.message = f"Starting {task_name}"
            self._shared.stop_requested = False
            gen = self._shared.task_generation

        self._task_thread = threading.Thread(
            target=self._run_task_sequence,
            args=(task_name, gen),
            daemon=True,
        )
        self._task_thread.start()

        response.success = True
        response.message = f"Task {task_name} started"
        response.task_id = request.task_id or "task_001"
        return response

    def _on_stop(self, request, response):
        stop_type = request.stop_type

        # 2: EMERGENCY — task 종료 + servo off
        if stop_type == 2:
            return self._handle_emergency_stop(response)
        if stop_type == 3:
            return self._handle_pause(response)
        if stop_type == 4:
            return self._handle_resume(response)
        # 5: 레거시 SAFE_STOP_RECOVER (하위 호환)
        # 6: TO_STANDBY (자동 복구 → STANDBY)
        if stop_type in (5, 6):
            return self._handle_to_standby(response)
        # 7: TO_RECOVERY (수동 복구 → RECOVERY/무중력 모드)
        if stop_type == 7:
            return self._handle_to_recovery(response)
        # 8: RECOVERY_DONE (RECOVERY → STANDBY)
        if stop_type == 8:
            return self._handle_recovery_done(response)

        self.get_logger().info(f"Stop request (type={stop_type})")

        with self._shared.lock:
            if self._shared.state not in (ExecutorState.RUNNING, ExecutorState.PAUSED):
                response.success = False
                response.message = "No task running"
                return response
            was_paused = self._shared.state == ExecutorState.PAUSED
            self._shared.stop_requested = True
            self._shared.state = ExecutorState.STOPPING
            self._shared.message = "Stopping..."

        stop_mode = 1 if stop_type == 1 else 0

        # PAUSED에서 바로 stop하면 DSR이 pause 상태를 유지한 채 check_motion이
        # idle로 전환되지 않아 task 스레드가 hang된다. resume으로 먼저 풀어 준다.
        if was_paused and self._move_resume_cli.service_is_ready():
            self.get_logger().info("Resuming first to clear DSR pause state before stop")
            resume_fut = self._move_resume_cli.call_async(MoveResume.Request())
            resume_fut.add_done_callback(
                lambda _f, sm=stop_mode: self._send_move_stop(sm)
            )
        else:
            self._send_move_stop(stop_mode)

        self._stop_watchdog_timer = self.create_timer(
            3.0, self._stop_watchdog, callback_group=self._cb_group
        )

        response.success = True
        response.message = "Stop requested"
        return response

    def _send_move_stop(self, stop_mode: int):
        if not self._move_stop_cli.service_is_ready():
            self.get_logger().warn("move_stop service not available")
            return
        req = MoveStop.Request()
        req.stop_mode = stop_mode
        fut = self._move_stop_cli.call_async(req)
        fut.add_done_callback(self._on_move_stop_done)

    def _handle_emergency_stop(self, response):
        """비상 정지: task 종료 + servo off.

        1. move_stop(stop_mode=1, QUICK) — 현재 모션 즉시 정지
        2. servo_off(STOP_TYPE_QUICK) — 실제 서보 OFF (빨간 불)
        3. task 상태 리셋

        복구 시에는 TO_STANDBY(6) 사용 — set_robot_control + set_robot_mode(AUTONOMOUS).
        """
        self.get_logger().warn("EMERGENCY STOP request — stopping task and servo off")

        with self._shared.lock:
            self._shared.stop_requested = True
            self._shared.state = ExecutorState.STOPPING
            self._shared.message = "비상 정지 중..."

        # 1. move_stop (QUICK) — 현재 모션 즉시 정지
        if self._move_stop_cli.service_is_ready():
            stop_req = MoveStop.Request()
            stop_req.stop_mode = 1  # ST_QUICK
            stop_fut = self._move_stop_cli.call_async(stop_req)
            self._wait_future(stop_fut, 2.0)
            self.get_logger().info("move_stop(QUICK) sent")
        else:
            self.get_logger().warn("move_stop service not available")

        # 2. servo_off — 실제 서보 OFF (빨간 불)
        if self._servo_off_cli.service_is_ready():
            servo_req = ServoOff.Request()
            servo_req.stop_type = SERVO_OFF_STOP_TYPE_QUICK  # Quick stop
            servo_fut = self._servo_off_cli.call_async(servo_req)
            if self._wait_future(servo_fut, 3.0):
                res = servo_fut.result()
                if res and getattr(res, "success", False):
                    self.get_logger().warn("servo_off(QUICK) ok — SERVO OFF (red light)")
                else:
                    self.get_logger().error("servo_off(QUICK) failed")
            else:
                self.get_logger().error("servo_off timeout")
        else:
            self.get_logger().error("servo_off service not available")

        # 3. task 상태 리셋
        self._reset_executor_state("비상 정지 완료 — 서보 OFF 상태")

        response.success = True
        response.message = "Emergency stop executed — servo off"
        return response

    def _handle_pause(self, response):
        self.get_logger().info("Pause request")
        with self._shared.lock:
            if self._shared.state != ExecutorState.RUNNING:
                response.success = False
                response.message = "No task running to pause"
                return response
            if self._shared.pause_in_flight:
                response.success = False
                response.message = "Pause already in progress"
                self.get_logger().info("Pause request ignored: already in flight")
                return response
            self._shared.pause_in_flight = True

        if self._move_pause_cli.service_is_ready():
            fut = self._move_pause_cli.call_async(MovePause.Request())
            fut.add_done_callback(self._on_move_pause_done)
            response.success = True
            response.message = "Pause requested"
        else:
            with self._shared.lock:
                self._shared.pause_in_flight = False
            response.success = False
            response.message = "move_pause service not available"
        return response

    def _handle_resume(self, response):
        self.get_logger().info("Resume request")
        with self._shared.lock:
            if self._shared.state != ExecutorState.PAUSED:
                response.success = False
                response.message = "Task not paused"
                return response

        if self._move_resume_cli.service_is_ready():
            fut = self._move_resume_cli.call_async(MoveResume.Request())
            fut.add_done_callback(self._on_move_resume_done)
            response.success = True
            response.message = "Resume requested"
        else:
            response.success = False
            response.message = "move_resume service not available"
        return response

    def _wait_future(self, future, timeout_sec: float) -> bool:
        """MultiThreadedExecutor 호환 future 폴링.

        콜백 스레드 안에서 `rclpy.spin_until_future_complete` 을 호출하면 현재
        실행 중인 MT executor 와 글로벌 SingleThreadedExecutor (DSR_ROBOT2 wrapper
        가 사용) 사이에 wait_set 경합이 발생할 수 있어 피한다. MT executor 의 다른
        worker 가 응답 콜백을 처리해 주므로 단순 polling 으로도 교착 없이 대기 가능.
        """
        start = time.monotonic()
        while not future.done() and (time.monotonic() - start) < timeout_sec:
            time.sleep(0.01)
        return future.done()

    def _call_set_robot_control(self, control_type: int, timeout: float = 3.0):
        """set_robot_control 서비스 호출 헬퍼. (success, message) 반환."""
        if not self._set_robot_control_cli.service_is_ready():
            return False, "set_robot_control service not available"

        req = SetRobotControl.Request()
        req.robot_control = control_type
        fut = self._set_robot_control_cli.call_async(req)
        if not self._wait_future(fut, timeout):
            return False, f"set_robot_control({control_type}) timeout"

        res = fut.result()
        if res is None or not getattr(res, "success", False):
            return False, f"set_robot_control({control_type}) failed"
        return True, f"set_robot_control({control_type}) ok"

    def _wait_robot_state(self, expected_states, timeout: float = 3.0, poll: float = 0.1) -> bool:
        """robot_status 구독을 통해 들어오는 robot_state 가 expected_states 중 하나가
        될 때까지 폴링 대기.

        DSR 의 `set_robot_control` 서비스는 "명령 수신" 을 의미하고, 실제 상태
        전환 (SAFE_STOP → STANDBY 등) 은 수백 ms ~ 수 초 걸린다. 서비스가 성공을
        반환했다고 바로 다음 단계를 보내면 DSR state machine 이 reject 하는
        사례가 있어 (유저 증상: "자동 복구 요청을 보냈습니다" 후 IDLE 에 머묾),
        이 헬퍼로 각 단계 후 실제 상태 전환을 확인한다.

        Args:
            expected_states: 기대하는 robot_state enum 값들 (iterable of int).
            timeout: 최대 대기 (초).
            poll: 폴링 간격 (초).

        Returns:
            True: timeout 안에 expected 에 도달.
            False: timeout.
        """
        import time as _time
        if isinstance(expected_states, int):
            expected_states = (expected_states,)
        else:
            expected_states = tuple(expected_states)
        deadline = _time.monotonic() + timeout
        while _time.monotonic() < deadline:
            if self._latest_robot_state in expected_states:
                return True
            _time.sleep(poll)
        return False

    def _reset_executor_state(self, message: str):
        """내부 ExecutorState 를 IDLE 로 리셋."""
        with self._shared.lock:
            self._shared.stop_requested = True
            self._shared.state = ExecutorState.IDLE
            self._shared.task_name = ""
            self._shared.module_index = 0
            self._shared.module_total = 0
            self._shared.module_step_offset = 0
            self._shared.module_name = ""
            self._shared.module_label = ""
            self._shared.current_step = 0
            self._shared.step_name = ""
            self._shared.message = message

    def _handle_to_standby(self, response):
        """SAFE_STOP/SAFE_OFF 계열 → STANDBY 복구 (자동 복구).

        현재 robot_state 를 관찰해 단계별로 set_robot_control 을 반복 호출한다.
        실측상 DSR 은 `CONTROL_RESET_SAFET_STOP(2)` 을 호출해도 `SAFE_STOP(5)` →
        `STANDBY(1)` 로 직행하지 않고 중간에 `SAFE_OFF(3)` 를 거쳐 다시 한 번
        `CONTROL_RESET_SAFET_OFF(3)` 가 필요한 경우가 많다. 상태 다이어그램(docs)
        이 STANDBY 직행을 이야기하지만 펌웨어 구현은 다르므로, 여기서는 실제
        관찰된 state 를 follow 하며 STANDBY 에 도달할 때까지 루프.

        상태별 전이:
            SAFE_STOP(5)   ──(2)─► SAFE_OFF(3)  ──(3)─► STANDBY(1)
                            └─ or ────────────────────► STANDBY(1)
            SAFE_OFF(3)    ──(3)─► STANDBY(1)  (1계열 진입 시)
            SAFE_OFF(3)    ──(5)─► RECOVERY(8) ──(7)─► STANDBY(1)  (2계열 진입 시)
            SAFE_STOP2(9)  ──(4)─► RECOVERY(8) ──(7)─► STANDBY(1)
            SAFE_OFF2(10)  ──(5)─► RECOVERY(8) ──(7)─► STANDBY(1)
            RECOVERY(8)    ──(7)─► STANDBY(1)
            STANDBY(1)/MOVING(2): 바로 종료.

        SAFE_OFF 1계열/2계열 분기 주의:
            DSR 펌웨어는 SAFE_STOP2(9) 에서 자동 전이된 SAFE_OFF 도 robot_state 를
            3 으로 보고한다 (SAFE_OFF2=10 는 별도 경로에서만 사용됨). 그러나 내부
            적으로 진입 경로를 기억해 1계열 해제 명령(CONTROL_RESET_SAFET_OFF=3)
            을 거부한다. 이 함수는 진입 경로를 모르므로 1계열을 먼저 시도하고,
            state 가 동일하게 유지되면 2계열(CONTROL_RECOVERY_SAFE_OFF=5)로 fallback
            한다.

        전체 타임아웃 (`RECOVERY_OVERALL_TIMEOUT`) 동안 위 전이를 반복. 같은 상태
        가 반복되면 무한 루프 방지를 위해 `max_iterations` 로 종료.
        """
        import time as _time

        RECOVERY_OVERALL_TIMEOUT = 20.0  # DSR 상태 전환이 느릴 때를 대비한 전체 한계
        PER_STEP_WAIT = 4.0              # set_robot_control 후 state 변화 대기
        MAX_ITERATIONS = 8               # 상태가 안 바뀔 때 안전 브레이크

        self.get_logger().info(
            f"TO_STANDBY (자동 복구) request — current robot_state={self._latest_robot_state}"
        )

        if not self._set_robot_control_cli.service_is_ready():
            response.success = False
            response.message = "set_robot_control service not available"
            self.get_logger().error(response.message)
            return response

        deadline = _time.monotonic() + RECOVERY_OVERALL_TIMEOUT
        last_state = None
        same_state_count = 0

        for iteration in range(MAX_ITERATIONS):
            if _time.monotonic() > deadline:
                self.get_logger().error(
                    f"TO_STANDBY overall timeout — last state={self._latest_robot_state}"
                )
                response.success = False
                response.message = (
                    f"STANDBY 전환 실패 (현재 state={self._latest_robot_state}). "
                    "로봇을 리부트하거나 수동 복구를 시도하세요."
                )
                return response

            state = self._latest_robot_state

            if state in (ROBOT_STATE_STANDBY, ROBOT_STATE_MOVING):
                # 목표 도달.
                break

            # 같은 non-terminal state 가 반복되면 DSR 이 더 이상 진행하지 않는 것.
            # (서비스는 success 를 주지만 실제 전이가 안 됨.) 즉시 실패 처리해서
            # 무한 루프 방지.
            if state == last_state:
                same_state_count += 1
                if same_state_count >= 2:
                    self.get_logger().error(
                        f"TO_STANDBY: state={state} 에서 진행 안 됨 "
                        f"(iteration={iteration})"
                    )
                    response.success = False
                    response.message = (
                        f"STANDBY 전환 실패 (현재 state={state}). "
                        "로봇이 같은 안전 상태에서 더 이상 진행하지 않습니다. "
                        "수동 복구 또는 리부트가 필요합니다."
                    )
                    return response
            else:
                same_state_count = 0
                last_state = state

            # 현재 state 에 맞는 control 선택.
            if state == ROBOT_STATE_SAFE_STOP:
                control, label = CONTROL_RESET_SAFET_STOP, "SAFE_STOP → (STANDBY/SAFE_OFF)"
            elif state == ROBOT_STATE_SAFE_OFF:
                # SAFE_OFF(3) 는 1계열(SAFE_STOP→SAFE_OFF) 로 도달한 것일 수도 있고,
                # 2계열(SAFE_STOP2→자동전이 SAFE_OFF) 로 도달한 것일 수도 있다.
                # DSR 펌웨어는 진입 경로를 기억해 해제 명령을 구분하므로 state 숫자
                # 만으로는 알 수 없다. 먼저 1계열 해제(CONTROL_RESET_SAFET_OFF=3)를
                # 시도하고, state 가 바뀌지 않으면(same_state_count>=1) 다음 iteration
                # 에서 2계열 해제(CONTROL_RECOVERY_SAFE_OFF=5)로 RECOVERY 경유를
                # 시도한다. RECOVERY(8) 로 넘어가면 아래 RECOVERY 분기가 STANDBY
                # 까지 마저 이어준다. 두 경로 모두 실패하면 same_state_count>=2 에서
                # 루프가 자연스럽게 실패 처리된다.
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
            elif state == ROBOT_STATE_SAFE_STOP2:
                control, label = CONTROL_RECOVERY_SAFE_STOP, "SAFE_STOP2 → RECOVERY"
            elif state == ROBOT_STATE_SAFE_OFF2:
                control, label = CONTROL_RECOVERY_SAFE_OFF, "SAFE_OFF2 → RECOVERY"
            elif state == ROBOT_STATE_RECOVERY:
                control, label = CONTROL_RESET_RECOVERY, "RECOVERY → STANDBY"
            else:
                # INITIALIZING / HOMMING / UNKNOWN 등 전이 중 상태: 잠깐 기다렸다가 재검사.
                self.get_logger().info(
                    f"TO_STANDBY: state={state} (transient?) — 대기 후 재검사"
                )
                _time.sleep(0.5)
                continue

            ok, msg = self._call_set_robot_control(control, 3.0)
            if not ok:
                self.get_logger().warning(f"{label} 호출 실패: {msg}")
                # 서비스 실패여도 잠시 대기 후 state 를 다시 본다 (이미 전이됐을 수 있음).
                _time.sleep(0.5)
                continue

            self.get_logger().info(f"{label}: {msg}")

            # state 가 바뀔 때까지 대기. 같은 state 로 남으면 다음 iteration 에서
            # same_state_count 로 감지된다.
            changed_states = tuple(
                s for s in (
                    ROBOT_STATE_STANDBY, ROBOT_STATE_SAFE_OFF, ROBOT_STATE_RECOVERY,
                    ROBOT_STATE_MOVING, ROBOT_STATE_INITIALIZING, ROBOT_STATE_HOMMING,
                )
                if s != state
            )
            self._wait_robot_state(changed_states, timeout=PER_STEP_WAIT)
            _time.sleep(0.2)

        else:
            # for 루프가 break 없이 끝남 — max_iterations 도달.
            self.get_logger().error(
                f"TO_STANDBY: max iterations reached — state={self._latest_robot_state}"
            )
            response.success = False
            response.message = (
                f"STANDBY 전환 실패 (현재 state={self._latest_robot_state}). "
                "반복 시도해도 진행되지 않아 중단합니다."
            )
            return response

        # EMERGENCY_STOP / NOT_READY / UNKNOWN 은 루프 안에서 걸러지지 않으므로
        # 최종 상태 재검사.
        final_state = self._latest_robot_state
        if final_state not in (ROBOT_STATE_STANDBY, ROBOT_STATE_MOVING):
            self.get_logger().error(
                f"TO_STANDBY: final state={final_state}, not STANDBY"
            )
            response.success = False
            response.message = (
                f"자동 복구를 지원하지 않는 로봇 상태입니다 (state={final_state}). "
                "비상정지를 해제하거나 수동 복구를 사용하세요."
            )
            return response

        # ── 4단계: AUTONOMOUS 모드 설정 (STANDBY 에서 task 실행 가능하도록) ──
        if self._set_robot_mode_cli.service_is_ready():
            mode_req = SetRobotMode.Request()
            mode_req.robot_mode = ROBOT_MODE_AUTONOMOUS
            mode_fut = self._set_robot_mode_cli.call_async(mode_req)
            if self._wait_future(mode_fut, 3.0):
                self.get_logger().info("set_robot_mode(AUTONOMOUS) ok")
            else:
                self.get_logger().warning("set_robot_mode(AUTONOMOUS) timeout (non-fatal)")

        # ── 5단계: 내부 상태 리셋 ──
        self._reset_executor_state("Robot recovered to STANDBY")

        response.success = True
        response.message = "Robot recovered to STANDBY"
        return response

    def _handle_to_recovery(self, response):
        """SAFE_STOP2/SAFE_OFF2 → RECOVERY 모드 전환 (수동 복구/무중력 모드).

        RECOVERY 모드에서는 사용자가 로봇을 손으로 이동할 수 있다.
        이동 완료 후 stop_type=8 (RECOVERY_DONE) 을 호출해 STANDBY 로 전환.

        주의: SAFE_STOP/SAFE_OFF(1계열)에서는 RECOVERY 모드로 직접 갈 수 없음.
        이 경우 _handle_to_standby 를 사용해 바로 STANDBY 로 가야 함.
        """
        self.get_logger().info(
            f"TO_RECOVERY (수동 복구) request — current robot_state={self._latest_robot_state}"
        )

        if not self._set_robot_control_cli.service_is_ready():
            response.success = False
            response.message = "set_robot_control service not available"
            self.get_logger().error(response.message)
            return response

        state = self._latest_robot_state

        if state == ROBOT_STATE_SAFE_STOP2:
            ok, msg = self._call_set_robot_control(CONTROL_RECOVERY_SAFE_STOP, 3.0)
        elif state == ROBOT_STATE_SAFE_OFF2:
            ok, msg = self._call_set_robot_control(CONTROL_RECOVERY_SAFE_OFF, 3.0)
        elif state == ROBOT_STATE_RECOVERY:
            # 이미 RECOVERY — 바로 성공 처리.
            self.get_logger().info("Already in RECOVERY mode")
            self._reset_executor_state("Robot already in RECOVERY mode")
            response.success = True
            response.message = "Robot already in RECOVERY mode."
            return response
        else:
            response.success = False
            response.message = (
                f"RECOVERY 모드 전환 실패: 지원하지 않는 state={state}. "
                "SAFE_STOP/SAFE_OFF(1계열)에서는 '자동 복구'를 사용하세요."
            )
            self.get_logger().warning(response.message)
            return response

        if not ok:
            response.success = False
            response.message = f"RECOVERY 전환 실패: {msg}"
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(msg)
        # DSR 가 실제로 RECOVERY(8) 로 전환될 때까지 대기.
        if not self._wait_robot_state(ROBOT_STATE_RECOVERY, timeout=3.0):
            self.get_logger().error(
                f"RECOVERY 전환 검증 timeout (state={self._latest_robot_state})"
            )
            response.success = False
            response.message = (
                f"RECOVERY 전환 실패 (현재 state={self._latest_robot_state})."
            )
            return response

        # 내부 상태는 IDLE 유지 (RECOVERY 중이라 task 실행 안 함)
        self._reset_executor_state("Robot in RECOVERY mode — move manually")

        response.success = True
        response.message = "Robot in RECOVERY mode. Move robot manually, then click 'Complete'."
        return response

    def _handle_recovery_done(self, response):
        """RECOVERY → STANDBY 전환 (수동 복구 완료)."""
        import time as _time

        self.get_logger().info(
            f"RECOVERY_DONE request — current robot_state={self._latest_robot_state}"
        )

        if not self._set_robot_control_cli.service_is_ready():
            response.success = False
            response.message = "set_robot_control service not available"
            self.get_logger().error(response.message)
            return response

        # 이미 STANDBY 인 경우 바로 반환.
        if self._latest_robot_state == ROBOT_STATE_STANDBY:
            self.get_logger().info("Already in STANDBY; recovery_done skipped")
            self._reset_executor_state("Robot already in STANDBY")
            response.success = True
            response.message = "Robot already in STANDBY"
            return response

        ok, msg = self._call_set_robot_control(CONTROL_RESET_RECOVERY, 3.0)
        if not ok:
            response.success = False
            response.message = f"RECOVERY → STANDBY 실패: {msg}"
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(msg)

        # DSR 상태가 STANDBY(1) 로 전환될 때까지 대기 + 검증.
        if not self._wait_robot_state(ROBOT_STATE_STANDBY, timeout=5.0):
            self.get_logger().error(
                f"RECOVERY → STANDBY 검증 timeout (state={self._latest_robot_state})"
            )
            response.success = False
            response.message = (
                f"STANDBY 전환 실패 (현재 state={self._latest_robot_state}). "
                "로봇을 안전 위치로 이동한 뒤 다시 시도하거나 리부트하세요."
            )
            return response

        _time.sleep(0.3)

        # AUTONOMOUS 모드 설정
        if self._set_robot_mode_cli.service_is_ready():
            mode_req = SetRobotMode.Request()
            mode_req.robot_mode = ROBOT_MODE_AUTONOMOUS
            mode_fut = self._set_robot_mode_cli.call_async(mode_req)
            if self._wait_future(mode_fut, 3.0):
                self.get_logger().info("set_robot_mode(AUTONOMOUS) ok")

        self._reset_executor_state("Robot recovered to STANDBY (manual recovery done)")

        response.success = True
        response.message = "Robot recovered to STANDBY"
        return response

    def _on_move_pause_done(self, future):
        try:
            res = future.result()
            if res and res.success:
                self.get_logger().info("Motion paused")
                with self._shared.lock:
                    self._shared.pause_in_flight = False
                    if self._shared.state == ExecutorState.RUNNING:
                        self._shared.state = ExecutorState.PAUSED
                        self._shared.message = "Paused"
                    else:
                        self.get_logger().warn(
                            f"MovePause succeeded but state is {self._shared.state.name}, "
                            "not transitioning to PAUSED"
                        )
            else:
                # MovePause 실패 시: 안전을 위해 task 전체 중단
                # (MoveStop + stop_requested 설정)
                error_msg = (
                    getattr(res, 'message', 'no message') if res
                    else "None result"
                )
                self.get_logger().warn(
                    f"MovePause failed ({error_msg}), aborting task with MoveStop"
                )
                self._abort_task_on_pause_failure("일시정지 실패 - 작업 중단 중...")
        except Exception as e:
            self.get_logger().error(f"move_pause exception: {e}, aborting task with MoveStop")
            self._abort_task_on_pause_failure("일시정지 오류 - 작업 중단 중...")

    def _abort_task_on_pause_failure(self, message: str):
        """MovePause 실패 시 안전하게 task를 중단."""
        with self._shared.lock:
            self._shared.pause_in_flight = False
            self._shared.stop_requested = True
            self._shared.state = ExecutorState.STOPPING
            self._shared.message = message
        self._send_move_stop(0)
        # 3초 후에도 STOPPING이면 강제 IDLE 전환
        if self._stop_watchdog_timer is not None:
            self._stop_watchdog_timer.cancel()
        self._stop_watchdog_timer = self.create_timer(
            3.0, self._stop_watchdog, callback_group=self._cb_group
        )

    def _on_move_resume_done(self, future):
        try:
            res = future.result()
            if res and res.success:
                self.get_logger().info("Motion resumed")
            elif res:
                # MoveResume 실패해도 task는 진행 (모션이 이미 완료된 경우 등)
                self.get_logger().warn(
                    f"MoveResume returned success=False: {getattr(res, 'message', 'no message')} "
                    "(task will continue anyway)"
                )
            else:
                self.get_logger().warn("MoveResume returned None result (task will continue anyway)")
        except Exception as e:
            self.get_logger().error(f"move_resume exception: {e} (task will continue anyway)")

        # MoveResume 결과와 관계없이 PAUSED → RUNNING 전환
        # (모션이 이미 완료된 상태에서 Resume하면 DSR은 실패를 반환할 수 있지만,
        #  executor 입장에서는 task를 계속 진행해야 함)
        with self._shared.lock:
            if self._shared.state == ExecutorState.PAUSED:
                self._shared.state = ExecutorState.RUNNING
                self._shared.message = "Resumed"
            else:
                self.get_logger().warn(
                    f"Resume callback but state is {self._shared.state.name}, "
                    "not transitioning to RUNNING"
                )

    def _on_move_stop_done(self, future):
        try:
            res = future.result()
            if res and res.success:
                self.get_logger().info("Robot motion stopped by DSR2")
        except Exception as e:
            self.get_logger().error(f"move_stop failed: {e}")

    def _stop_watchdog(self):
        if self._stop_watchdog_timer is not None:
            self._stop_watchdog_timer.cancel()
            self._stop_watchdog_timer = None
        with self._shared.lock:
            if self._shared.state == ExecutorState.STOPPING:
                self.get_logger().warn("Stop watchdog - forcing IDLE")
                self._shared.state = ExecutorState.IDLE
                self._shared.message = "Stopped (forced)"
                self._shared.task_name = ""
                # zombie task thread가 있어도 빠져나오도록 유지.
                # 새 start가 generation을 증가시키면 자연 무효화.
                self._shared.stop_requested = True

    # ═══════════════════ Task sequence runner (task thread) ═══════════════════
    def _run_task_sequence(self, task_name: str, generation: int):
        modules = TASK_REGISTRY[task_name]
        try:
            step_offset = _auto_serving_parent_skip_steps(task_name)
            for idx, module in enumerate(modules):
                mname = getattr(module, "TASK_NAME", module.__name__)
                mlabel = getattr(module, "TASK_LABEL", mname)
                with self._shared.lock:
                    if generation != self._shared.task_generation:
                        return
                    self._shared.module_index = idx
                    self._shared.module_step_offset = step_offset
                    self._shared.module_name = mname
                    self._shared.module_label = mlabel
                self.get_logger().info(
                    f"Running module {idx + 1}/{len(modules)}: {mname}"
                )

                ctx = _TaskContext(self._shared, idx, generation)
                if ctx.check_stop():
                    self._finish(generation, ExecutorState.IDLE, "Task stopped by user")
                    return

                success, message = module.run(ctx)

                if not success:
                    final_state = (
                        ExecutorState.IDLE
                        if (ctx.check_stop() or "stopped" in message.lower())
                        else ExecutorState.ERROR
                    )
                    self._finish(generation, final_state, message)
                    return

                step_offset += len(module.STEPS)
                self.get_logger().info(
                    f"Module {idx + 1}/{len(modules)} done: {mname}"
                )

            self._finish(generation, ExecutorState.IDLE, f"Task {task_name} completed")
        except Exception as e:
            self.get_logger().error(f"Task sequence error: {e}")
            with self._shared.lock:
                stale = generation != self._shared.task_generation
                was_stop = self._shared.stop_requested
            if stale:
                return
            final_state = ExecutorState.IDLE if was_stop else ExecutorState.ERROR
            self._finish(generation, final_state, str(e))

    def _finish(self, generation: int, state: ExecutorState, message: str):
        with self._shared.lock:
            if generation != self._shared.task_generation:
                return
        if self._stop_watchdog_timer is not None:
            self._stop_watchdog_timer.cancel()
            self._stop_watchdog_timer = None
        with self._shared.lock:
            self._shared.state = state
            self._shared.message = message
            self._shared.task_name = ""
            self._shared.module_index = 0
            self._shared.module_total = 0
            self._shared.module_step_offset = 0
            self._shared.module_name = ""
            self._shared.module_label = ""
            self._shared.current_step = 0
            self._shared.stop_requested = False


def main(args=None):
    rclpy.init(args=args)

    # 1. dsr_node: DSR_ROBOT2 전용. 어떤 executor에도 add하지 않는다.
    #    DSR_ROBOT2 함수가 호출될 때만 글로벌 SingleThreadedExecutor가 잠깐
    #    가져가 spin하고 돌려준다 (rclpy.spin_until_future_complete 패턴).
    #
    #    use_global_arguments=False:
    #    launch 파일의 Node(name=...) / remap (__node:=...) 은 rclpy 기본 동작상
    #    프로세스 내 **모든** 노드 이름을 덮어쓴다. 이걸 막지 않으면 dsr_node도
    #    control_node와 똑같은 이름("motion_executor")이 되어 rcl_logging_rosout
    #    에서 "Publisher already registered for provided node name" 경고가 뜬다.
    dsr_node = rclpy.create_node(
        "motion_executor_dsr", namespace=ROBOT_ID,
        use_global_arguments=False,
    )
    DR_init.__dsr__node = dsr_node
    dsr_node.get_logger().info("DSR node bound (not attached to any executor)")

    # 2. control_node: 외부 I/F 전용. MultiThreadedExecutor에서 spin.
    shared = _SharedState()
    control_node = ControlNode(shared)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(control_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        control_node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
