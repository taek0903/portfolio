"""
Task Controller Node

외부에서 Task 실행/중단을 요청하는 인터페이스.
Executor와 Service 통신.
로봇 상태는 motion_executor가 발행하는 `motion_executor/robot_status` 토픽
(`cobot_interfaces/RobotStatus`)을 구독해 받는다.

Note: DSR_ROBOT2를 직접 사용하지 않음
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import threading
import time
from enum import IntEnum

from cobot_interfaces.msg import RobotStatus, TaskState
from cobot_interfaces.srv import StartTask, StopTask


# 상수 직접 정의 (primitives import 없이)
ROBOT_ID = "dsr01"


class TaskStateEnum(IntEnum):
    IDLE = 0
    RUNNING = 1
    STOPPING = 2
    ERROR = 3
    PAUSED = 4


class RobotStateEnum(IntEnum):
    """DSR GetRobotState / cobot_interfaces/RobotStatus.robot_state 와 동일."""

    INITIALIZING = 0
    STANDBY = 1
    MOVING = 2
    SAFE_OFF = 3
    TEACHING = 4
    SAFE_STOP = 5
    EMERGENCY_STOP = 6
    HOMMING = 7
    RECOVERY = 8
    SAFE_STOP2 = 9
    SAFE_OFF2 = 10
    NOT_READY = 15
    UNKNOWN = 255


# 외력/충돌 등으로 작업을 중단 표시해야 하는 DSR 상태.
ROBOT_FAULT_STATES = frozenset({
    RobotStateEnum.SAFE_OFF,
    RobotStateEnum.SAFE_STOP,
    RobotStateEnum.EMERGENCY_STOP,
    RobotStateEnum.SAFE_STOP2,
    RobotStateEnum.SAFE_OFF2,
})


class TaskControllerNode(Node):
    def __init__(self):
        super().__init__("task_controller", namespace=ROBOT_ID)
        
        self._service_cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = ReentrantCallbackGroup()
        
        # State
        self._task_state = TaskStateEnum.IDLE
        self._task_message = ""
        self._lock = threading.Lock()
        
        # Robot State
        self._robot_state = RobotStateEnum.STANDBY
        self._robot_state_available = False
        # 로봇 안전 정지 진입 시점의 진행률 (executor 가 멈춘 뒤에도 올라가 보이는 것 방지)
        self._fault_progress_snapshot = 0.0
        
        # Executor state (from subscription)
        self._executor_state = 0
        self._executor_message = ""
        self._executor_progress = 0.0
        # Executor 로부터 수신한 그대로 UI 로 forward 하기 위한 캐시
        self._executor_task_name = ""
        self._executor_current_step = 0
        self._executor_total_steps = 0
        self._executor_step_name = ""
        self._executor_module_name = ""
        self._executor_module_label = ""
        self._executor_module_index = 0
        self._executor_module_total = 0
        
        # Service clients for Executor
        self._executor_start_client = self.create_client(
            StartTask, 'execute_task/start',
            callback_group=self._service_cb_group
        )
        self._executor_stop_client = self.create_client(
            StopTask, 'execute_task/stop',
            callback_group=self._service_cb_group
        )
        
        # Services (외부 인터페이스)
        self._start_srv = self.create_service(
            StartTask, 'task/start', self._start_callback,
            callback_group=self._service_cb_group
        )
        self._stop_srv = self.create_service(
            StopTask, 'task/stop', self._stop_callback,
            callback_group=self._service_cb_group
        )
        
        # Subscribe to motion_executor topics
        # - execute_task/state: task 실행 상태 (TaskState)
        # - motion_executor/robot_status: DSR monitoring 서비스 4종을 10Hz
        #   병렬 폴링해 합친 상태 (`cobot_interfaces/RobotStatus`).
        #   posj/tool_force/external_joint_torque 까지 함께 들어오므로 추후
        #   외력 기반 safety 로직도 이 토픽만 구독하면 된다.
        self._executor_state_sub = self.create_subscription(
            TaskState, 'execute_task/state',
            self._executor_state_callback, 10,
            callback_group=self._timer_cb_group,
        )
        self._robot_status_sub = self.create_subscription(
            RobotStatus, 'motion_executor/robot_status',
            self._robot_status_callback, 10,
            callback_group=self._timer_cb_group,
        )
        
        # State publisher
        self._state_pub = self.create_publisher(TaskState, 'task/state', 10)
        
        # Timers
        self._state_timer = self.create_timer(0.1, self._publish_state)
        
        self.get_logger().info("Task Controller started")
    
    def _executor_state_callback(self, msg):
        """Executor 상태 수신"""
        with self._lock:
            self._executor_state = msg.state
            self._executor_message = msg.message
            self._executor_task_name = getattr(msg, "task_name", "") or ""
            self._executor_current_step = int(getattr(msg, "current_step", 0) or 0)
            self._executor_total_steps = int(getattr(msg, "total_steps", 0) or 0)
            self._executor_step_name = getattr(msg, "current_step_name", "") or ""
            self._executor_module_name = getattr(msg, "current_module_name", "") or ""
            self._executor_module_label = getattr(msg, "current_module_label", "") or ""
            self._executor_module_index = int(getattr(msg, "module_index", 0) or 0)
            self._executor_module_total = int(getattr(msg, "module_total", 0) or 0)

            robot_fault = (
                self._robot_state_available
                and self._robot_state in ROBOT_FAULT_STATES
            )

            if robot_fault:
                # 진행률은 안전 정지 직전 값으로 고정.
                self._executor_progress = self._fault_progress_snapshot
                if msg.state == 1:  # RUNNING — DSR 는 멈췄는데 소프트웨어만 RUNNING 인 창구
                    return

            if not robot_fault:
                self._executor_progress = msg.progress

            # Executor 상태에 따라 Task 상태 업데이트
            if msg.state == 0:  # IDLE
                if self._task_state in (TaskStateEnum.RUNNING, TaskStateEnum.STOPPING, TaskStateEnum.PAUSED):
                    self._task_state = TaskStateEnum.IDLE
                    self._task_message = msg.message
                    self.get_logger().info(f"Task completed: {msg.message}")
            elif msg.state == 1:  # RUNNING
                self._task_state = TaskStateEnum.RUNNING
                self._task_message = msg.message
            elif msg.state == 2:  # STOPPING
                self._task_state = TaskStateEnum.STOPPING
            elif msg.state == 3:  # ERROR
                self._task_state = TaskStateEnum.ERROR
                self._task_message = msg.message
            elif msg.state == 4:  # PAUSED
                self._task_state = TaskStateEnum.PAUSED
                self._task_message = msg.message
                self.get_logger().info(f"Task paused: {msg.message}")
    
    def _robot_status_callback(self, msg: RobotStatus):
        """motion_executor/robot_status 토픽 수신.

        msg.robot_state 값이 valid 일 때만 내부 상태를 갱신한다. executor가
        아직 DSR 서비스에 연결되지 않은 초기 구간에서는 robot_state_valid=false
        이므로 마지막 유효값을 그대로 유지한다.
        """
        if not msg.robot_state_valid:
            return
        if not self._robot_state_available:
            self.get_logger().info("Robot status topic connected")
            self._robot_state_available = True
        try:
            state = RobotStateEnum(msg.robot_state)
        except ValueError:
            state = RobotStateEnum.UNKNOWN

        with self._lock:
            prev = self._robot_state
            self._robot_state = state
            prev_fault = prev in ROBOT_FAULT_STATES
            new_fault = state in ROBOT_FAULT_STATES

            if new_fault and not prev_fault:
                self._fault_progress_snapshot = self._executor_progress
                if self._task_state in (
                    TaskStateEnum.RUNNING,
                    TaskStateEnum.PAUSED,
                    TaskStateEnum.STOPPING,
                ):
                    self._task_state = TaskStateEnum.ERROR
                    self._task_message = (
                        f"로봇 안전 정지 ({state.name}) — 작업이 중단되었습니다."
                    )
                    self.get_logger().error(self._task_message)
                elif self._task_state == TaskStateEnum.IDLE:
                    self._task_state = TaskStateEnum.ERROR
                    self._task_message = (
                        f"로봇 안전 정지 ({state.name}) — 복구 후 작업을 시작하세요."
                    )
                    self.get_logger().warning(self._task_message)
                else:
                    self._task_message = (
                        f"로봇 안전 정지 ({state.name}) — 작업이 중단되었습니다."
                    )
            elif new_fault and prev_fault:
                self._task_message = (
                    f"로봇 안전 정지 ({state.name}) — 작업이 중단되었습니다."
                )
            elif prev_fault and not new_fault:
                if (
                    self._task_state == TaskStateEnum.ERROR
                    and self._executor_state == 0
                ):
                    self._task_state = TaskStateEnum.IDLE
                    self._task_message = "로봇 안전 상태가 해제되었습니다."
                    self.get_logger().info(self._task_message)

    def _wait_future(self, future, timeout_sec: float):
        """
        MultiThreadedExecutor 호환 future 대기.
        `rclpy.spin_until_future_complete`를 콜백 안에서 호출하면 글로벌 executor와
        현재 실행 중인 MT executor 사이에 wait_set 경합이 생겨 응답이 처리되지 않는다.
        여기서는 단순히 future.done()을 폴링 — MT executor의 다른 워커가 응답 콜백을
        계속 처리해주기 때문에 교착 없이 안전하게 대기할 수 있다.
        """
        start = time.monotonic()
        while not future.done() and (time.monotonic() - start) < timeout_sec:
            time.sleep(0.01)
        return future.done()

    def _publish_state(self):
        """통합 상태 발행"""
        msg = TaskState()
        with self._lock:
            msg.state = int(self._task_state)
            msg.message = self._task_message or self._executor_message
            robot_fault = (
                self._robot_state_available
                and self._robot_state in ROBOT_FAULT_STATES
            )
            if robot_fault and self._task_state == TaskStateEnum.ERROR:
                msg.progress = self._fault_progress_snapshot
            else:
                msg.progress = self._executor_progress
            # Executor 에서 받은 task/module/step 정보를 UI 로 그대로 forward.
            msg.task_name = self._executor_task_name
            msg.current_step = self._executor_current_step
            msg.total_steps = self._executor_total_steps
            msg.current_step_name = self._executor_step_name
            msg.current_module_name = self._executor_module_name
            msg.current_module_label = self._executor_module_label
            msg.module_index = self._executor_module_index
            msg.module_total = self._executor_module_total
        msg.stamp = self.get_clock().now().to_msg()
        self._state_pub.publish(msg)
    
    def _start_callback(self, request, response):
        """Task 시작"""
        with self._lock:
            current_state = self._task_state

        self.get_logger().info(f"Start request: {request.task_name}, state={TaskStateEnum(current_state).name}")

        if current_state == TaskStateEnum.RUNNING:
            response.success = False
            response.message = "Task already running"
            return response

        with self._lock:
            robot_fault = (
                self._robot_state_available
                and self._robot_state in ROBOT_FAULT_STATES
            )
        if robot_fault:
            response.success = False
            response.message = (
                "로봇이 안전 정지 상태입니다. 화면의 복구(홈/재시작)를 먼저 수행하세요."
            )
            return response

        # 방금 stop을 호출해서 STOPPING → IDLE 전환이 진행 중인 경우,
        # executor cleanup이 끝날 때까지 짧게 대기 (최대 3초).
        if current_state == TaskStateEnum.STOPPING:
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                time.sleep(0.05)
                with self._lock:
                    current_state = self._task_state
                if current_state != TaskStateEnum.STOPPING:
                    break
            self.get_logger().info(
                f"STOPPING cleared → state={TaskStateEnum(current_state).name}"
            )
            if current_state == TaskStateEnum.STOPPING:
                response.success = False
                response.message = "Task still stopping, please retry"
                return response
        
        # Executor 서비스 대기
        if not self._executor_start_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "Task Executor not available"
            return response
        
        # Executor에 시작 요청
        exec_request = StartTask.Request()
        exec_request.task_name = request.task_name
        exec_request.task_id = request.task_id or "task_001"
        
        future = self._executor_start_client.call_async(exec_request)
        self._wait_future(future, timeout_sec=5.0)

        if future.result() is not None:
            result = future.result()
            response.success = result.success
            response.message = result.message
            response.task_id = result.task_id
            
            if result.success:
                with self._lock:
                    self._task_state = TaskStateEnum.RUNNING
                    self._task_message = f"Started: {request.task_name}"
        else:
            response.success = False
            response.message = "Service call failed"
        
        return response
    
    def _stop_callback(self, request, response):
        """Task 중단/일시정지/재개/SAFE_STOP 복구"""
        stop_type = request.stop_type
        
        # PAUSE (3)
        if stop_type == 3:
            self.get_logger().info("Pause request")
            with self._lock:
                if self._task_state != TaskStateEnum.RUNNING:
                    response.success = False
                    response.message = "No task running to pause"
                    return response
        # RESUME (4)
        elif stop_type == 4:
            self.get_logger().info("Resume request")
            with self._lock:
                if self._task_state != TaskStateEnum.PAUSED:
                    response.success = False
                    response.message = "Task not paused"
                    return response
        # EMERGENCY (2): task_state 무관하게 항상 허용.
        # 사용자가 STANDBY(IDLE) 상태에서 긴급정지 버튼을 눌렀을 때도
        # motion_executor 가 servo_off 를 실제 DSR 에 전달해 실선반 빨간불 상태로
        # 들어갈 수 있어야 한다. 기존 else 분기는 "No task running" 으로 막고 있었음.
        elif stop_type == 2:
            self.get_logger().info("EMERGENCY stop request (unconditional)")
            with self._lock:
                self._task_state = TaskStateEnum.STOPPING
        # SAFE_STOP_RECOVER (5), TO_STANDBY (6), TO_RECOVERY (7), RECOVERY_DONE (8)
        # 어떤 task_state 에서도 허용. SAFE_STOP 은 motion 을 비정상 종료시켜
        # ERROR/IDLE 에 도달해 있는 것이 일반적이지만, RUNNING/PAUSED 가 남아 있는
        # 경우에도 motion_executor 가 깔끔하게 정리한다.
        elif stop_type in (5, 6, 7, 8):
            labels = {
                5: "SAFE_STOP_RECOVER (legacy)",
                6: "TO_STANDBY (자동 복구)",
                7: "TO_RECOVERY (수동 복구/무중력)",
                8: "RECOVERY_DONE (수동 복구 완료)",
            }
            self.get_logger().info(f"{labels.get(stop_type, stop_type)} request")
            with self._lock:
                self._task_state = TaskStateEnum.STOPPING
        # STOP (0, 1)
        else:
            self.get_logger().info(f"Stop request (type={stop_type})")
            with self._lock:
                if self._task_state not in (TaskStateEnum.RUNNING, TaskStateEnum.PAUSED):
                    response.success = False
                    response.message = "No task running"
                    return response
                self._task_state = TaskStateEnum.STOPPING
        
        # Executor에 요청 전달
        if not self._executor_stop_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "Task Executor not available"
            return response
        
        exec_request = StopTask.Request()
        exec_request.stop_type = stop_type
        future = self._executor_stop_client.call_async(exec_request)
        # 복구(5/6/7/8) / 긴급정지(2) 는 motion_executor 에서 여러 개의 DSR 서비스
        # 호출을 순차로 수행하므로 일반 stop(0/1/3/4) 보다 훨씬 긴 타임아웃 필요.
        # _handle_to_standby 는 DSR 의 실제 state 전이를 폴링하며 최대 ~20s 까지
        # 반복 시도하므로 컨트롤러 타임아웃도 그보다 여유 있게 설정.
        # (SAFE_STOP → SAFE_OFF → STANDBY 2-step 전이 + 관찰 지연 대비.)
        timeout_sec = 25.0 if stop_type in (2, 5, 6, 7, 8) else 2.0
        self._wait_future(future, timeout_sec=timeout_sec)

        if future.result() is not None:
            result = future.result()
            response.success = result.success
            response.message = result.message
            # 복구 성공 시 task_state 를 IDLE 로 리셋해 후속 StartTask 를
            # 받을 준비 상태로 만든다. executor 도 IDLE 로 자신의 state 를 내려주지만,
            # topic 전파 사이의 race 를 피하기 위해 여기서도 미리 갱신.
            # - 5, 6, 8: STANDBY 복구 완료 → IDLE
            # - 7: RECOVERY 모드 진입 → IDLE (task 실행은 안 하지만 대기 상태)
            if stop_type == 2 and result.success:
                with self._lock:
                    self._task_state = TaskStateEnum.IDLE
                    self._task_message = "Emergency stop — servo off"
            elif stop_type in (5, 6, 7, 8) and result.success:
                with self._lock:
                    self._task_state = TaskStateEnum.IDLE
                    messages = {
                        5: "SAFE_STOP recovered",
                        6: "Robot recovered to STANDBY",
                        7: "Robot in RECOVERY mode",
                        8: "Manual recovery completed",
                    }
                    self._task_message = messages.get(stop_type, "Recovery done")
        else:
            response.success = False
            response.message = "Service call failed"
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskControllerNode()

    # MultiThreadedExecutor 필수: _start_callback / _stop_callback 안에서
    # executor 쪽 서비스를 call_async + spin_until_future_complete 로 기다리므로
    # single-thread에서는 자기 콜백을 스스로 블로킹하는 데드락이 발생.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
