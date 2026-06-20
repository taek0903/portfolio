"""
Robot Status Node

`dsr_controller2` 의 monitoring 캐시 기반 서비스 4종을 10Hz 병렬 폴링해
`/{ns}/motion_executor/robot_status` (cobot_interfaces/RobotStatus) 로 발행.

## 왜 별도 프로세스·별도 노드인가

motion_executor 프로세스는 다음을 **모두** 한 프로세스 안에 가지고 있다.

- `ControlNode` + `MultiThreadedExecutor` (외부 I/F + DSR2 명령 client)
- `DsrNode` — DSR_ROBOT2 Python wrapper 가 `rclpy.spin_until_future_complete`
  으로 임시 SingleThreadedExecutor 를 잠깐 점유하는 구조
- task thread — DSR_ROBOT2 함수 직접 호출

이 조합에 상태 폴링 client 까지 얹으면 idle 상태에서도 client wait-set 경로가
응답을 못 받는 증상이 재현된다 (서버는 정상, `ros2 service call` CLI 는 즉답
오는데 motion_executor 만 `ok=0` 반복).

해결: **상태 폴링만 별도 프로세스**로 분리. DSR_ROBOT2 와 무관한 clean MT
executor 환경에서 돌아간다. 구독 경로는 기존과 동일하게 유지 해
subscriber (task_cli, task_controller, ui_bridge) 쪽은 수정 불필요.

## 선택적 폴링

launch argument 또는 ROS parameter로 추가 서비스를 활성화할 수 있음:
- poll_robot_state: bool (기본 True) - 필수, 비활성화 불가
- poll_posj: bool (기본 False) - 조인트 위치 폴링
- poll_tool_force: bool (기본 False) - 툴 힘 폴링
- poll_ext_torque: bool (기본 False) - 외부 토크 폴링

기본값은 robot_state만 폴링하여 초기 연결 안정성을 높임.
pos_log 등 추가 기능 구현 시 필요한 서비스를 활성화하면 됨.

Usage:
  ros2 run cobot1 robot_status_publisher --ros-args -p poll_posj:=true
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from cobot_interfaces.msg import RobotStatus
from dsr_msgs2.srv import (
    GetCurrentPosj,
    GetExternalTorque,
    GetRobotState,
    GetToolForce,
)


ROBOT_ID = "dsr01"
POLL_HZ = 10.0
WATCHDOG_NS = 2_000_000_000      # 2 s: 응답 안 오면 강제 재발사
WARN_PERIOD_NS = 5_000_000_000   # 5 s: "not ready" 재경고 주기
REPORT_PERIOD_NS = 5_000_000_000  # 5 s: 진단 요약 주기


# ─────────── slot spec ───────────
# 각 필드 별로 (service type, path suffix, request factory, result→cache adapter).
# RobotStatus 메시지 필드에 바로 꽂기 위한 adapter 는 `on_ok` 에 들어간다.

def _req_tool_force():
    req = GetToolForce.Request()
    req.ref = 0  # DR_BASE
    return req


class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__("robot_status_publisher", namespace=ROBOT_ID)
        self._cb = ReentrantCallbackGroup()

        # 선택적 폴링 파라미터 선언
        # 기본값은 robot_state만 폴링하여 초기 연결 안정성을 높임.
        # posj/tool_force/ext_torque는 pos_log 등 추가 기능 구현 시 활성화.
        self.declare_parameter("poll_robot_state", True)  # 필수 (항상 True)
        self.declare_parameter("poll_posj", False)
        self.declare_parameter("poll_tool_force", False)
        self.declare_parameter("poll_ext_torque", False)

        poll_posj = self.get_parameter("poll_posj").value
        poll_tool_force = self.get_parameter("poll_tool_force").value
        poll_ext_torque = self.get_parameter("poll_ext_torque").value

        # 퍼블리시 경로는 **절대** 경로로 기존 토픽 유지.
        # 구독자들(task_cli, task_controller)이 `motion_executor/robot_status`
        # 를 상대 경로로 구독 → `/dsr01/motion_executor/robot_status` 로 해석.
        self._pub = self.create_publisher(
            RobotStatus,
            f"/{ROBOT_ID}/motion_executor/robot_status",
            10,
        )

        # 슬롯 정의 (활성화된 것만)
        all_slots = [
            {
                "name": "get_robot_state",
                "path": f"/{ROBOT_ID}/system/get_robot_state",
                "type": GetRobotState,
                "make_req": GetRobotState.Request,
                "cache_key": "robot_state",
                "enabled": True,  # 항상 활성화 (필수)
            },
            {
                "name": "get_current_posj",
                "path": f"/{ROBOT_ID}/aux_control/get_current_posj",
                "type": GetCurrentPosj,
                "make_req": GetCurrentPosj.Request,
                "cache_key": "posj",
                "enabled": poll_posj,
            },
            {
                "name": "get_tool_force",
                "path": f"/{ROBOT_ID}/aux_control/get_tool_force",
                "type": GetToolForce,
                "make_req": _req_tool_force,
                "cache_key": "tool_force",
                "enabled": poll_tool_force,
            },
            {
                "name": "get_external_torque",
                "path": f"/{ROBOT_ID}/aux_control/get_external_torque",
                "type": GetExternalTorque,
                "make_req": GetExternalTorque.Request,
                "cache_key": "ext_torque",
                "enabled": poll_ext_torque,
            },
        ]

        self._slots = [s for s in all_slots if s["enabled"]]
        disabled_slots = [s["name"] for s in all_slots if not s["enabled"]]

        for slot in self._slots:
            slot["client"] = self.create_client(
                slot["type"], slot["path"], callback_group=self._cb,
            )
            slot["inflight"] = False
            slot["sent_ns"] = 0
            slot["warned"] = False
            slot["last_warn_ns"] = 0
            slot["ok_count"] = 0
            slot["fail_count"] = 0
            slot["last_ok_ns"] = 0

        # 필드 캐시. 마지막으로 성공한 응답의 값이 담긴다.
        # 비활성화된 서비스는 None 유지
        self._cache: dict[str, object] = {
            "robot_state": None,
            "posj": None,
            "tool_force": None,
            "ext_torque": None,
        }
        self._last_updated_ns = 0
        self._last_report_ns = 0

        self.create_timer(1.0 / POLL_HZ, self._tick, callback_group=self._cb)

        enabled_names = [s["name"] for s in self._slots]
        self.get_logger().info(
            f"robot_status_publisher ready. publishing /{ROBOT_ID}/motion_executor/robot_status @ {POLL_HZ:.0f}Hz"
        )
        self.get_logger().info(f"  enabled services: {enabled_names}")
        if disabled_slots:
            self.get_logger().info(f"  disabled services: {disabled_slots}")

    # ═══════════════════ main tick ═══════════════════
    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        self._publish(now_ns)
        self._dispatch(now_ns)
        self._maybe_report(now_ns)

    # ═══════════════════ dispatch ═══════════════════
    def _dispatch(self, now_ns: int):
        """4개 서비스를 **독립** 발사. 각 slot 의 inflight 래치로 서로 간섭 X."""
        for slot in self._slots:
            # Watchdog: 2s 지나도 응답 없으면 강제 리셋 후 재발사 허용.
            if slot["inflight"]:
                if now_ns - slot["sent_ns"] < WATCHDOG_NS:
                    continue
                self.get_logger().warn(
                    f"{slot['name']} response timeout "
                    f"({(now_ns - slot['sent_ns']) / 1e9:.1f}s); resetting"
                )
                slot["inflight"] = False
                slot["fail_count"] += 1

            if not slot["client"].service_is_ready():
                if (
                    not slot["warned"]
                    or now_ns - slot["last_warn_ns"] >= WARN_PERIOD_NS
                ):
                    self.get_logger().warn(f"{slot['path']} not ready yet")
                    slot["warned"] = True
                    slot["last_warn_ns"] = now_ns
                continue

            if slot["warned"]:
                self.get_logger().info(f"{slot['name']} service connected")
                slot["warned"] = False

            slot["inflight"] = True
            slot["sent_ns"] = now_ns
            fut = slot["client"].call_async(slot["make_req"]())
            # closure 로 slot 캡처. done_callback 은 rclpy thread 에서 invoke.
            fut.add_done_callback(
                lambda f, s=slot: self._on_result(s, f)
            )

    def _on_result(self, slot, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().debug(f"{slot['name']} failed: {e}")
            result = None
        ok = result is not None and getattr(result, "success", True)
        slot["inflight"] = False
        if ok:
            slot["ok_count"] += 1
            slot["last_ok_ns"] = self.get_clock().now().nanoseconds
            self._last_updated_ns = slot["last_ok_ns"]
            key = slot["cache_key"]
            if key == "robot_state":
                self._cache[key] = int(result.robot_state)
            elif key == "posj":
                self._cache[key] = list(result.pos)
            elif key == "tool_force":
                self._cache[key] = list(result.tool_force)
            elif key == "ext_torque":
                self._cache[key] = list(result.ext_torque)
        else:
            slot["fail_count"] += 1

    # ═══════════════════ publish ═══════════════════
    def _publish(self, now_ns: int):
        msg = RobotStatus()

        rs = self._cache["robot_state"]
        if rs is not None:
            msg.robot_state = int(rs)
            msg.robot_state_valid = True
        else:
            msg.robot_state = RobotStatus.ROBOT_STATE_UNKNOWN
            msg.robot_state_valid = False

        pj = self._cache["posj"]
        if pj is not None:
            msg.posj = list(pj)
            msg.posj_valid = True
        else:
            msg.posj = [0.0] * 6
            msg.posj_valid = False

        tf = self._cache["tool_force"]
        if tf is not None:
            msg.tool_force = list(tf)
            msg.tool_force_valid = True
        else:
            msg.tool_force = [0.0] * 6
            msg.tool_force_valid = False

        et = self._cache["ext_torque"]
        if et is not None:
            msg.external_joint_torque = list(et)
            msg.external_joint_torque_valid = True
        else:
            msg.external_joint_torque = [0.0] * 6
            msg.external_joint_torque_valid = False

        msg.last_updated_ns = self._last_updated_ns
        msg.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)

    # ═══════════════════ diagnostic log ═══════════════════
    def _maybe_report(self, now_ns: int):
        if now_ns - self._last_report_ns < REPORT_PERIOD_NS:
            return
        self._last_report_ns = now_ns
        parts = []
        for slot in self._slots:
            age_ms = (
                (now_ns - slot["last_ok_ns"]) / 1e6
                if slot["last_ok_ns"] else -1.0
            )
            parts.append(
                f"{slot['name']}: ok={slot['ok_count']} "
                f"fail={slot['fail_count']} age={age_ms:.0f}ms"
            )
        self.get_logger().info("[status poll] " + " | ".join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusPublisher()
    # 단일 노드만 있고 DSR_ROBOT2 경쟁이 없으므로 2 threads 면 충분.
    executor = MultiThreadedExecutor(num_threads=2)
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
