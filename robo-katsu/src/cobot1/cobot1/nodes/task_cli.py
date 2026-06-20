"""
Task CLI - pause/resume 지원

Usage:
  ros2 run cobot1 task_cli start <task_name>
  ros2 run cobot1 task_cli stop
  ros2 run cobot1 task_cli pause
  ros2 run cobot1 task_cli resume
  ros2 run cobot1 task_cli status
"""

import rclpy
from rclpy.node import Node
import sys

from cobot_interfaces.msg import RobotStatus, TaskState
from cobot_interfaces.srv import StartTask, StopTask


ROBOT_ID = "dsr01"


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: task_cli <start|stop|pause|resume|status> [task_name]")
        return
    
    cmd = sys.argv[1]
    
    rclpy.init(args=args)
    node = rclpy.create_node("task_cli", namespace=ROBOT_ID)
    
    try:
        if cmd == "start":
            if len(sys.argv) < 3:
                print("Usage: task_cli start <task_name>")
                return
            
            task_name = sys.argv[2]
            client = node.create_client(StartTask, 'task/start')
            
            if not client.wait_for_service(timeout_sec=3.0):
                print("Error: Task Controller not available")
                return
            
            req = StartTask.Request()
            req.task_name = task_name
            
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            
            if future.result():
                r = future.result()
                print(f"{'OK' if r.success else 'FAIL'}: {r.message}")
            else:
                print("Error: Service call failed")
        
        elif cmd in ("stop", "pause", "resume"):
            client = node.create_client(StopTask, 'task/stop')
            
            if not client.wait_for_service(timeout_sec=3.0):
                print("Error: Task Controller not available")
                return
            
            req = StopTask.Request()
            if cmd == "stop":
                req.stop_type = 0
            elif cmd == "pause":
                req.stop_type = 3  # PAUSE
            elif cmd == "resume":
                req.stop_type = 4  # RESUME
            
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            
            if future.result():
                r = future.result()
                print(f"{'OK' if r.success else 'FAIL'}: {r.message}")
            else:
                print("Error: Service call failed")
        
        elif cmd == "status":
            task_state_names = {0: "IDLE", 1: "RUNNING", 2: "STOPPING", 3: "ERROR", 4: "PAUSED"}
            robot_state_names = {
                0: "INITIALIZING", 1: "STANDBY", 2: "MOVING", 3: "SAFE_OFF",
                4: "TEACHING", 5: "SAFE_STOP", 6: "EMERGENCY_STOP",
                7: "HOMMING", 8: "RECOVERY", 9: "SAFE_STOP2", 10: "SAFE_OFF2",
                15: "NOT_READY", 255: "UNKNOWN",
            }

            task_msg = [None]
            robot_msg = [None]
            node.create_subscription(TaskState, 'task/state', lambda m: task_msg.__setitem__(0, m), 10)
            node.create_subscription(
                RobotStatus, 'motion_executor/robot_status',
                lambda m: robot_msg.__setitem__(0, m), 10,
            )

            # 두 토픽 모두 ~10Hz 발행되므로 최대 2초 대기.
            deadline_iters = 20
            for _ in range(deadline_iters):
                rclpy.spin_once(node, timeout_sec=0.1)
                if task_msg[0] is not None and robot_msg[0] is not None:
                    break

            def fmt6(arr):
                return "[" + ", ".join(f"{v:7.2f}" for v in arr) + "]"

            def fmt_field(valid, arr):
                return fmt6(arr) if valid else "(no data)"

            print("=" * 56)
            print("TASK STATE (task_controller → task/state):")
            if task_msg[0]:
                s = task_msg[0]
                print(f"  State    : {task_state_names.get(s.state, f'UNKNOWN({s.state})')}")
                if s.message:
                    print(f"  Message  : {s.message}")
                if s.progress > 0:
                    print(f"  Progress : {s.progress * 100:.1f}%")
            else:
                print("  (No data — task_controller가 떠있는지 확인)")

            print("-" * 56)
            print("ROBOT STATUS (motion_executor → motion_executor/robot_status):")
            if robot_msg[0]:
                r = robot_msg[0]
                if r.robot_state_valid:
                    label = robot_state_names.get(r.robot_state, f'UNKNOWN({r.robot_state})')
                else:
                    label = "(no data)"
                print(f"  State    : {label}")
                print(f"  Joint(°) : {fmt_field(r.posj_valid, r.posj)}")
                print(f"  ToolFrc  : {fmt_field(r.tool_force_valid, r.tool_force)}")
                print(f"  ExtTrq   : {fmt_field(r.external_joint_torque_valid, r.external_joint_torque)}")
            else:
                print("  (No data — motion_executor 가 떠있는지 확인)")
            print("=" * 56)
        
        else:
            print(f"Unknown command: {cmd}")
            print("Usage: task_cli <start|stop|pause|resume|status> [task_name]")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
