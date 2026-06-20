"""
Task System Launch File

세 개 노드를 개별 프로세스로 띄운다 — 역할 분리 + rmw 경합 방지.

- motion_executor        : DSR2 명령·태스크 실행 (dsr_node + task thread)
- robot_status_publisher : DSR2 monitoring 서비스 4종 폴링 → robot_status 토픽
- task_controller        : 외부 UI ↔ motion_executor 중계

배포 시나리오:
  - 디바이스1 (로봇 근접): motion_executor + robot_status_publisher
  - 디바이스2 (UI/원격):    task_controller
  - 개발/단일 머신:        전부 (기본값)

Usage:
  ros2 launch cobot1 task_system.launch.py
  ros2 launch cobot1 task_system.launch.py executor:=true status:=true controller:=false
  ros2 launch cobot1 task_system.launch.py executor:=false status:=false controller:=true

robot_status_publisher 선택적 폴링 (추가 기능용):
  ros2 launch cobot1 task_system.launch.py poll_posj:=true poll_tool_force:=true poll_ext_torque:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    executor_arg = DeclareLaunchArgument(
        "executor", default_value="true",
        description="Run motion_executor on this device",
    )
    status_arg = DeclareLaunchArgument(
        "status", default_value="true",
        description="Run robot_status_publisher on this device",
    )
    controller_arg = DeclareLaunchArgument(
        "controller", default_value="true",
        description="Run task_controller on this device",
    )
    namespace_arg = DeclareLaunchArgument(
        "robot_namespace", default_value="dsr01",
        description="Robot namespace",
    )

    # robot_status_publisher 선택적 폴링 인자
    # 기본값 false: robot_state만 폴링하여 초기 연결 안정성 확보
    # pos_log 등 추가 기능 필요 시 true로 설정
    poll_posj_arg = DeclareLaunchArgument(
        "poll_posj", default_value="false",
        description="Poll joint positions (enable for pos_log features)",
    )
    poll_tool_force_arg = DeclareLaunchArgument(
        "poll_tool_force", default_value="false",
        description="Poll tool force (enable for force monitoring)",
    )
    poll_ext_torque_arg = DeclareLaunchArgument(
        "poll_ext_torque", default_value="false",
        description="Poll external torque (enable for torque monitoring)",
    )

    ns = LaunchConfiguration("robot_namespace")

    motion_executor = Node(
        package="cobot1",
        executable="motion_executor",
        name="motion_executor",
        namespace=ns,
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("executor")),
    )

    robot_status_publisher = Node(
        package="cobot1",
        executable="robot_status_publisher",
        name="robot_status_publisher",
        namespace=ns,
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("status")),
        parameters=[{
            "poll_posj": LaunchConfiguration("poll_posj"),
            "poll_tool_force": LaunchConfiguration("poll_tool_force"),
            "poll_ext_torque": LaunchConfiguration("poll_ext_torque"),
        }],
    )

    task_controller = Node(
        package="cobot1",
        executable="task_controller",
        name="task_controller",
        namespace=ns,
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("controller")),
    )

    return LaunchDescription([
        executor_arg,
        status_arg,
        controller_arg,
        namespace_arg,
        poll_posj_arg,
        poll_tool_force_arg,
        poll_ext_torque_arg,
        motion_executor,
        robot_status_publisher,
        task_controller,
    ])
