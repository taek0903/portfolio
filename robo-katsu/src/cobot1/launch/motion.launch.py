"""
Motion Launch File (Device 1 — 로봇 근접 머신)

디바이스1(로봇이 물리적으로 연결된 머신)에서 필요한 2개 프로세스를 한 번에 기동한다:

- **motion_executor**        : DSR2 명령·태스크 실행 (DSR_ROBOT2 wrapper + task thread)
- **robot_status_publisher** : DSR2 monitoring 서비스 4종 폴링 → robot_status 토픽

디바이스2(UI/원격) 에서는 `ui.launch.py` 로 `task_controller + ui_bridge + rosbridge`
를 띄운다. 단일 머신 개발 시에는 `task_system.launch.py` 를 사용해도 된다.

Usage:
  # 기본 (dsr01, 폴링은 robot_state 만)
  ros2 launch cobot1 motion.launch.py

  # 추가 폴링 활성화
  ros2 launch cobot1 motion.launch.py poll_posj:=true poll_tool_force:=true poll_ext_torque:=true

  # namespace 커스텀
  ros2 launch cobot1 motion.launch.py robot_namespace:=dsr01
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "robot_namespace", default_value="dsr01",
        description="Robot namespace (matches motion_executor ROBOT_ID)",
    )
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
    )

    robot_status_publisher = Node(
        package="cobot1",
        executable="robot_status_publisher",
        name="robot_status_publisher",
        namespace=ns,
        output="screen",
        emulate_tty=True,
        parameters=[{
            "poll_posj": LaunchConfiguration("poll_posj"),
            "poll_tool_force": LaunchConfiguration("poll_tool_force"),
            "poll_ext_torque": LaunchConfiguration("poll_ext_torque"),
        }],
    )

    return LaunchDescription([
        namespace_arg,
        poll_posj_arg,
        poll_tool_force_arg,
        poll_ext_torque_arg,
        motion_executor,
        robot_status_publisher,
    ])
