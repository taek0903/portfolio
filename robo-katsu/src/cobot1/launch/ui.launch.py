"""
UI Launch File (Device 2 — UI/원격 머신)

디바이스2(UI/원격)에서 필요한 3개 프로세스를 한 번에 기동한다:

- **task_controller**      : 외부 UI ↔ motion_executor 중계 (ROS2 서비스/토픽)
- **ui_bridge**            : Firebase 동기화 + KST 당일 카운트·에러 bootstrap
- **rosbridge_websocket**  : 웹 UI 가 ROS 에 연결하기 위한 WebSocket 서버 (9090)

디바이스1(로봇 근접)에서는 여전히 `task_system.launch.py` 로 `motion_executor`
+ `robot_status_publisher` 만 띄운다.

Usage:
  # 가상 모드 (에뮬레이터 옆에서 테스트)
  ros2 launch cobot1 ui.launch.py mode:=virtual

  # 실기 모드 (기본)
  ros2 launch cobot1 ui.launch.py

  # 포트/네임스페이스 커스텀
  ros2 launch cobot1 ui.launch.py robot_namespace:=dsr01 rosbridge_port:=9090
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "robot_namespace", default_value="dsr01",
        description="Robot namespace (matches device1 motion_executor)",
    )
    mode_arg = DeclareLaunchArgument(
        "mode", default_value="real",
        description="ui_bridge mode: 'virtual' or 'real' (Firebase 이벤트 필터 기준)",
    )
    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port", default_value="9090",
        description="rosbridge_websocket listen port",
    )

    ns = LaunchConfiguration("robot_namespace")
    mode = LaunchConfiguration("mode")
    rosbridge_port = LaunchConfiguration("rosbridge_port")

    task_controller = Node(
        package="cobot1",
        executable="task_controller",
        name="task_controller",
        namespace=ns,
        output="screen",
        emulate_tty=True,
    )

    # ui_bridge 는 루트 namespace 로 띄워 웹 UI 가 /status 를 바로 구독할 수 있게 한다.
    # 내부에서 ROBOT_NAMESPACE (dsr01) 하위 토픽을 직접 구독/발행.
    ui_bridge = Node(
        package="cobot1",
        executable="ui_bridge",
        name="ui_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[{"mode": mode}],
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml",
            )
        ),
        launch_arguments={"port": rosbridge_port}.items(),
    )

    return LaunchDescription([
        namespace_arg,
        mode_arg,
        rosbridge_port_arg,
        task_controller,
        ui_bridge,
        rosbridge,
    ])
