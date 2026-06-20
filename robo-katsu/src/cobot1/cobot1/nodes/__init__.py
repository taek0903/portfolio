"""cobot1 ROS2 nodes.

각 노드는 `setup.py` `entry_points` 로 `ros2 run cobot1 <executable>` 형태로 실행된다.

- motion_executor        : DSR2 명령 실행 + task thread
- robot_status_publisher : DSR2 monitoring 서비스 폴링 → RobotStatus 토픽
- task_controller        : 외부 UI ↔ motion_executor 중계 + 통합 상태 머신
- ui_bridge              : 웹 UI ↔ ROS 브릿지 + Firebase Realtime Database sync
- task_cli               : 터미널용 일회성 CLI 클라이언트
"""
