"""
Home Task - 홈 위치(JReady)로 복귀.

references/tong.py, references/source.py 공통으로 쓰이는 JReady를 그대로 사용.
단, movej → amovej 로 변경 (pause/resume 지원).
"""

from ._common import initialize_robot


TASK_NAME = "home"
TASK_DESCRIPTION = "홈 위치로 이동"
TASK_LABEL = "홈 복귀"

ROBOT_TCP = "GripperDA_v1"
ROBOT_TOOL = "Tool Weight"
VELOCITY = 60
ACC = 60

JReady = [0, 0, 90, 0, 90, 0]

STEPS = [
    ("init",         "로봇 초기화"),
    ("move_to_home", "홈 위치로 이동"),
]


def run(ctx):
    from DSR_ROBOT2 import amovej

    total = len(STEPS)

    if ctx.check_stop(): return False, "Task stopped by user"
    ctx.update_progress(1, total, "init", "로봇 초기화 중...")
    initialize_robot(tcp=ROBOT_TCP, tool=ROBOT_TOOL)

    if ctx.check_stop(): return False, "Task stopped by user"
    ctx.update_progress(2, total, "move_to_home", "홈 위치로 이동")
    amovej(JReady, vel=VELOCITY, acc=ACC)
    if not ctx.wait_motion():
        return False, "Task stopped by user"

    return True, "Home position reached"
