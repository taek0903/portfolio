"""
Sauce Task - 소스통을 집어 식판 위에서 반복 동작으로 소스를 짜고 반납.

기반: references/task_combine/sauce.py
좌표/순서/그리퍼 DO 매핑(rice/tong 과 다름)/wait 시간/move_periodic 파라미터/
TCP·Tool·vel·acc 는 원본과 1:1 동일.

변경점:
- movej/movel → amovej/amovel + ctx.wait_motion() (pause/resume 지원)
- move_periodic → amove_periodic + ctx.wait_motion() (pause/resume 지원)
- 그리퍼 DO 전이: `_common.gripper_transition` (transient-safe) 사용.
"""

from ._common import initialize_robot, gripper_transition


TASK_NAME = "sauce"
TASK_DESCRIPTION = "소스통을 들고 식판 위에서 소스 짜기 후 반납"
TASK_LABEL = "소스"

ROBOT_TCP = "GripperDA_v1"     # rice/tong 과 통일 (reference 의 "GripperDA" 에서 변경)
ROBOT_TOOL = "Tool Weight"
VELOCITY = 300
ACC = 80


# ───────────────────── Gripper ─────────────────────
# reference task_combine/sauce.py 도 tong 과 동일하게 DO1→DO2→DO3 를 순차 set
# 하는데, 전이 중간에 원치 않는 상태를 거쳐 소스통이 미세하게 흔들리는 사례가
# 있어 _common.gripper_transition 으로 통일. bit_delay/settle 값은 sauce 의
# 기존 wait_time 파라미터와 호환되게 전달한다.
def _gripper(action: str, ctx, *, wait_time: float = 3.0):
    """references/task_combine/sauce.py 의 gripper() 와 DO 매핑 동일.

    sauce (소스통 파지, 68mm): DO1=0, DO2=0, DO3=1
    TIGHT (소스 짜기, 50mm)  : DO1=1, DO2=1, DO3=1
    GRIP_RELEASE             : DO1=0, DO2=1, DO3=0

    wait_time:
        설정 후 settle 대기 시간 (초). 기본 3.0. "흔들기 + 짜기 동시 시작"
        구간에서는 wait_time=0.0 으로 호출해 DO 만 set 하고 즉시 반환
        (reference 의 `gripper("TIGHT", wait_time=0.0)` 패턴).

    중단 정책 (2026-04-22 — "holding failsafe"):
        safe_wait 중 stop/fault 감지 시 **DO 를 GRIP_RELEASE 로 강제하지 않는다**.
        현재 DO 상태 그대로 유지 → 소스통 낙하 방지.
        복구 시에는 `task_gripper_open` (recovery_* 첫 모듈) 이 명시적으로 연다.
    """
    if action == "sauce":
        target = [0, 0, 1]
    elif action == "TIGHT":
        target = [1, 1, 1]
    elif action == "GRIP_RELEASE":
        target = [0, 1, 0]
    else:
        return False

    return gripper_transition(ctx, target, bit_delay=0.1, settle=wait_time)


# ───────────────────── STEPS ─────────────────────
STEPS = [
    ("init",            "로봇 초기화"),
    ("ready_1",         "JReady (원위치)"),
    ("release_1",       "그리퍼 열기 (GRIP_RELEASE)"),
    ("source_above",    "소스통 위 (pos1)"),
    ("source_down",     "소스통 잡기 직전 (pos2)"),
    ("grip_source",     "소스통 잡기 (sauce, 68mm)"),
    ("source_lift",     "소스통 들어올리기 (pos1)"),
    ("to_plate_above",  "식판 상공 (pos3)"),

    # "흔들기 + 짜기 동시 시작" 구간 (references/task_combine/sauce.py 최신판)
    ("shake_start",     "흔들기 시작 (amove_periodic)"),
    ("grip_tight",      "소스 짜기 (TIGHT, 즉시)"),
    ("shake_wait",      "흔들기 완료 대기"),

    ("grip_back",       "파지 복귀 (sauce, 68mm)"),
    ("return_above",    "반납 위 (pos1)"),
    ("return_down",     "반납 내려놓기 (pos2)"),
    ("release_source",  "소스통 놓기 (GRIP_RELEASE)"),
    ("return_lift",     "반납 위 복귀 (pos1)"),
    ("ready_final",     "최종 JReady"),
]

_STEP_INDEX = {name: i + 1 for i, (name, _) in enumerate(STEPS)}
_STEP_DESC = {name: desc for name, desc in STEPS}
_TOTAL = len(STEPS)


def _step(ctx, name: str) -> bool:
    if ctx.check_stop():
        return False
    ctx.update_progress(_STEP_INDEX[name], _TOTAL, name, _STEP_DESC[name])
    return True


# ───────────────────── run ─────────────────────
def run(ctx):
    from DSR_ROBOT2 import (
        posx, amovej, amovel, amove_periodic, DR_BASE,
    )

    # 위치 좌표 (references/task_combine/sauce.py 와 1:1 동일)
    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([371.54, -262.40, 307.33, 134.64,  179.61, 135.07])  # 소스통 위
    pos2 = posx([371.35, -262.77, 173.15, 138.52,  179.50, 139.01])  # 소스통 집기 직전
    pos3 = posx([652.17,  -19.67, 286.20,   8.13,  156.13, -22.47])  # 소스통 바로 위 (식판)

    try:
        if not _step(ctx, "init"): return False, "Task stopped by user"
        initialize_robot(tcp=ROBOT_TCP, tool=ROBOT_TOOL)

        if not _step(ctx, "ready_1"): return False, "Task stopped by user"
        amovej(JReady, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "release_1"): return False, "Task stopped by user"
        if not _gripper("GRIP_RELEASE", ctx): return False, "Task stopped by user"

        if not _step(ctx, "source_above"): return False, "Task stopped by user"
        amovel(pos1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "source_down"): return False, "Task stopped by user"
        amovel(pos2, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "grip_source"): return False, "Task stopped by user"
        if not _gripper("sauce", ctx): return False, "Task stopped by user"

        if not _step(ctx, "source_lift"): return False, "Task stopped by user"
        amovel(pos1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "to_plate_above"): return False, "Task stopped by user"
        amovel(pos3, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        # ═════════════ 흔들기 + 짜기 "동시 시작" ═════════════
        # reference/task_combine/sauce.py 최신판의 구조:
        #   amove_periodic(...)                # 흔들기 비동기 시작
        #   gripper("TIGHT", wait_time=0.0)    # DO 만 set, 대기 없음
        #   wait(10.2)                         # 흔들기 끝까지 대기
        # 우리는 마지막 wait(10.2) 대신 ctx.wait_motion() 으로 실제 amove_periodic
        # 완료 시점까지 pause/stop 지원하며 대기 (reference 의 10.2 s 상수보다 정확).
        if not _step(ctx, "shake_start"): return False, "Task stopped by user"
        amove_periodic(
            amp=[20, 25, 0, 0, 0, 0],
            period=[1.6, 3.2, 1.6, 0, 0, 0],
            atime=3.1,
            repeat=2,
            ref=DR_BASE,
        )

        # 흔들기 시작과 동시에 그리퍼를 TIGHT 으로 전환. wait_time=0 으로 즉시 반환.
        if not _step(ctx, "grip_tight"): return False, "Task stopped by user"
        if not _gripper("TIGHT", ctx, wait_time=0.0): return False, "Task stopped by user"

        # 흔들기 완료 대기 (pause/stop 지원)
        if not _step(ctx, "shake_wait"): return False, "Task stopped by user"
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "grip_back"): return False, "Task stopped by user"
        if not _gripper("sauce", ctx): return False, "Task stopped by user"

        if not _step(ctx, "return_above"): return False, "Task stopped by user"
        amovel(pos1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "return_down"): return False, "Task stopped by user"
        amovel(pos2, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "release_source"): return False, "Task stopped by user"
        if not _gripper("GRIP_RELEASE", ctx): return False, "Task stopped by user"

        if not _step(ctx, "return_lift"): return False, "Task stopped by user"
        amovel(pos1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "ready_final"): return False, "Task stopped by user"
        amovej(JReady, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        return True, "Sauce task completed"

    except Exception as e:
        if ctx.check_stop():
            return False, "Task stopped by user"
        return False, f"Sauce task error: {e}"
