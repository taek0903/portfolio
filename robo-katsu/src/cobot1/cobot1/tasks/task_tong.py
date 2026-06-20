"""
Tong Task - 집게 집기 → 샐러드 배식 → 돈까스 1 배식 → 돈까스 2 배식 → 집게 반납.

기반: references/task_combine/tong.py
좌표/순서/그리퍼 DO 매핑/wait 시간/TCP·Tool·vel·acc 는 원본과 1:1 동일.

변경점:
- movej/movel → amovej/amovel + ctx.wait_motion() (pause/resume 지원)
- 그리퍼 DO 전이: `_common.gripper_transition` (transient-safe) 사용.
  - reference 의 순차 set_digital_output 은 RELEASE→GRIP_BASIC 중간에
    (1,1,0) = GRIP_TIGHT(0mm) 함정 상태를 거쳐 oscillation 이 관찰됐다.
  - 1→0 비트를 먼저, 0→1 비트를 나중에 write 해 함정 상태를 회피.
"""

from ._common import initialize_robot, gripper_transition


TASK_NAME = "tong"
TASK_DESCRIPTION = "집게 → 샐러드/돈까스1/돈까스2 배식 → 집게 반납"
TASK_LABEL = "샐러드·돈까스"

ROBOT_TCP = "GripperDA_v1"
ROBOT_TOOL = "Tool Weight"
VELOCITY = 300
ACC = 80


# ───────────────────── Gripper ─────────────────────
# reference task_combine/tong.py 는 DO1/DO2/DO3 를 순차적으로 set_digital_output
# 하는데, RELEASE(0,1,0) → GRIP_BASIC(1,0,0) 전이 시 `DO1=1` 이 먼저 찍힌
# 순간 (1,1,0) = GRIP_TIGHT (0mm 완전 압착) 를 0.x s 동안 거치게 된다.
# 공압 응답 시간 때문에 실제로 강하게 쪼여지는 순간 oscillation 으로 관찰됐다.
# references/debug_gripper/tong_fixed.py 의 "bit 1→0 먼저, 0→1 마지막" 패턴을
# _common.gripper_transition 으로 일반화해 이식.
def _gripper(action: str, ctx):
    """references/task_combine/tong.py 의 gripper() 와 DO 매핑 동일.

    GRIP_BASIC (35mm): DO1=1, DO2=0, DO3=0
    RELEASE    (75mm): DO1=0, DO2=1, DO3=0
    GRIP_TIGHT (0mm) : DO1=1, DO2=1, DO3=0

    전이 정책 — transient-safe:
        `_common.gripper_transition` 가 현재 DO tracker 와 target 을 비교해
        1→0 비트를 먼저, 0→1 비트를 나중에 쓴다. 이로 인해 RELEASE→GRIP_BASIC
        전이가 (1,1,0) 함정 상태를 거치지 않는다.

    중단 정책 (2026-04-22 — "holding failsafe"):
        safe_wait 중 stop/fault 감지 시 **DO 를 RELEASE 로 강제하지 않는다**.
        현재 DO 상태 그대로 유지 → 쥐고 있던 payload 낙하 방지.
        복구 시에는 `task_gripper_open` (recovery_* 첫 모듈) 이 명시적으로 연다.
    """
    if action == "GRIP_BASIC":
        target = [1, 0, 0]
    elif action == "RELEASE":
        target = [0, 1, 0]
    elif action == "GRIP_TIGHT":
        target = [1, 1, 0]
    else:
        return False

    return gripper_transition(ctx, target, bit_delay=0.1, settle=1.5)


# ───────────────────── STEPS ─────────────────────
STEPS = [
    ("init",                "로봇 초기화"),
    ("ready_1",             "JReady"),
    ("release_1",           "그리퍼 열기"),
    ("tong_above_1",        "집게 위 (pos1)"),
    ("tong_down_1",         "집게 접근 (pos2)"),
    ("grip_tong",           "집게 파지 (GRIP_BASIC)"),
    ("tong_lift",           "집게 들어올리기 (pos1)"),
    ("ready_2",             "JReady"),

    # 샐러드 (reference tong.py 최신판: 털기 모션 제거, pos12 중간 경유로만 사용)
    ("salad_above",         "샐러드 위 (pos10)"),
    ("salad_down",          "샐러드 아래 (pos11)"),
    ("grip_salad",          "샐러드 집기 (GRIP_TIGHT)"),
    ("salad_slight_up",     "샐러드 들어올리기 중간 경유 (pos12)"),
    ("salad_up",            "샐러드 위 복귀 (pos10)"),
    ("plate_above_salad",   "식판 공중 (pos6_2)"),
    ("plate_drop_salad",    "식판 아래 (pos7_2) — 샐러드 놓기 직전"),
    ("release_salad",       "샐러드 놓기 (GRIP_BASIC)"),
    ("plate_above_salad_2", "식판 공중 복귀 (pos6_2)"),
    ("salad_home",          "샐러드 위 복귀 (pos10)"),
    ("ready_3",             "JReady"),

    # 돈까스 1
    ("pork1_above",         "돈까스1 위 (pos3)"),
    ("pork1_down",          "돈까스1 아래 (pos4)"),
    ("grip_pork1",          "돈까스1 집기 (GRIP_TIGHT)"),
    ("pork1_lift",          "돈까스1 들어올리기 (pos3)"),
    ("pork1_to_mid",        "중간 경유 (pos5)"),
    ("pork1_plate_above",   "식판 한참 위 (pos6)"),
    ("pork1_plate_drop",    "식판 바로 위 (pos7)"),
    ("release_pork1",       "돈까스1 놓기 (GRIP_BASIC)"),
    ("pork1_plate_back",    "식판 한참 위 복귀 (pos6)"),
    ("pork1_mid_back",      "중간 복귀 (pos5)"),

    # 돈까스 2
    ("pork2_above",         "돈까스2 위 (pos8)"),
    ("pork2_down",          "돈까스2 아래 (pos9)"),
    ("grip_pork2",          "돈까스2 집기 (GRIP_TIGHT)"),
    ("pork2_lift",          "돈까스2 들어올리기 (pos8)"),
    ("pork2_mid",           "중간 경유 (pos5)"),
    ("pork2_plate_above",   "식판 공중 (pos6_1)"),
    ("pork2_plate_drop",    "식판 바로 위 (pos7_1)"),
    ("release_pork2",       "돈까스2 놓기 (GRIP_BASIC)"),
    ("pork2_plate_back",    "식판 공중 복귀 (pos6_1)"),
    ("pork2_mid_back",      "중간 복귀 (pos5)"),

    # 집게 반납
    ("tong_above_2",        "집게 반납 위 (pos1)"),
    ("tong_down_2",         "집게 내려놓기 (pos2)"),
    ("release_tong",        "집게 반납 (RELEASE)"),
    ("tong_above_3",        "집게 위 복귀 (pos1)"),
    ("ready_final",         "최종 JReady"),
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
    from DSR_ROBOT2 import posx, amovej, amovel

    # 위치 좌표 (references/task_combine/tong.py 와 1:1 동일)
    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([273.96, -248.55, 314.37, 128.50,  179.98, 128.52])  # 집게 위
    pos2 = posx([275.48, -252.13, 148.32, 136.44,  179.77, 136.72])  # 집게 아래

    # 1번 돈까스
    pos3 = posx([467.28,  159.75, 329.71,  22.73,  176.34,  23.24])  # 돈까스1 위
    pos4 = posx([477.64,  163.29, 200.25,  23.11,  176.60,  23.72])  # 돈까스1 아래

    # 식판 경유/배치 (돈까스1)
    pos5 = posx([453.93,   11.44, 338.32, 112.20,  177.64, 112.11])  # 식판 중간 경유
    pos6 = posx([652.65,   16.23, 317.19,   4.07,  157.35,   3.72])  # 식판 한참 위
    pos7 = posx([669.94,   16.05, 290.75,   3.88,  156.47,   3.66])  # 식판 바로 위

    # 2번 돈까스
    pos8 = posx([520.80,  161.72, 327.75, 104.54,  178.06, 104.91])  # 돈까스2 위
    pos9 = posx([523.32,  169.27, 198.30, 106.93,  177.95, 107.49])  # 돈까스2 아래

    # 식판 배치 (돈까스2)
    pos6_1 = posx([646.89,  -28.32, 307.62,   0.03,  147.99,   3.56])   # 식판 공중
    pos7_1 = posx([638.04,  -32.76, 252.58, 178.53, -152.99, -178.09])  # 식판 바로 위

    # 샐러드
    pos10 = posx([497.34, -169.84, 296.22, 155.26, -136.53, 178.28])  # 샐러드 공중
    pos11 = posx([530.00, -230.28, 137.70, 144.66, -155.31, 168.53])  # 샐러드 아래
    pos12 = posx([530.00, -230.28, 237.70, 144.66, -155.31, 168.53])  # 샐러드 들어올리기 중간 경유

    # 샐러드용 식판
    pos6_2 = posx([724.82,   17.10, 283.40,   0.87,  150.10,  11.78])  # 식판 공중
    pos7_2 = posx([748.22,   17.51, 236.21,   0.86,  151.72,  11.77])  # 식판 아래

    try:
        # 초기화
        if not _step(ctx, "init"): return False, "Task stopped by user"
        initialize_robot(tcp=ROBOT_TCP, tool=ROBOT_TOOL)

        # ═════════════ 집게 집기 ═════════════
        if not _step(ctx, "ready_1"): return False, "Task stopped by user"
        amovej(JReady, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "release_1"): return False, "Task stopped by user"
        if not _gripper("RELEASE", ctx): return False, "Task stopped by user"

        if not _step(ctx, "tong_above_1"): return False, "Task stopped by user"
        amovel(pos1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "tong_down_1"): return False, "Task stopped by user"
        amovel(pos2, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "grip_tong"): return False, "Task stopped by user"
        if not _gripper("GRIP_BASIC", ctx): return False, "Task stopped by user"

        if not _step(ctx, "tong_lift"): return False, "Task stopped by user"
        amovel(pos1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "ready_2"): return False, "Task stopped by user"
        amovej(JReady, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        # ═════════════ 샐러드 ═════════════
        if not _step(ctx, "salad_above"): return False, "Task stopped by user"
        amovel(pos10, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "salad_down"): return False, "Task stopped by user"
        amovel(pos11, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "grip_salad"): return False, "Task stopped by user"
        if not _gripper("GRIP_TIGHT", ctx): return False, "Task stopped by user"

        if not _step(ctx, "salad_slight_up"): return False, "Task stopped by user"
        amovel(pos12, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "salad_up"): return False, "Task stopped by user"
        amovel(pos10, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "plate_above_salad"): return False, "Task stopped by user"
        amovel(pos6_2, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "plate_drop_salad"): return False, "Task stopped by user"
        amovel(pos7_2, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "release_salad"): return False, "Task stopped by user"
        if not _gripper("GRIP_BASIC", ctx): return False, "Task stopped by user"

        if not _step(ctx, "plate_above_salad_2"): return False, "Task stopped by user"
        amovel(pos6_2, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "salad_home"): return False, "Task stopped by user"
        amovel(pos10, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "ready_3"): return False, "Task stopped by user"
        amovej(JReady, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        # ═════════════ 돈까스 1 ═════════════
        if not _step(ctx, "pork1_above"): return False, "Task stopped by user"
        amovel(pos3, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "pork1_down"): return False, "Task stopped by user"
        amovel(pos4, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "grip_pork1"): return False, "Task stopped by user"
        if not _gripper("GRIP_TIGHT", ctx): return False, "Task stopped by user"

        if not _step(ctx, "pork1_lift"): return False, "Task stopped by user"
        amovel(pos3, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "pork1_to_mid"): return False, "Task stopped by user"
        amovel(pos5, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "pork1_plate_above"): return False, "Task stopped by user"
        amovel(pos6, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "pork1_plate_drop"): return False, "Task stopped by user"
        amovel(pos7, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "release_pork1"): return False, "Task stopped by user"
        if not _gripper("GRIP_BASIC", ctx): return False, "Task stopped by user"

        if not _step(ctx, "pork1_plate_back"): return False, "Task stopped by user"
        amovel(pos6, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "pork1_mid_back"): return False, "Task stopped by user"
        amovel(pos5, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        # ═════════════ 돈까스 2 ═════════════
        if not _step(ctx, "pork2_above"): return False, "Task stopped by user"
        amovel(pos8, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "pork2_down"): return False, "Task stopped by user"
        amovel(pos9, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "grip_pork2"): return False, "Task stopped by user"
        if not _gripper("GRIP_TIGHT", ctx): return False, "Task stopped by user"

        if not _step(ctx, "pork2_lift"): return False, "Task stopped by user"
        amovel(pos8, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "pork2_mid"): return False, "Task stopped by user"
        amovel(pos5, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "pork2_plate_above"): return False, "Task stopped by user"
        amovel(pos6_1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "pork2_plate_drop"): return False, "Task stopped by user"
        amovel(pos7_1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "release_pork2"): return False, "Task stopped by user"
        if not _gripper("GRIP_BASIC", ctx): return False, "Task stopped by user"

        if not _step(ctx, "pork2_plate_back"): return False, "Task stopped by user"
        amovel(pos6_1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "pork2_mid_back"): return False, "Task stopped by user"
        amovel(pos5, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        # ═════════════ 집게 반납 ═════════════
        if not _step(ctx, "tong_above_2"): return False, "Task stopped by user"
        amovel(pos1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "tong_down_2"): return False, "Task stopped by user"
        amovel(pos2, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "release_tong"): return False, "Task stopped by user"
        if not _gripper("RELEASE", ctx): return False, "Task stopped by user"

        if not _step(ctx, "tong_above_3"): return False, "Task stopped by user"
        amovel(pos1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "ready_final"): return False, "Task stopped by user"
        amovej(JReady, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        return True, "Tong task completed"

    except Exception as e:
        if ctx.check_stop():
            return False, "Task stopped by user"
        return False, f"Tong task error: {e}"
