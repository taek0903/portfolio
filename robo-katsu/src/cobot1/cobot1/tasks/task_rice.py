"""
Rice Task - 주걱(도구) 집기 + 밥 푸기(힘 제어) + 식판 배식 + 주걱 반납.

기반: references/task_combine/rice_1.py (pick_up_tool + perform_task)
      references/task_combine/rice_2.py (test_return_tool)

두 reference 를 하나의 연속 task 로 병합 (같은 도구·TCP·vel/acc 공유).
좌표·궤적(movesx 3 점)·force control 파라미터·그리퍼 DO 매핑은 원본과 1:1 동일.

변경점:
- movej/movel → amovej/amovel + ctx.wait_motion() (pause/resume 지원)
- movesx → amovesx + ctx.wait_motion()
- force control 루프: pause 감지 시 release_force + release_compliance_ctrl 로
  disarm → PAUSED 해제 대기 → resume 시 task_compliance_ctrl + set_desired_force
  재장전. stop 또는 timeout 시 finally 에서 반드시 disarm.
- set_ref_coord(1) (TOOL) 사용 후 rice 종료 시 set_ref_coord(0) (BASE) 로 원복.
- rice_2 첫 줄의 `movej(plate)` (테스트 준비용, rice_1 마지막 포즈와 동일) 제거.
"""

import time

from ._common import initialize_robot, safe_wait, sync_do_bits


TASK_NAME = "rice"
TASK_DESCRIPTION = "주걱 집기 → 밥 푸기(힘 제어) → 배식 → 주걱 반납"
TASK_LABEL = "밥"

ROBOT_TCP = "GripperDA_v1"
ROBOT_TOOL = "Tool Weight"
VELOCITY = 300
ACC = 80


# ───────────────────── Gripper (references/task_combine/rice_1.gripper 와 동일) ─────────────────────
# reference rice_1.py 는 "매 액션 전 DO1~DO4 를 0 으로 리셋 → wait(0.1) → 목표 채널 set"
# 구조를 사용한다. 동일 정책 유지.
def _gripper(action: str, ctx):
    """그리퍼 제어 (references/task_combine/rice_1.py 와 동일 DO 매핑).

    GRIP_BASIC: DO1=1, DO2=1
    RELEASE:    DO2=1

    중단 정책 (2026-04-22 변경 — "holding failsafe"):
        safe_wait 중 stop/fault 감지 시 **DO 를 RELEASE 로 강제 세팅하지 않는다**.
        현재 DO 상태를 그대로 둬야 산업용 cobot 관습(쥐고 있던 payload 낙하 방지)에
        맞고, 이후 복구 시점에 `task_gripper_open` (= recovery_* 의 첫 모듈) 이
        명시적으로 그리퍼를 열어준다.

        주의: rice 는 레퍼런스 정책상 액션 직전에 DO1~DO4=0 으로 리셋한다. 이 0.1s
        구간에서 stop 이 들어오면 DO 가 "all zero" 상태로 남지만 이는 rice 의 기존
        매 액션 간 전이 상태와 동일해 새로운 위험을 만들지 않는다.
    """
    from DSR_ROBOT2 import set_digital_output

    if ctx.check_stop():
        return False

    # reference rice_1.py 의 "0 리셋 → 0.1s wait → 목표 채널 set" 구조 유지.
    for ch in (1, 2, 3, 4):
        set_digital_output(ch, 0)
    sync_do_bits([0, 0, 0])  # tong/sauce 와 공유되는 DO tracker 동기화
    if not safe_wait(ctx, 0.1):
        return False

    if ctx.check_stop():
        return False

    if action == "GRIP_BASIC":
        set_digital_output(1, 1)
        set_digital_output(2, 1)
        sync_do_bits([1, 1, 0])
    elif action == "RELEASE":
        set_digital_output(2, 1)
        sync_do_bits([0, 1, 0])

    if not safe_wait(ctx, 1.5):
        return False

    return True


# ───────────────────── STEPS ─────────────────────
STEPS = [
    # Phase 1: 주걱 집기 (references/task_combine/rice_1.pick_up_tool)
    ("init",             "로봇 초기화"),
    ("ready_pickup",     "JReady (주걱 집기 전)"),
    ("release_pre",      "그리퍼 열기"),
    ("pickup_above",     "주걱 위 (pos1)"),
    ("pickup_down",      "주걱 접근 (pos2)"),
    ("grip_tool",        "주걱 파지 (GRIP_BASIC)"),
    ("pickup_lift",      "주걱 들어올리기 (pos1)"),

    # Phase 2: 밥 푸기 (references/task_combine/rice_1.perform_task)
    ("ready_task",       "작업 JReady"),
    ("descend_joint",    "배식 전 하강 자세"),
    ("force_contact",    "힘 제어: 바닥 감지"),
    ("scoop_arc",        "반원 궤적 퍼올리기 (amovesx)"),
    ("scoop_retract",    "TOOL -Z 100mm 리트랙트"),
    ("scoop_shift_y",    "Joint 1 -30° 이동"),
    ("to_plate_1",       "식판 자세 1"),
    ("plate_rotate",     "6축 -90° (배식)"),
    ("to_plate_2",       "식판 자세 2"),

    # Phase 3: 주걱 반납 (references/task_combine/rice_2.test_return_tool)
    ("return_ready",     "JReady 경유 (반납 준비)"),
    ("return_above",     "데스크 진입 (pos1)"),
    ("return_down",      "주걱 내려놓기 (pos2)"),
    ("release_tool",     "그리퍼 열기"),
    ("return_mid",       "회피 위치 (pos3)"),
    ("return_lift",      "상승 (pos1)"),
    ("return_final",     "최종 원점 (JFinal)"),
]

_STEP_INDEX = {name: i + 1 for i, (name, _) in enumerate(STEPS)}
_STEP_DESC = {name: desc for name, desc in STEPS}
_TOTAL = len(STEPS)


def _step(ctx, name: str) -> bool:
    """step 이름으로 진행률 업데이트. STEPS 정의와 strict 일치."""
    if ctx.check_stop():
        return False
    ctx.update_progress(_STEP_INDEX[name], _TOTAL, name, _STEP_DESC[name])
    return True


# ───────────────────── Force control (pause/stop 지원) ─────────────────────
def _wait_force_contact(
    ctx,
    *,
    stx,
    fd,
    direction,
    axis,
    fmin: float = 0,
    fmax: float = 12,
    ref_coord: int = 1,
    timeout_sec: float = 15.0,
) -> bool:
    """Z축 힘 접촉 감지 루프. pause/stop 완전 지원.

    compliance motion 은 DSR 의 MovePause 로 제어되지 않으므로, task 내부에서
    pause 를 감지해 힘 제어를 disarm (release_force + release_compliance_ctrl)
    하고, resume 시 재장전한다. 이렇게 해야 grip 된 주걱으로 계속 바닥을 누르는
    일이 없고 UI 의 pause 반응성이 일관된다.

    Args:
        stx/fd/direction: task_compliance_ctrl / set_desired_force 파라미터
        axis/fmin/fmax:   check_force_condition 파라미터
        ref_coord:        set_ref_coord 인자 (1=TOOL, 0=BASE 등). 종료 시 호출자가 원복.
        timeout_sec:      바닥 감지 타임아웃. pause/resume 사이의 대기 시간은 제외.

    Returns:
        True: 바닥 감지 성공.
        False: stop 요청 또는 timeout. (finally 블록에서 disarm 보장)
    """
    from DSR_ROBOT2 import (
        task_compliance_ctrl,
        set_desired_force,
        release_force,
        release_compliance_ctrl,
        check_force_condition,
        get_tool_force,
        set_ref_coord,
        DR_TOOL,
        DR_FC_MOD_REL,
    )

    def _arm():
        set_ref_coord(ref_coord)
        task_compliance_ctrl(stx=stx)
        time.sleep(0.5)
        set_desired_force(fd=fd, dir=direction, mod=DR_FC_MOD_REL)

    def _disarm():
        try:
            release_force()
        except Exception:
            pass
        try:
            release_compliance_ctrl()
        except Exception:
            pass

    _arm()
    armed = True
    deadline = time.monotonic() + timeout_sec
    try:
        while True:
            if ctx.check_stop():
                return False
            if time.monotonic() > deadline:
                return False

            # pause 감지 → disarm 후 PAUSED 해제 대기 → resume 시 재장전
            if ctx.is_paused():
                if armed:
                    _disarm()
                    armed = False
                while True:
                    if ctx.check_stop():
                        return False
                    if not ctx.is_paused():
                        break
                    time.sleep(0.05)
                _arm()
                armed = True
                # pause/resume 동안은 timeout 카운터 리셋 (사용자가 의도한 대기)
                deadline = time.monotonic() + timeout_sec
                continue

            ret = check_force_condition(axis, min=fmin, max=fmax)
            # 필요 시 force 로깅 (reference 의 fz 계산 형태 유지)
            _ = get_tool_force(DR_TOOL)
            if ret in (-1, 1):
                return True
            time.sleep(0.05)
    finally:
        if armed:
            _disarm()


# ───────────────────── run ─────────────────────
def run(ctx):
    from DSR_ROBOT2 import (
        posx,
        amovej, amovel, amovesx,
        get_current_posj, get_current_posx,
        set_ref_coord,
        DR_AXIS_Z, DR_MV_MOD_REL, DR_TOOL,
    )

    # ── Phase 1 좌표 (references/task_combine/rice_1.pick_up_tool 와 동일) ──
    JReady_home = [0, 0, 90, 0, 90, 0]
    p_tool_above = posx([288.70, 219.31, 311.70, 113.48, 179.96, 113.0])   # pos1
    p_tool_down  = posx([281.07, 218.15, 168.10,  40.36,  179.76,  40.20]) # pos2 (reference rice_1.py 최신판)

    # ── Phase 2 좌표 (references/task_combine/rice_1.perform_task 와 동일) ──
    JReady_task = [0, -10, 70, 0, 90, 0]
    J_descend   = [11.66, 23.74, 60.51, 4.64, 75.15, 8.15]

    # Phase 3 좌표 (references/task_combine/rice_2.test_return_tool 와 동일)
    JReady_ret = [0, -10, 70, 0, 90, 0]
    JFinal     = [0, 0, 90, 0, 90, 0]
    p_ret_1 = posx([304.70, 219.31, 311.70, 113.48,  179.96, 113.0])   # pos1
    p_ret_2 = posx([304.15, 219.77, 190.26,  22.35, -179.92,  21.90]) # pos2
    p_ret_3 = posx([290.11, 212.02, 171.43,  28.01, -179.88,  27.29]) # pos3 (회피)

    try:
        # ═════════════ Phase 1: 주걱 집기 ═════════════
        if not _step(ctx, "init"): return False, "Task stopped by user"
        initialize_robot(tcp=ROBOT_TCP, tool=ROBOT_TOOL)

        if not _step(ctx, "ready_pickup"): return False, "Task stopped by user"
        amovej(JReady_home, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "release_pre"): return False, "Task stopped by user"
        if not _gripper("RELEASE", ctx): return False, "Task stopped by user"

        if not _step(ctx, "pickup_above"): return False, "Task stopped by user"
        amovel(p_tool_above, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "pickup_down"): return False, "Task stopped by user"
        amovel(p_tool_down, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "grip_tool"): return False, "Task stopped by user"
        if not _gripper("GRIP_BASIC", ctx): return False, "Task stopped by user"

        if not _step(ctx, "pickup_lift"): return False, "Task stopped by user"
        amovel(p_tool_above, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        # ═════════════ Phase 2: 밥 푸기 ═════════════
        if not _step(ctx, "ready_task"): return False, "Task stopped by user"
        amovej(JReady_task, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "descend_joint"): return False, "Task stopped by user"
        amovej(J_descend, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"
        # reference 의 wait(0.5) 는 stop 가능한 대기로 대체
        if not safe_wait(ctx, 0.5): return False, "Task stopped by user"

        # 힘 제어: 바닥 감지 (pause/stop 지원)
        # reference/task_combine/rice_1.py 의 check_force_condition(min=0, max=10) 과 동치.
        if not _step(ctx, "force_contact"): return False, "Task stopped by user"
        contacted = _wait_force_contact(
            ctx,
            stx=[1000, 1000, 200, 200, 200, 200],
            fd=[0, 0, 15, 0, 0, 0],
            direction=[0, 0, 1, 0, 0, 0],
            axis=DR_AXIS_Z,
            fmin=0,
            fmax=12,
            ref_coord=1,
            timeout_sec=15.0,
        )
        if not contacted:
            if ctx.check_stop():
                return False, "Task stopped by user"
            return False, "Force contact timeout (바닥 감지 실패)"

        # 힘 제어 종료 후 좌표계를 BASE(0) 로 명시적 복원.
        # _wait_force_contact 내부에서 set_ref_coord(1) (TOOL) 을 사용했으므로,
        # 이후 amovel 등의 기본 좌표계 해석에 영향이 없도록 복원 필요.
        set_ref_coord(0)
        if not safe_wait(ctx, 1.0): return False, "Task stopped by user"

        # 반원 퍼올리기 (references/task_combine/rice_1.py 의 movesx 3 점 그대로)
        if not _step(ctx, "scoop_arc"): return False, "Task stopped by user"
        p1   = posx([-15, 0, 15, 0, -10, 0])
        p1_1 = posx([-10, 0, 10, 0, -10, 0])
        p1_2 = posx([-10, 0, 13, 0, -12, 0])
        amovesx([p1, p1_1, p1_2], vel=15, acc=15, mod=DR_MV_MOD_REL, ref=DR_TOOL)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        # 리트랙트 (TOOL -Z 100mm)
        if not _step(ctx, "scoop_retract"): return False, "Task stopped by user"
        amovel([0, 0, -100, 0, 0, 0], vel=15, acc=20, mod=DR_MV_MOD_REL, ref=DR_TOOL)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        # Joint Y(j2) -30° (reference 는 `cur[1] - 30`)
        if not _step(ctx, "scoop_shift_y"): return False, "Task stopped by user"
        cur = get_current_posj()
        if isinstance(cur, tuple):
            cur = cur[0]
        target = [cur[0], cur[1] - 30, cur[2], cur[3], cur[4], cur[5]]
        amovej(target, vel=15, acc=30)
        if not ctx.wait_motion(): return False, "Task stopped by user"
        if not safe_wait(ctx, 1.0): return False, "Task stopped by user"

        # 식판 자세 1
        if not _step(ctx, "to_plate_1"): return False, "Task stopped by user"
        plate_1 = [-13.19, 22.44, 128.57, -11.92, -59.73, 18.29]
        amovej(plate_1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        # 6 축 -90° (배식 동작)
        if not _step(ctx, "plate_rotate"): return False, "Task stopped by user"
        cur_j = get_current_posj()
        if isinstance(cur_j, tuple):
            cur_j = cur_j[0]
        next_j = [float(v) for v in cur_j]
        next_j[5] -= 90.0
        amovej(next_j, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"
        if not safe_wait(ctx, 1.0): return False, "Task stopped by user"

        # 식판 자세 2
        if not _step(ctx, "to_plate_2"): return False, "Task stopped by user"
        plate_2 = [-11.5, 16.93, 127.69, -11.28, -53.39, -71.13]
        amovej(plate_2, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        # ═════════════ Phase 3: 주걱 반납 ═════════════
        # 주의: reference/task_combine/rice_2.py 의 `movej(plate)` (테스트 준비) 는
        # Phase 2 마지막 plate_2 와 동일 포즈이므로 병합 시 중복 제거.
        if not _step(ctx, "return_ready"): return False, "Task stopped by user"
        amovej(JReady_ret, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "return_above"): return False, "Task stopped by user"
        amovel(p_ret_1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "return_down"): return False, "Task stopped by user"
        amovel(p_ret_2, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        # DEBUG: 그리퍼 열기 전 현재 위치 확인
        cur_pos = get_current_posx()
        if isinstance(cur_pos, tuple):
            cur_pos = cur_pos[0]
        print(f"[DEBUG] return_down 완료 후 현재 위치: X={cur_pos[0]:.1f}, Y={cur_pos[1]:.1f}, Z={cur_pos[2]:.1f}")
        print(f"[DEBUG] 목표 위치 p_ret_2: X=304.15, Y=219.77, Z=190.26")

        if not _step(ctx, "release_tool"): return False, "Task stopped by user"
        if not _gripper("RELEASE", ctx): return False, "Task stopped by user"

        if not _step(ctx, "return_mid"): return False, "Task stopped by user"
        amovel(p_ret_3, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "return_lift"): return False, "Task stopped by user"
        amovel(p_ret_1, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        if not _step(ctx, "return_final"): return False, "Task stopped by user"
        amovej(JFinal, vel=VELOCITY, acc=ACC)
        if not ctx.wait_motion(): return False, "Task stopped by user"

        return True, "Rice task completed"

    except Exception as e:
        if ctx.check_stop():
            return False, "Task stopped by user"
        return False, f"Rice task error: {e}"

    finally:
        # 조합 task 에서 다음 모듈(tong 등)이 TOOL 좌표계에 영향받지 않도록
        # BASE (0) 로 원복. 실패해도 무시.
        try:
            set_ref_coord(0)
        except Exception:
            pass
