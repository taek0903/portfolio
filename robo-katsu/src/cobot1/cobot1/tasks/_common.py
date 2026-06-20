"""
Task 모듈 공통 헬퍼

- initialize_robot(tcp, tool): DSR2 tool/tcp/mode 설정
- safe_wait(ctx, duration): 인터럽트 가능한 대기

모션 완료 대기는 `ctx.wait_motion()` 을 사용한다. motion_executor는 노드를
control/dsr 두 개로 분리해 DSR_ROBOT2.check_motion()을 안전하게 호출한다
(docs/rclpy-executor-dsr2-troubleshooting.md 참조).
"""

import time


def initialize_robot(tcp: str, tool: str = "Tool Weight") -> None:
    """Tool/TCP 설정 후 AUTONOMOUS로 전환. reference/*.py의 initialize_robot과 동일.

    최적화: 이미 원하는 tcp/tool 이 설정되어 있다면 MANUAL↔AUTONOMOUS 전환 +
    ``time.sleep(2)`` 를 건너뛰어 auto_serving (rice→tong→sauce) 연속 실행 시
    누적 초기화 딜레이(약 6초)를 제거한다. 단독 실행 시(첫 진입)에는 기존 동작
    그대로 Tool/TCP 를 강제 설정한다.
    """
    from DSR_ROBOT2 import (
        set_tool, set_tcp, set_robot_mode,
        get_tcp, get_tool,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
    )

    try:
        current_tcp = get_tcp()
        current_tool = get_tool()
    except Exception:
        current_tcp = None
        current_tool = None

    if current_tcp == tcp and current_tool == tool:
        # 이미 설정 완료 상태 - 중복 초기화 skip (연속 task 실행 딜레이 최소화)
        return

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(tool)
    set_tcp(tcp)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)


def safe_wait(ctx, duration_sec: float, interval: float = 0.05) -> bool:
    """인터럽트 가능한 대기.

    Args:
        ctx: TaskContext (check_stop() 메서드 필요)
        duration_sec: 대기 시간 (초)
        interval: stop 체크 간격 (초)

    Returns:
        True: 대기 완료
        False: stop 요청으로 인터럽트됨
    """
    elapsed = 0.0
    while elapsed < duration_sec:
        if ctx.check_stop():
            return False
        time.sleep(min(interval, duration_sec - elapsed))
        elapsed += interval
    return True


def safe_digital_output(ctx, channel: int, value: int) -> bool:
    """안전한 digital output 설정.

    stop 상태 체크 후 set_digital_output 호출.

    Args:
        ctx: TaskContext
        channel: DO 채널 번호
        value: 0 또는 1

    Returns:
        True: 설정 완료
        False: stop 요청으로 중단됨
    """
    if ctx.check_stop():
        return False
    from DSR_ROBOT2 import set_digital_output
    set_digital_output(channel, value)
    return True


# ═════════════════════════════════════════════════════════════════════════════
# Transient-safe gripper DO transition
# ─────────────────────────────────────────────────────────────────────────────
# 공압 그리퍼의 DO1/DO2/DO3 조합 중 (1,1,0) 은 "0 mm 전체 압착" 상태이므로,
# RELEASE(0,1,0) → GRIP_BASIC(1,0,0) 같은 단순 전이를 순차 set_digital_output
# 으로 처리하면 중간에 (1,1,0) 을 0.x s 라도 밟아 집고 있던 물체가 순간적으로
# 강하게 압착된다 (유저가 관찰한 "gripper oscillation").
#
# 이를 피하기 위해 모든 tong/sauce 계열 그리퍼 전이는 다음 규칙을 따른다.
#   1. 현재 bit 가 1 → 0 으로 내려가는 채널을 먼저 DO1→DO2→DO3 순서로 0 으로.
#   2. 0 → 1 로 올라가는 채널을 DO3→DO2→DO1 역순으로 1 로.
#   3. 각 비트 write 사이에 bit_delay 대기 (공압 밸브 안정화).
#   4. 모든 bit 적용 후 settle 대기 (기계적 위치 도달).
# 이렇게 하면 (1,1,0) 같은 "함정 상태" 를 거치지 않는다.
#
# references/debug_gripper/tong_fixed.py 의 구현을 정식 task 에 이식.
# ═════════════════════════════════════════════════════════════════════════════

# 모듈 레벨 DO 상태 tracker. 여러 task 모듈 (tong/sauce/rice/gripper_open)
# 사이에서 공유된다. _gripper 호출 시점의 하드웨어 DO 상태를 기억해 transition
# 을 계획하는 용도.
_cur_do_bits = [0, 0, 0]  # [DO1, DO2, DO3]


def sync_do_bits(bits) -> None:
    """직접 set_digital_output 을 쓴 task 모듈 (e.g. rice) 이 tracker 를 맞출 때 호출.

    Args:
        bits: 3-element iterable of 0/1 — 호출 직후의 DO1/DO2/DO3 상태.
    """
    global _cur_do_bits
    if not (hasattr(bits, "__iter__") and not isinstance(bits, (str, bytes))):
        raise ValueError(f"sync_do_bits: iterable required, got {bits!r}")
    vals = [int(bool(b)) for b in bits]
    if len(vals) != 3:
        raise ValueError(f"sync_do_bits: 3 elements required, got {vals!r}")
    _cur_do_bits = vals


def gripper_transition(
    ctx,
    target_bits,
    *,
    bit_delay: float = 0.1,
    settle: float = 1.5,
) -> bool:
    """Transient-safe 한 그리퍼 DO 전이.

    현재 tracker 상태에서 target_bits 로 넘어갈 때 (1→0) 비트를 먼저 내리고
    (0→1) 비트를 나중에 올려 (1,1,0) 같은 함정 상태를 회피한다.

    Args:
        ctx: TaskContext (check_stop/safe_wait 용)
        target_bits: [DO1, DO2, DO3] 목표값 (각 0 또는 1)
        bit_delay: 개별 비트 write 사이 대기 (초). 공압 밸브 기본값 0.1.
        settle: 모든 비트 적용 후 기계적 안정화 대기 (초). 기본 1.5.

    Returns:
        True: 전이 완료. (tracker 도 target_bits 로 갱신됨)
        False: stop 요청으로 중단됨.
    """
    from DSR_ROBOT2 import set_digital_output
    global _cur_do_bits

    if not (hasattr(target_bits, "__iter__") and not isinstance(target_bits, (str, bytes))):
        raise ValueError(f"gripper_transition: iterable required, got {target_bits!r}")
    target = [int(bool(b)) for b in target_bits]
    if len(target) != 3:
        raise ValueError(f"gripper_transition: 3 elements required, got {target!r}")

    if ctx.check_stop():
        return False

    # Phase 1: 1→0 비트를 DO1→DO2→DO3 순서로 내린다.
    for i in (0, 1, 2):
        if _cur_do_bits[i] == 1 and target[i] == 0:
            set_digital_output(i + 1, 0)
            _cur_do_bits[i] = 0
            if bit_delay > 0.0 and not safe_wait(ctx, bit_delay):
                return False

    # Phase 2: 0→1 비트를 DO3→DO2→DO1 역순으로 올린다.
    for i in (2, 1, 0):
        if _cur_do_bits[i] == 0 and target[i] == 1:
            set_digital_output(i + 1, 1)
            _cur_do_bits[i] = 1
            if bit_delay > 0.0 and not safe_wait(ctx, bit_delay):
                return False

    # Phase 3: settle.
    if settle > 0.0:
        if not safe_wait(ctx, settle):
            return False

    return True


def gripper_force_release(ctx, *, bit_delay: float = 0.1, settle: float = 2.0) -> bool:
    """Tracker 상태와 무관하게 RELEASE (0,1,0) 를 강제 인가.

    SAFE_STOP/SAFE_OFF 복구 직후 `task_gripper_open` 에서 호출. 복구 과정에
    DSR 이 DO 를 초기화했을 가능성이 있으므로 tracker 를 믿지 않고 3 비트
    모두 명시적으로 쓴다. 순서는 여전히 transient-safe (1→0 먼저, 0→1 마지막).

    Args:
        ctx: TaskContext
        bit_delay: 비트 간 대기. 기본 0.1.
        settle: 마지막 안정화 대기. rice._gripper 의 기존 1.5 s 보다 약간 늘려
            (2.0 s) 공압 실린더가 75 mm 완전히 열리는 것을 보장.

    Returns:
        True: RELEASE 완료. False: stop 요청.
    """
    from DSR_ROBOT2 import set_digital_output
    global _cur_do_bits

    if ctx.check_stop():
        return False

    # Phase 1: 1→0 비트 강제 clear (DO1, DO3).
    set_digital_output(1, 0)
    _cur_do_bits[0] = 0
    if bit_delay > 0.0 and not safe_wait(ctx, bit_delay):
        return False

    set_digital_output(3, 0)
    _cur_do_bits[2] = 0
    if bit_delay > 0.0 and not safe_wait(ctx, bit_delay):
        return False

    # Phase 2: DO2=1 (RELEASE).
    set_digital_output(2, 1)
    _cur_do_bits[1] = 1
    if bit_delay > 0.0 and not safe_wait(ctx, bit_delay):
        return False

    if settle > 0.0:
        if not safe_wait(ctx, settle):
            return False

    return True
