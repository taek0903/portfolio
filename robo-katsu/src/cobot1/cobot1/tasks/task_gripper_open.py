"""
Gripper Open Task — 그리퍼 안전 해제 전용 얇은 모듈.

SAFE_STOP/SAFE_OFF 복구 후 홈/작업 재시작 시, 로봇이 물건(주걱/집게/소스통/돈까스/
샐러드)을 쥔 상태 그대로 JReady 로 이동하면 안전 문제가 발생할 수 있다. 이 모듈은
그 위험을 제거하기 위해 실제 모션 전에 먼저 그리퍼를 연다.

설계 선택 — `_common.gripper_force_release` 로 위임:

    ── 왜 task_rice._gripper 가 아니라 gripper_force_release 인가 ──
    - rice 의 "DO1~DO4 전부 0 리셋 → 0.1s wait → DO2=1" 패턴은 0.1 s 동안 공압이
      모두 빠지는 구간을 거친다. 평소엔 문제 없지만 SAFE_STOP 직후 공압 잔압이
      애매한 상태에서 0.1 s 만 기다리고 DO2 를 켜면 RELEASE 위치 (75 mm) 까지
      못 벌어지는 사례가 있었다 (유저 리포트: "복구 후 그리퍼가 안 열림").
    - `gripper_force_release` 는 tracker 와 무관하게 DO1=0/DO3=0 을 각각
      bit_delay 만큼 기다린 뒤 DO2=1 을 인가하고, 마지막 settle 을 2.0 s
      로 넉넉히 줘서 공압 실린더가 완전히 열리는 것을 보장한다.

`motion_executor.TASK_REGISTRY` 의 "home" / "recovery_*" 합성 첫 모듈로 들어간다.
단독 실행용 "gripper_open" 키도 제공.
"""

from ._common import gripper_force_release


TASK_NAME = "gripper_open"
TASK_DESCRIPTION = "그리퍼 안전 해제 (transient-safe RELEASE)"
TASK_LABEL = "그리퍼 해제"


STEPS = [
    ("release_gripper", "그리퍼 해제 (RELEASE)"),
]


def run(ctx):
    total = len(STEPS)

    if ctx.check_stop():
        return False, "Task stopped by user"
    ctx.update_progress(1, total, "release_gripper", "그리퍼 해제 중...")

    # bit_delay=0.1, settle=2.0 — rice 의 기존 0.1+1.5 = 1.6 s 대비 약간 연장.
    if not gripper_force_release(ctx, bit_delay=0.1, settle=2.0):
        return False, "Task stopped by user"

    return True, "Gripper released"
