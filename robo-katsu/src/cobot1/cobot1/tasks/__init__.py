"""
Task Modules for Cobot1

각 task 는 `run(ctx)` 함수를 노출하는 모듈이다.
`ctx` 는 motion_executor 가 주입하는 얇은 어댑터로 다음 메서드를 제공한다:

  ctx.check_stop() -> bool
  ctx.is_paused() -> bool          # force-control 등 MovePause 로 제어되지 않는 구간 전용
  ctx.wait_motion() -> bool        # amovej/amovel/amovesx/amove_periodic 완료 대기
  ctx.update_progress(step: int, total: int, name: str, message: str = "") -> None

task 등록은 motion_executor.TASK_REGISTRY 에서 수행한다.
"""

# task_gripper_open 이 task_rice 에 의존하므로 rice 를 먼저 import.
from . import task_rice, task_gripper_open, task_home, task_tong, task_sauce

__all__ = [
    "task_gripper_open",
    "task_home",
    "task_rice",
    "task_tong",
    "task_sauce",
]
