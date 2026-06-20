"""
Pause/Resume Race Condition Tests

Issue 1 테스트: 일시정지 버튼 간헐적 동작 문제

재현하려는 시나리오:
1. Task 실행 중 pause 요청
2. pause 요청이 처리되기 전에 두 번째 pause 요청
3. pause 성공했지만 모션이 이미 완료된 경우
4. 상태 전파 지연으로 인한 불일치
"""

import threading
import time
import unittest
from enum import IntEnum
from unittest.mock import MagicMock, patch, AsyncMock


class ExecutorState(IntEnum):
    IDLE = 0
    RUNNING = 1
    STOPPING = 2
    ERROR = 3
    PAUSED = 4


class SharedStateMock:
    """_SharedState mock for testing."""

    def __init__(self):
        self.lock = threading.Lock()
        self.state = ExecutorState.IDLE
        self.task_name = ""
        self.task_generation = 0
        self.stop_requested = False
        self.message = ""


class TestPauseRaceConditions(unittest.TestCase):
    """일시정지 관련 race condition 테스트."""

    def test_pause_when_not_running_should_fail(self):
        """RUNNING이 아닌 상태에서 pause 요청은 실패해야 함."""
        shared = SharedStateMock()
        shared.state = ExecutorState.IDLE

        # _handle_pause 로직 시뮬레이션
        with shared.lock:
            can_pause = shared.state == ExecutorState.RUNNING

        self.assertFalse(can_pause, "IDLE 상태에서 pause가 허용되면 안 됨")

    def test_pause_when_already_paused_should_fail(self):
        """이미 PAUSED 상태에서 pause 요청은 실패해야 함."""
        shared = SharedStateMock()
        shared.state = ExecutorState.PAUSED

        with shared.lock:
            can_pause = shared.state == ExecutorState.RUNNING

        self.assertFalse(can_pause, "PAUSED 상태에서 중복 pause가 허용되면 안 됨")

    def test_double_pause_race_condition(self):
        """두 번 연속 pause 요청 시 race condition 테스트.

        시나리오:
        1. state = RUNNING
        2. Thread A: pause 요청 시작, state 체크 통과
        3. Thread B: pause 요청 시작, state 체크 통과 (아직 RUNNING)
        4. Thread A: MovePause 서비스 호출
        5. Thread B: MovePause 서비스 호출 (중복!)
        """
        shared = SharedStateMock()
        shared.state = ExecutorState.RUNNING

        pause_count = [0]
        pause_lock = threading.Lock()

        def simulate_pause():
            # 현재 로직: lock 안에서 상태 체크만 하고 서비스 호출은 lock 밖에서
            with shared.lock:
                if shared.state != ExecutorState.RUNNING:
                    return False

            # 서비스 호출 시뮬레이션 (lock 밖)
            time.sleep(0.01)  # 서비스 호출 지연

            with pause_lock:
                pause_count[0] += 1

            return True

        threads = [threading.Thread(target=simulate_pause) for _ in range(2)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # 버그: 두 요청 모두 성공할 수 있음 (중복 MovePause 호출)
        self.assertEqual(
            pause_count[0], 2,
            "현재 구현은 중복 pause를 허용함 (버그 증명)"
        )

    def test_pause_callback_state_check_race(self):
        """MovePause 콜백에서 상태 체크 race condition.

        시나리오:
        1. pause 요청 → MovePause 서비스 호출
        2. 사용자가 stop 요청 → state = STOPPING
        3. MovePause 콜백 도착 → state != RUNNING이므로 PAUSED로 안 바뀜
        """
        shared = SharedStateMock()
        shared.state = ExecutorState.RUNNING

        # 1. pause 요청 시작
        with shared.lock:
            initial_check = shared.state == ExecutorState.RUNNING
        self.assertTrue(initial_check)

        # 2. pause 서비스 호출 중에 stop 요청이 오면
        shared.state = ExecutorState.STOPPING

        # 3. MovePause 콜백에서 상태 체크
        # 현재 로직: if self._shared.state == ExecutorState.RUNNING
        with shared.lock:
            if shared.state == ExecutorState.RUNNING:
                shared.state = ExecutorState.PAUSED

        # 버그: state가 PAUSED로 안 바뀜 (STOPPING 유지)
        self.assertEqual(
            shared.state, ExecutorState.STOPPING,
            "콜백에서 상태가 RUNNING이 아니면 PAUSED로 전환 안 됨"
        )


class TestPauseCallbackLogging(unittest.TestCase):
    """pause 콜백에서 실패 케이스 로깅 테스트."""

    def test_pause_failure_should_be_logged(self):
        """MovePause 서비스가 success=False 반환 시 로그가 있어야 함."""
        # 현재 코드의 문제점:
        # def _on_move_pause_done(self, future):
        #     res = future.result()
        #     if res and res.success:
        #         # 성공 처리
        #     # else: 아무것도 안 함! ← 버그
        logged_errors = []

        def mock_logger_error(msg):
            logged_errors.append(msg)

        def mock_logger_warn(msg):
            logged_errors.append(msg)

        # 실패 케이스 시뮬레이션
        class MockResult:
            success = False
            message = "Robot not in pausable state"

        result = MockResult()

        # 현재 로직 (버그)
        if result and result.success:
            pass  # 성공
        # else: 로깅 없음!

        self.assertEqual(len(logged_errors), 0, "현재 구현은 실패 시 로깅 안 함 (버그)")


class TestMotionCheckRace(unittest.TestCase):
    """wait_motion과 pause 사이의 race condition 테스트."""

    def test_motion_completes_before_pause_takes_effect(self):
        """모션이 pause 적용 전에 완료되면 task가 계속 진행됨.

        시나리오:
        1. amovel() 시작
        2. pause 요청
        3. MovePause 서비스 호출
        4. 그 사이에 모션이 완료됨
        5. check_motion() == 0 반환
        6. wait_motion() 반환
        7. task가 다음 step으로 진행!
        """
        motion_complete = threading.Event()
        pause_requested = threading.Event()
        task_continued = [False]

        def mock_check_motion():
            # 모션이 이미 완료됨
            return 0

        def mock_wait_motion(check_stop_fn, check_motion_fn):
            while True:
                if check_stop_fn():
                    return False
                if check_motion_fn() == 0:
                    return True
                time.sleep(0.01)

        # pause가 적용되기 전에 모션이 완료되면
        stop_requested = False

        def check_stop():
            return stop_requested

        # 모션이 이미 완료된 상태
        result = mock_wait_motion(check_stop, mock_check_motion)

        # 버그: pause 요청했지만 task가 계속 진행됨
        self.assertTrue(
            result,
            "모션이 이미 완료되면 wait_motion이 True 반환하고 task 계속 진행"
        )


class TestStatePropagtionDelay(unittest.TestCase):
    """상태 전파 지연 테스트."""

    def test_ui_state_lag_behind_executor(self):
        """UI 상태가 executor 상태보다 뒤쳐질 수 있음.

        전파 경로:
        motion_executor → task_controller → ui_bridge → UI
        각 단계에서 토픽 구독 + 처리 지연 발생
        """
        executor_state = ExecutorState.PAUSED
        controller_state = ExecutorState.RUNNING  # 아직 업데이트 안 됨
        ui_state = ExecutorState.RUNNING  # 아직 업데이트 안 됨

        # 사용자가 UI 상태를 보고 pause 버튼을 또 누름
        user_sees_running = ui_state == ExecutorState.RUNNING
        self.assertTrue(
            user_sees_running,
            "UI는 아직 RUNNING으로 표시 (전파 지연)"
        )

        # 실제로는 이미 PAUSED
        self.assertEqual(
            executor_state, ExecutorState.PAUSED,
            "실제 executor는 이미 PAUSED"
        )


class TestRecommendedFixes(unittest.TestCase):
    """권장 수정 사항 테스트."""

    def test_pause_with_in_flight_guard(self):
        """in-flight 가드로 중복 pause 방지."""
        shared = SharedStateMock()
        shared.state = ExecutorState.RUNNING
        pause_in_flight = [False]
        pause_count = [0]

        def pause_with_guard():
            with shared.lock:
                if shared.state != ExecutorState.RUNNING:
                    return False
                if pause_in_flight[0]:
                    return False  # 이미 진행 중
                pause_in_flight[0] = True

            try:
                # 서비스 호출 시뮬레이션
                time.sleep(0.01)
                pause_count[0] += 1
                return True
            finally:
                with shared.lock:
                    pause_in_flight[0] = False

        threads = [threading.Thread(target=pause_with_guard) for _ in range(2)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # 수정 후: 하나만 성공
        self.assertEqual(
            pause_count[0], 1,
            "in-flight 가드로 중복 pause 방지됨"
        )


if __name__ == "__main__":
    unittest.main()
