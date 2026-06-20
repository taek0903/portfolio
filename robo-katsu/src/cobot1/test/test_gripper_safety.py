"""
Gripper Safety Tests

Issue 3 테스트: SAFE_STOP/EMERGENCY 상태에서 그리퍼 동작 문제

문제점:
- _gripper() 함수에 stop/fault 체크가 없음
- SAFE_STOP 발생 시에도 task 스레드가 _gripper() 중이면 digital output 제어됨
- 서보가 꺼져 있어도 공압식 그리퍼는 동작 가능

위험:
- 안전 정지 상태에서 예상치 못한 그리퍼 동작
- 작업자 부상 위험
"""

import threading
import time
import unittest
from unittest.mock import MagicMock, patch


class SafetyState:
    """안전 상태 시뮬레이션."""
    SAFE = 0
    SAFE_STOP = 1
    EMERGENCY = 2


class MockContext:
    """Task context mock."""

    def __init__(self):
        self._stop_requested = False
        self._fault_state = SafetyState.SAFE

    def check_stop(self):
        return self._stop_requested

    def check_fault(self):
        return self._fault_state != SafetyState.SAFE

    def set_stop(self):
        self._stop_requested = True

    def set_fault(self, state):
        self._fault_state = state


class TestGripperSafety(unittest.TestCase):
    """그리퍼 안전 동작 테스트."""

    def test_current_gripper_no_stop_check(self):
        """현재 _gripper() 구현은 stop 체크가 없음 (버그).

        현재 코드:
        def _gripper(action: str):
            from DSR_ROBOT2 import set_digital_output, wait
            for ch in (1, 2, 3, 4):
                set_digital_output(ch, 0)  # stop 체크 없음!
            wait(0.1)
            ch = _TONG_DO_MAP[action]
            set_digital_output(ch, 1)  # stop 체크 없음!
            wait(1.5)
        """
        digital_outputs = {}
        stop_requested = [False]

        def mock_set_digital_output(ch, value):
            # 현재 구현: stop 체크 없이 그냥 실행
            digital_outputs[ch] = value

        def mock_wait(seconds):
            time.sleep(seconds * 0.01)  # 테스트용 단축

        # SAFE_STOP 발생
        stop_requested[0] = True

        # 현재 _gripper 로직 (stop 체크 없음)
        TONG_DO_MAP = {"GRIP_BASIC": 1, "RELEASE": 2}
        action = "GRIP_BASIC"

        for ch in (1, 2, 3, 4):
            mock_set_digital_output(ch, 0)
        mock_wait(0.1)
        ch = TONG_DO_MAP[action]
        mock_set_digital_output(ch, 1)
        mock_wait(1.5)

        # 버그: SAFE_STOP 상태인데도 digital output이 실행됨
        self.assertEqual(digital_outputs.get(1), 1, "SAFE_STOP에서도 그리퍼 동작함 (버그)")

    def test_safe_gripper_with_stop_check(self):
        """수정된 _gripper(): stop 체크 포함."""
        digital_outputs = {}
        ctx = MockContext()
        executed_steps = []

        def safe_gripper(action: str, ctx: MockContext):
            """안전한 그리퍼 제어 - stop/fault 체크 포함."""
            TONG_DO_MAP = {"GRIP_BASIC": 1, "RELEASE": 2}

            # Step 1: 모든 채널 리셋
            if ctx.check_stop() or ctx.check_fault():
                return False

            for ch in (1, 2, 3, 4):
                digital_outputs[ch] = 0
            executed_steps.append("reset_channels")

            # Step 2: wait 중 stop 체크
            for _ in range(10):  # 0.1초를 10번으로 분할
                if ctx.check_stop() or ctx.check_fault():
                    return False
                time.sleep(0.001)

            # Step 3: 목표 채널 설정
            if ctx.check_stop() or ctx.check_fault():
                return False

            ch = TONG_DO_MAP[action]
            digital_outputs[ch] = 1
            executed_steps.append("set_target")

            # Step 4: wait 중 stop 체크
            for _ in range(150):  # 1.5초를 150번으로 분할
                if ctx.check_stop() or ctx.check_fault():
                    # 그리퍼를 안전한 상태로 (열기)
                    digital_outputs[TONG_DO_MAP["RELEASE"]] = 1
                    executed_steps.append("emergency_release")
                    return False
                time.sleep(0.001)

            return True

        # 테스트 1: 정상 동작
        ctx = MockContext()
        result = safe_gripper("GRIP_BASIC", ctx)
        self.assertTrue(result)
        self.assertIn("set_target", executed_steps)

        # 테스트 2: 중간에 SAFE_STOP 발생
        executed_steps.clear()
        digital_outputs.clear()
        ctx = MockContext()

        def trigger_fault():
            time.sleep(0.05)
            ctx.set_fault(SafetyState.SAFE_STOP)

        fault_thread = threading.Thread(target=trigger_fault)
        fault_thread.start()

        result = safe_gripper("GRIP_BASIC", ctx)
        fault_thread.join()

        self.assertFalse(result, "SAFE_STOP 시 그리퍼 동작 중단되어야 함")
        self.assertIn("emergency_release", executed_steps, "비상 시 그리퍼 열림 처리")

    def test_gripper_during_safe_stop_transition(self):
        """SAFE_STOP 진입 중 그리퍼 동작 시나리오.

        타임라인:
        T0: task 실행 중, 모션 완료
        T1: _gripper("GRIP_TIGHT") 시작
        T2: 외력 감지 → SAFE_STOP 진입
        T3: _gripper() 계속 실행 (stop 체크 없음!)
        T4: 그리퍼가 세게 닫힘 (위험!)
        """
        gripper_actions = []
        safe_stop_at = [None]

        def current_gripper_behavior(action):
            """현재 구현의 동작."""
            gripper_actions.append(f"start_{action}")
            time.sleep(0.01)  # 0.1s wait 시뮬레이션

            # SAFE_STOP이 여기서 발생해도 체크 안 함
            if safe_stop_at[0] and time.time() >= safe_stop_at[0]:
                pass  # 체크 안 함!

            gripper_actions.append(f"execute_{action}")
            time.sleep(0.015)  # 1.5s wait 시뮬레이션
            gripper_actions.append(f"complete_{action}")

        # SAFE_STOP 타이밍 설정
        safe_stop_at[0] = time.time() + 0.005

        current_gripper_behavior("GRIP_TIGHT")

        # 버그 증명: SAFE_STOP 후에도 그리퍼 동작 완료됨
        self.assertIn("complete_GRIP_TIGHT", gripper_actions)


class TestDigitalOutputSafety(unittest.TestCase):
    """Digital output 안전 제어 테스트."""

    def test_do_should_respect_robot_state(self):
        """Digital output은 로봇 상태를 고려해야 함."""
        robot_state = {"state": SafetyState.SAFE}
        do_calls = []

        def safe_set_digital_output(ch, value, robot_state_ref):
            """안전한 DO 설정."""
            if robot_state_ref["state"] != SafetyState.SAFE:
                do_calls.append(("blocked", ch, value))
                return False
            do_calls.append(("executed", ch, value))
            return True

        # 정상 상태에서 DO 설정
        result = safe_set_digital_output(1, 1, robot_state)
        self.assertTrue(result)

        # SAFE_STOP 상태에서 DO 설정 시도
        robot_state["state"] = SafetyState.SAFE_STOP
        result = safe_set_digital_output(2, 1, robot_state)
        self.assertFalse(result, "SAFE_STOP에서 DO 설정은 차단되어야 함")

        blocked_calls = [c for c in do_calls if c[0] == "blocked"]
        self.assertEqual(len(blocked_calls), 1)


class TestGripperEmergencyRelease(unittest.TestCase):
    """비상 시 그리퍼 열림 동작 테스트."""

    def test_gripper_should_release_on_emergency(self):
        """비상 정지 시 그리퍼는 열려야 함 (안전).

        시나리오:
        - 그리퍼가 물체를 잡고 있는 중
        - EMERGENCY_STOP 발생
        - 그리퍼가 자동으로 열려야 함 (물체를 놓음)

        이유:
        - 잡고 있던 물체가 떨어질 수 있지만,
        - 사람이 끼인 경우 열어야 안전함
        """
        gripper_state = {"holding": True}
        emergency_release_called = [False]

        def on_emergency(gripper_state_ref):
            """비상 시 그리퍼 동작."""
            gripper_state_ref["holding"] = False
            emergency_release_called[0] = True

        # EMERGENCY 발생 시
        on_emergency(gripper_state)

        self.assertTrue(emergency_release_called[0])
        self.assertFalse(gripper_state["holding"])


class TestGripperInterruptHandling(unittest.TestCase):
    """그리퍼 동작 인터럽트 처리 테스트."""

    def test_gripper_wait_should_be_interruptible(self):
        """그리퍼 wait(1.5)가 인터럽트 가능해야 함.

        현재 문제:
        - wait(1.5)는 blocking call
        - 중간에 stop 요청해도 1.5초 기다려야 함

        해결:
        - wait를 작은 단위로 분할하고 각 단위에서 stop 체크
        """
        stop_requested = [False]
        wait_completed = [False]
        wait_interrupted = [False]

        def interruptible_wait(duration_sec, stop_check_fn, interval=0.01):
            """인터럽트 가능한 wait."""
            elapsed = 0
            while elapsed < duration_sec:
                if stop_check_fn():
                    return False  # 인터럽트됨
                time.sleep(interval)
                elapsed += interval
            return True

        def run_wait():
            result = interruptible_wait(
                0.15,  # 테스트용 단축
                lambda: stop_requested[0],
                0.01
            )
            if result:
                wait_completed[0] = True
            else:
                wait_interrupted[0] = True

        # stop 요청 후 wait
        thread = threading.Thread(target=run_wait)
        thread.start()

        time.sleep(0.05)
        stop_requested[0] = True

        thread.join()

        self.assertTrue(wait_interrupted[0], "wait가 인터럽트되어야 함")
        self.assertFalse(wait_completed[0])


if __name__ == "__main__":
    unittest.main()
