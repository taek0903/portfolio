"""
UI Bridge Node

웹 UI와 Task Controller 사이의 브릿지 + Firebase Realtime Database sync.

역할:
- UI에서 오는 Topic 메시지 → Task Controller Service 호출
- TaskController TaskState Topic → UI용 JSON 상태 메시지 발행
- UI status payload 를 Firebase Realtime Database 에 동기화
  * 스냅샷 경로 (`robots/<ui_ns>/status`) 에 최신 상태 `set()`
  * 의미 있는 상태 전이 시 이벤트 경로 (`robots/<ui_ns>/events`) 에 `push()`

Firebase 설정은 env 파일 기반 (`FIREBASE_SYNC_ENV_FILE` 또는
`~/.config/cobot1/firebase/firebase_sync.env`). 미설정 시 sync 기능은 비활성화되고
UI 상태 publish 는 평소대로 진행.
"""

import json
import os
import re
import secrets
import threading
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from urllib import error as urlerror, parse as urlparse, request as urlrequest

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from cobot_interfaces.msg import RobotStatus, TaskState
from cobot_interfaces.srv import StartTask, StopTask
from dsr_msgs2.srv import MoveStop, ServoOff

try:
    import firebase_admin
    from firebase_admin import credentials, db
except ImportError:  # firebase-admin 미설치 시 REST fallback
    firebase_admin = None
    credentials = None
    db = None


# StopTask stop_type 상수
STOP_NORMAL = 0
STOP_IMMEDIATE = 1
STOP_EMERGENCY = 2
STOP_PAUSE = 3
STOP_RESUME = 4
# TaskState 상수
STATE_IDLE = 0
STATE_RUNNING = 1
STATE_STOPPING = 2
STATE_ERROR = 3
STATE_PAUSED = 4

# UI action → task_name 매핑.
# motion_executor.TASK_REGISTRY 에 등록된 이름과 반드시 일치해야 함.
# gripper_on / gripper_off / clean 은 현재 task 로는 없어 UI 에서 눌러도 실패한다.
# (후속 TASK_REGISTRY 확장 시점에 매핑 활성화 예정)
ACTION_TO_TASK = {
    "auto_serving": "auto_serving",
    "home": "home",
    "rice": "rice",
    "tong": "tong",
    "sauce": "sauce",
    "gripper_open": "gripper_open",
    "gripper_on": "gripper_on",
    "gripper_off": "gripper_off",
    "clean": "clean",
    # 복구 플로우 — UI 의 "홈으로"/"재시작" 버튼이 직접 호출
    "recovery_home": "recovery_home",
    "recovery_rice": "recovery_rice",
    "recovery_tong": "recovery_tong",
    "recovery_sauce": "recovery_sauce",
    "recovery_auto_serving": "recovery_auto_serving",
    # auto_serving 중단 후 해당 모듈부터 끝까지 연속 실행
    "resume_from_rice": "resume_from_rice",
    "resume_from_tong": "resume_from_tong",
    "resume_from_sauce": "resume_from_sauce",
}

# 화면 표시용 라벨. UI action key 와 task_name 양쪽을 모두 포함해야
# task_cli 로 직접 실행한 경우에도 active_action 라벨이 올바르게 표시된다.
ACTION_LABELS = {
    "auto_serving": "배식 시작",
    "rice": "밥 푸기",
    "tong": "샐러드·돈까스 담기",
    "sauce": "소스 뿌리기",
    "home": "홈으로 이동",
    "gripper_open": "그리퍼 해제",
    "gripper_on": "그리퍼 ON",
    "gripper_off": "그리퍼 OFF",
    "clean": "도구 청소",
    "recovery_home": "홈으로 이동",
    "recovery_rice": "밥부터 다시 푸기",
    "recovery_tong": "샐러드·돈까스부터 다시 담기",
    "recovery_sauce": "소스부터 다시 뿌리기",
    "recovery_auto_serving": "처음부터 다시 받기",
    "resume_from_rice": "마저 진행하기",
    "resume_from_tong": "마저 진행하기",
    "resume_from_sauce": "마저 진행하기",
}

# TASK_REGISTRY 와 수동 동기. motion_executor.TASK_REGISTRY 가 source of truth 지만
# ui_bridge 는 DSR_ROBOT2 의존성을 피하기 위해 import 하지 않고 이름·라벨·아이콘만
# 복제한다. 신규 task 추가 시 양쪽 모두 수정 필요.
_MODULE_META = {
    "gripper_open": {"label": "그리퍼 해제", "icon": "🖐"},
    "home":         {"label": "홈 복귀",     "icon": "🏠"},
    "rice":         {"label": "밥",          "icon": "🍚"},
    "tong":         {"label": "샐러드·돈까스", "icon": "🍱"},
    "sauce":        {"label": "소스",        "icon": "🍯"},
}


def _module(name: str) -> dict:
    meta = _MODULE_META.get(name, {"label": name, "icon": ""})
    return {"name": name, "label": meta["label"], "icon": meta["icon"]}


# task_name → 순차 실행 모듈 리스트 (UI step 진행 표시용).
# motion_executor.TASK_REGISTRY 와 구성 동일해야 progress/step 이 일치.
TASK_MODULES: dict[str, list[dict]] = {
    "gripper_open":          [_module("gripper_open")],
    "home":                  [_module("gripper_open"), _module("home")],
    "rice":                  [_module("gripper_open"), _module("rice")],
    "tong":                  [_module("gripper_open"), _module("tong")],
    "sauce":                 [_module("gripper_open"), _module("sauce")],
    "auto_serving":          [_module("gripper_open"), _module("rice"), _module("tong"), _module("sauce")],
    "recovery_home":         [_module("gripper_open"), _module("home")],
    "recovery_rice":         [_module("gripper_open"), _module("rice")],
    "recovery_tong":         [_module("gripper_open"), _module("tong")],
    "recovery_sauce":        [_module("gripper_open"), _module("sauce")],
    "recovery_auto_serving": [_module("gripper_open"), _module("rice"), _module("tong"), _module("sauce")],
    "resume_from_rice":      [_module("gripper_open"), _module("rice"), _module("tong"), _module("sauce")],
    "resume_from_tong":      [_module("gripper_open"), _module("tong"), _module("sauce")],
    "resume_from_sauce":     [_module("gripper_open"), _module("sauce")],
}

# today_errors 링버퍼 최대 크기 (너무 많이 쌓이면 payload 비대화 방지)
TODAY_ERRORS_MAX = 50

# TaskState → op_state 매핑
STATE_TO_OP = {
    STATE_IDLE: "IDLE",
    STATE_RUNNING: "WORKING",
    STATE_STOPPING: "WORKING",
    STATE_ERROR: "IDLE",
    STATE_PAUSED: "PAUSED",
}

# DSR2 GetRobotState enum → 사람이 읽을 수 있는 이름.
# dsr_msgs2/srv/GetRobotState.srv 정의 기준. 255 는 아직 한 번도 수신 못한 상태(UNKNOWN) 로 사용.
ROBOT_STATE_NAMES = {
    0: "INITIALIZING",
    1: "STANDBY",
    2: "MOVING",
    3: "SAFE_OFF",
    4: "TEACHING",
    5: "SAFE_STOP",
    6: "EMERGENCY_STOP",
    7: "HOMMING",
    8: "RECOVERY",
    9: "SAFE_STOP2",
    10: "SAFE_OFF2",
    15: "NOT_READY",
    255: "UNKNOWN",
}

# DSR robot_state 값 — 외력/안전 정지 UI·명령 차단에 사용 (RobotStatus.msg 상수와 동일)
DSR_SAFETY_ROBOT_STATES = frozenset({3, 5, 6, 9, 10})
# 1계열 안전 정지: SAFE_STOP(5), SAFE_OFF(3)
DSR_SAFE_STOP_1_SERIES = frozenset({3, 5})
# 2계열 안전 정지: SAFE_STOP2(9), SAFE_OFF2(10)
DSR_SAFE_STOP_2_SERIES = frozenset({9, 10})
# RECOVERY 모드 (8): 수동 복구 중 — 손으로 로봇 이동 가능한 상태
DSR_RECOVERY_STATE = 8
# EMERGENCY_STOP (6)
DSR_EMERGENCY_STOP = 6
# NOT_READY (15)
DSR_NOT_READY = 15


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SystemState: Task 상태 + Robot 상태를 조합한 통합 시스템 상태
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# UI에서 복잡한 상태 파생 로직 없이 이 값만 보고 렌더링 결정 가능
class SystemState:
    """통합 시스템 상태 상수."""

    # 정상 상태 (0-9)
    IDLE = 0              # 대기 (작업 없음, 로봇 정상)
    WORKING = 1           # 작업 수행 중
    PAUSED = 2            # 일시 정지

    # 복구 필요 상태 (10-19)
    SAFE_STOP = 10        # 안전 정지 (1계열: SAFE_STOP, SAFE_OFF)
    SAFE_STOP_2 = 11      # 보호 정지 (2계열: SAFE_STOP2, SAFE_OFF2)
    EMERGENCY = 12        # 비상 정지
    RECOVERY = 13         # 수동 복구 모드 (손으로 이동 가능)

    # 복구 완료 후 선택 대기 (20-29)
    RECOVERED = 20        # 복구 완료, 후속 동작 선택 대기

    # 에러/비정상 (30-39)
    ERROR = 30            # 일반 에러
    NOT_READY = 31        # 시스템 준비 안 됨
    DISCONNECTED = 32     # 로봇 연결 안 됨


SYSTEM_STATE_NAMES = {
    SystemState.IDLE: "IDLE",
    SystemState.WORKING: "WORKING",
    SystemState.PAUSED: "PAUSED",
    SystemState.SAFE_STOP: "SAFE_STOP",
    SystemState.SAFE_STOP_2: "SAFE_STOP_2",
    SystemState.EMERGENCY: "EMERGENCY",
    SystemState.RECOVERY: "RECOVERY",
    SystemState.RECOVERED: "RECOVERED",
    SystemState.ERROR: "ERROR",
    SystemState.NOT_READY: "NOT_READY",
    SystemState.DISCONNECTED: "DISCONNECTED",
}

SYSTEM_STATE_LABELS = {
    SystemState.IDLE: "대기",
    SystemState.WORKING: "작업 중",
    SystemState.PAUSED: "일시 정지",
    SystemState.SAFE_STOP: "안전 정지",
    SystemState.SAFE_STOP_2: "보호 정지",
    SystemState.EMERGENCY: "비상 정지",
    SystemState.RECOVERY: "수동 복구 모드",
    SystemState.RECOVERED: "복구 완료",
    SystemState.ERROR: "오류",
    SystemState.NOT_READY: "준비 안 됨",
    SystemState.DISCONNECTED: "연결 안 됨",
}

# 스냅샷 중복 판단용 필드 — 이 필드들이 같으면 heartbeat 주기 외에 재전송 생략
SYNC_SIGNATURE_FIELDS = (
    "system_status",
    "task_status",
    "robot_status",
    "message",
    "mode",
)

# KST timezone (+09:00)
KST = timezone(timedelta(hours=9))


def kst_now_iso():
    """현재 시간을 KST ISO8601 형식으로 반환."""
    return datetime.now(KST).isoformat()


def kst_today_midnight() -> datetime:
    """현재 KST 날짜의 00:00:00+09:00 datetime 반환."""
    now = datetime.now(KST)
    return now.replace(hour=0, minute=0, second=0, microsecond=0)


def _default_env_path() -> Path:
    """XDG 표준 위치의 기본 env 파일 경로."""
    xdg_config = os.getenv("XDG_CONFIG_HOME", "").strip()
    base = Path(xdg_config) if xdg_config else Path.home() / ".config"
    return base / "cobot1" / "firebase" / "firebase_sync.env"


_ENV_LINE_RE = re.compile(r"^\s*(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$")


def _parse_env_file(path: Path) -> dict:
    """.env 파일을 dict 로 파싱. shell 없이도 읽을 수 있게 단순 파서 사용."""
    result = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        m = _ENV_LINE_RE.match(line)
        if not m:
            continue
        key, value = m.group(1), m.group(2)
        # 양끝 따옴표 제거 (따옴표 안의 값은 리터럴로 취급)
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ('"', "'"):
            value = value[1:-1]
        result[key] = value
    return result


def _load_env_file(logger) -> Path | None:
    """env 파일을 찾아서 os.environ 에 반영. 발견된 파일 경로를 반환."""
    override = os.getenv("FIREBASE_SYNC_ENV_FILE", "").strip()
    candidates = []
    if override:
        candidates.append(Path(override).expanduser())
    candidates.append(_default_env_path())

    for candidate in candidates:
        if not candidate.is_file():
            continue
        try:
            values = _parse_env_file(candidate)
        except Exception as exc:  # noqa: BLE001
            logger.warning(f"Failed to parse env file {candidate}: {exc}")
            continue

        # FIREBASE_* 키만 환경에 주입 (다른 변수 오염 방지).
        # 이미 설정된 값은 덮어쓰지 않아 shell export 를 우선시한다.
        for key, value in values.items():
            if not key.startswith("FIREBASE_"):
                continue
            if os.environ.get(key):
                continue
            os.environ[key] = value
        return candidate
    return None


class FirebaseRealtimeSync:
    """
    Firebase Realtime Database 동기화 래퍼.

    - admin SDK 사용 가능 + service account JSON 있으면 admin 모드
    - 그 외에는 REST 모드 (HTTP PUT/POST)
    - 둘 다 불가하면 disabled
    - status 는 스냅샷 경로에 set(), event 는 events 경로에 push()
    - 동일 시그니처 반복 sync 는 heartbeat 주기 내에서 스킵
    - in-flight 락으로 동시 네트워크 호출 중복 제거
    """

    def __init__(self, logger, env_file_path: Path | None = None):
        self._logger = logger
        self._env_file_path = env_file_path
        # env 파일과 같은 폴더가 service account JSON auto-detect 기준이 된다.
        self._base_dir = env_file_path.parent if env_file_path else _default_env_path().parent
        self._database_url = os.getenv("FIREBASE_DATABASE_URL", "").strip().rstrip("/")
        self._status_path = os.getenv(
            "FIREBASE_STATUS_PATH", "robots/m0609/status"
        ).strip().strip("/")
        self._events_path = os.getenv(
            "FIREBASE_EVENTS_PATH", "robots/m0609/events"
        ).strip().strip("/")
        self._auth_token = os.getenv("FIREBASE_DATABASE_AUTH", "").strip()
        self._timeout_sec = self._read_float_env("FIREBASE_SYNC_TIMEOUT_SEC", 3.0)
        self._heartbeat_sec = self._read_float_env("FIREBASE_SYNC_HEARTBEAT_SEC", 5.0)
        self._service_account_file = self._resolve_service_account_file()

        self._mode = "disabled"  # "admin" | "rest" | "disabled"
        self._enabled = False
        self._status_endpoint_url = ""
        self._events_endpoint_url = ""
        self._admin_app = None
        self._admin_status_ref = None
        self._admin_events_ref = None
        self._lock = threading.Lock()
        self._sync_in_flight = False
        self._last_requested_signature = ""
        self._last_request_monotonic = 0.0
        self._last_result = "disabled"
        self._last_synced_at = ""
        self._last_error = ""
        self._configure_backend()

    def _read_float_env(self, name: str, default: float) -> float:
        value = os.getenv(name, "").strip()
        if not value:
            return default
        try:
            parsed = float(value)
            return parsed if parsed > 0 else default
        except ValueError:
            self._logger.warning(
                f"Invalid {name} value '{value}'. Falling back to {default}."
            )
            return default

    def _resolve_service_account_file(self) -> Path | None:
        configured = os.getenv("FIREBASE_SERVICE_ACCOUNT_FILE", "").strip()
        if configured:
            candidate = Path(configured).expanduser()
            if not candidate.is_absolute():
                candidate = (self._base_dir / candidate).resolve()
            if candidate.is_file():
                return candidate
            self._logger.warning(
                f"FIREBASE_SERVICE_ACCOUNT_FILE not found: {candidate}"
            )
            return None

        # env 파일과 같은 폴더에서 *-firebase-adminsdk-*.json 자동 탐지
        if not self._base_dir.is_dir():
            return None
        matches = sorted(self._base_dir.glob("*-firebase-adminsdk-*.json"))
        if not matches:
            matches = sorted(self._base_dir.glob("*firebase-adminsdk*.json"))
        if not matches:
            return None
        if len(matches) > 1:
            self._logger.warning(
                f"Multiple Firebase service account files in {self._base_dir}. "
                f"Using {matches[0].name}. "
                "Set FIREBASE_SERVICE_ACCOUNT_FILE to pick explicitly."
            )
        return matches[0]

    def _build_endpoint_url(self, path: str) -> str:
        encoded_path = "/".join(
            urlparse.quote(segment, safe="") for segment in path.split("/") if segment
        )
        base = f"{self._database_url}/{encoded_path}.json"
        if not self._auth_token:
            return base
        return f"{base}?auth={urlparse.quote(self._auth_token, safe='')}"

    def _configure_backend(self):
        if not self._database_url or not self._status_path:
            self._logger.info(
                "Firebase sync disabled (FIREBASE_DATABASE_URL not set)."
            )
            return

        if firebase_admin and self._service_account_file:
            try:
                app_name = f"cobot1_ui_bridge_{os.getpid()}"
                cred = credentials.Certificate(str(self._service_account_file))
                self._admin_app = firebase_admin.initialize_app(
                    cred,
                    {"databaseURL": self._database_url},
                    name=app_name,
                )
                self._admin_status_ref = db.reference(
                    self._status_path, app=self._admin_app
                )
                self._admin_events_ref = db.reference(
                    self._events_path, app=self._admin_app
                )
                self._mode = "admin"
                self._enabled = True
                self._last_result = "ready"
                self._logger.info(
                    "Firebase sync enabled (admin SDK): "
                    f"status={self._status_path}, events={self._events_path}, "
                    f"cred={self._service_account_file.name}"
                )
                return
            except Exception as exc:  # noqa: BLE001
                self._logger.warning(
                    f"Firebase admin SDK init failed, falling back to REST: {exc}"
                )

        if self._service_account_file and not firebase_admin:
            self._logger.warning(
                "Service account JSON detected but `firebase-admin` is not installed. "
                "Running in REST mode (no SDK). `pip install firebase-admin` recommended."
            )

        self._status_endpoint_url = self._build_endpoint_url(self._status_path)
        self._events_endpoint_url = self._build_endpoint_url(self._events_path)
        self._mode = "rest"
        self._enabled = True
        self._last_result = "ready"
        self._logger.info(
            f"Firebase sync enabled (REST): status={self._status_path}, "
            f"events={self._events_path}"
        )

    def _make_signature(self, payload: dict) -> str:
        return json.dumps(
            {f: payload.get(f) for f in SYNC_SIGNATURE_FIELDS},
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )

    def enqueue_snapshot(self, payload: dict):
        """스냅샷 경로에 set(). 동일 시그니처면 heartbeat 주기 내에서 스킵."""
        if not self._enabled:
            return

        signature = self._make_signature(payload)
        now = time.monotonic()

        with self._lock:
            if self._sync_in_flight:
                return
            if (
                signature == self._last_requested_signature
                and now - self._last_request_monotonic < self._heartbeat_sec
            ):
                return
            self._sync_in_flight = True
            self._last_requested_signature = signature
            self._last_request_monotonic = now
            self._last_result = "syncing"
            payload_to_send = dict(payload)

        threading.Thread(
            target=self._snapshot_worker,
            args=(payload_to_send,),
            daemon=True,
        ).start()

    def enqueue_event(self, event: dict):
        """events 경로에 push(). 중복 억제 없이 항상 기록 (호출측에서 판단)."""
        if not self._enabled:
            return
        payload = {
            **event,
            "firebase_sync_source": "cobot1.ui_bridge",
        }
        threading.Thread(
            target=self._event_worker,
            args=(payload,),
            daemon=True,
        ).start()

    def _snapshot_worker(self, payload: dict):
        sync_payload = {**payload, "firebase_sync_source": "cobot1.ui_bridge"}
        try:
            if self._mode == "admin":
                self._admin_status_ref.set(sync_payload)
            else:
                self._rest_request("PUT", self._status_endpoint_url, sync_payload)
        except Exception as exc:  # noqa: BLE001
            self._set_error_state(str(exc))
            return
        self._set_success_state()

    def _event_worker(self, payload: dict):
        try:
            if self._mode == "admin":
                self._admin_events_ref.push(payload)
            else:
                self._rest_request("POST", self._events_endpoint_url, payload)
        except Exception as exc:  # noqa: BLE001
            # 이벤트 실패는 스냅샷 상태에 영향 주지 않도록 로그만.
            self._logger.warning(f"Firebase event push failed: {exc}")

    def _rest_request(self, method: str, url: str, payload: dict):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        req = urlrequest.Request(
            url,
            data=body,
            headers={"Content-Type": "application/json; charset=utf-8"},
            method=method,
        )
        try:
            with urlrequest.urlopen(req, timeout=self._timeout_sec) as response:
                response.read()
        except (urlerror.HTTPError, urlerror.URLError, TimeoutError) as exc:
            raise RuntimeError(str(exc)) from exc

    def _set_success_state(self):
        synced_at = kst_now_iso()
        with self._lock:
            previous = self._last_result
            self._sync_in_flight = False
            self._last_result = "synced"
            self._last_synced_at = synced_at
            self._last_error = ""
        if previous == "error":
            self._logger.info("Firebase sync restored.")

    def _set_error_state(self, message: str):
        log_needed = False
        with self._lock:
            if message != self._last_error or self._last_result != "error":
                log_needed = True
            self._sync_in_flight = False
            self._last_result = "error"
            self._last_error = message
        if log_needed:
            self._logger.warning(f"Firebase sync failed: {message}")

    def fetch_today_events(
        self,
        since: datetime | None = None,
        type_filter: str | None = None,
        mode_filter: str | None = None,
        fetch_limit: int = 500,
    ) -> list[dict]:
        """events 경로에서 `since`(기본: 당일 KST 00:00) 이후 이벤트를 반환.

        append-only 구조라 정렬된 인덱스 없이도 `limitToLast=fetch_limit` 로
        최근 N 개를 받아 클라이언트에서 `created_at` 필터링한다. 실패 시 빈 리스트.

        Args:
            since: 이 시점 이후 이벤트만 포함 (inclusive). None 이면 오늘 KST 00:00.
            type_filter: "complete" / "error" / "safe_stop" 등. None 이면 모두.
            mode_filter: "virtual" / "real". None 이면 모든 모드. ui_bridge 저장
                이벤트에는 `mode` 필드가 포함되어 있어, 실행 모드별로 독립된
                카운트/에러 버퍼를 복원할 수 있다.
            fetch_limit: 서버에서 가져올 최근 이벤트 최대 개수.
        """
        if not self._enabled:
            return []

        boundary = since if since is not None else kst_today_midnight()
        boundary_iso = boundary.isoformat()

        try:
            if self._mode == "admin":
                raw = (
                    db.reference(self._events_path, app=self._admin_app)
                    .order_by_key()
                    .limit_to_last(fetch_limit)
                    .get()
                )
            else:
                raw = self._rest_fetch_events(fetch_limit)
        except Exception as exc:  # noqa: BLE001
            self._logger.warning(f"Firebase fetch_today_events failed: {exc}")
            return []

        if not raw:
            return []

        if isinstance(raw, dict):
            iterable = raw.values()
        elif isinstance(raw, list):
            iterable = (v for v in raw if v is not None)
        else:
            return []

        events: list[dict] = []
        for ev in iterable:
            if not isinstance(ev, dict):
                continue
            created_at = ev.get("created_at", "")
            if not created_at or created_at < boundary_iso:
                continue
            if type_filter and ev.get("type") != type_filter:
                continue
            if mode_filter and ev.get("mode") != mode_filter:
                continue
            events.append(ev)

        events.sort(key=lambda e: e.get("created_at", ""))
        return events

    def _rest_fetch_events(self, fetch_limit: int):
        encoded_path = "/".join(
            urlparse.quote(segment, safe="")
            for segment in self._events_path.split("/")
            if segment
        )
        params = [
            ("orderBy", '"$key"'),
            ("limitToLast", str(fetch_limit)),
        ]
        if self._auth_token:
            params.append(("auth", self._auth_token))
        query = urlparse.urlencode(params, quote_via=urlparse.quote)
        url = f"{self._database_url}/{encoded_path}.json?{query}"
        req = urlrequest.Request(url, method="GET")
        with urlrequest.urlopen(req, timeout=self._timeout_sec) as response:
            body = response.read().decode("utf-8")
        return json.loads(body) if body else None

    def build_status_fields(self) -> dict:
        with self._lock:
            return {
                "firebase_sync_enabled": self._enabled,
                "firebase_sync_mode": self._mode,
                "firebase_sync_path": self._status_path if self._enabled else "",
                "firebase_sync_events_path": (
                    self._events_path if self._enabled else ""
                ),
                "firebase_sync_status": self._last_result,
                "firebase_sync_last_synced_at": self._last_synced_at,
                "firebase_sync_error": self._last_error,
            }


class UiBridgeNode(Node):
    def __init__(self):
        super().__init__("ui_bridge")

        self._cb_group = ReentrantCallbackGroup()

        self.declare_parameter("robot_namespace", "dsr01")
        self.declare_parameter("ui_namespace", "m0609")
        self.declare_parameter("mode", "real")  # "virtual" or "real"

        robot_ns = self.get_parameter("robot_namespace").value
        ui_ns = self.get_parameter("ui_namespace").value
        self._mode = self.get_parameter("mode").value

        self.get_logger().info(f"Robot namespace: /{robot_ns}")
        self.get_logger().info(f"UI namespace: /{ui_ns}")
        self.get_logger().info(f"Mode: {self._mode}")

        # ── Firebase env 파일 로딩 (os.environ 에 주입) ────────────────
        env_path = _load_env_file(self.get_logger())
        if env_path:
            self.get_logger().info(f"Loaded firebase env: {env_path}")
        else:
            self.get_logger().info(
                "No firebase env file (FIREBASE_SYNC_ENV_FILE or "
                f"{_default_env_path()}); sync disabled unless env vars exported."
            )
        self._firebase_sync = FirebaseRealtimeSync(self.get_logger(), env_path)
        # ──────────────────────────────────────────────────────────

        # Internal state
        self._event_id = 0
        # 당일(KST) 에러 이벤트 링버퍼. _enqueue_event("error", …) 때 append.
        # 새로고침해도 Firebase 에서 복원되어 당일 내역 유지.
        self._today_errors: list[dict] = []
        # 마지막으로 확인한 KST 날짜 (자정 롤오버 감지용)
        self._today_kst_date = datetime.now(KST).date()
        self._is_emergency = False
        self._last_message = "UI Bridge가 시작되었습니다."
        self._level = "success"

        # Firebase 에서 당일 이벤트 pull (기동 직후 새로고침 대응).
        # sync 비활성화/오프라인이면 빈 리스트 반환 → 0 으로 초기화.
        self._bootstrap_from_firebase()

        # TaskState cache
        self._task_state = STATE_IDLE
        self._task_name = ""
        self._progress = 0.0
        self._task_message = ""
        self._current_step = 0
        self._total_steps = 0
        self._current_step_name = ""
        # 합성 task (auto_serving) 의 현재 서브모듈
        self._module_name = ""
        self._module_label = ""
        self._module_index = 0
        self._module_total = 0

        # RobotStatus cache (robot_status_publisher 로부터 10Hz 수신)
        self._robot_state_enum = 255  # UNKNOWN until first valid sample
        self._robot_state_enum_valid = False
        self._posj = [0.0] * 6
        self._posj_valid = False
        self._tool_force = [0.0] * 6
        self._tool_force_valid = False
        self._external_joint_torque = [0.0] * 6
        self._external_joint_torque_valid = False
        # 로봇 안전 정지 진입 순간의 task 진행률 (task_controller 반영 전·race 시 UI 깜빡임 방지)
        self._progress_frozen_for_safety = None  # None | float 0..1

        # 복구 완료 상태 추적 (SAFE_* → STANDBY 전이 시 True, 작업 시작 시 False)
        self._recovery_completed = False
        self._prev_robot_state_enum = 255

        # Service clients (to TaskController)
        self._start_client = self.create_client(
            StartTask, f'/{robot_ns}/task/start',
            callback_group=self._cb_group,
        )
        self._stop_client = self.create_client(
            StopTask, f'/{robot_ns}/task/stop',
            callback_group=self._cb_group,
        )

        # DSR 시스템 서비스 직접 호출용 클라이언트.
        # EMERGENCY_STOP 은 task_controller → motion_executor 경로로 타면 복구
        # 요청(_handle_to_standby, 최대 ~20s 블로킹) 때문에 도달이 지연될 수 있어,
        # ui_bridge 에서 DSR 에 바로 move_stop + servo_off 를 쏘는 독립 경로를
        # 유지한다. 기존 task_controller 경유 호출도 함께 유지해 task_state 정리
        # (IDLE 전이 + "비상 정지" 메시지) 는 기존과 동일하게 수행.
        # 관련: docs/safe-off-recovery-bug.md "수정 2"
        self._dsr_move_stop_cli = self.create_client(
            MoveStop, f'/{robot_ns}/motion/move_stop',
            callback_group=self._cb_group,
        )
        self._dsr_servo_off_cli = self.create_client(
            ServoOff, f'/{robot_ns}/system/servo_off',
            callback_group=self._cb_group,
        )

        self._task_state_sub = self.create_subscription(
            TaskState, f'/{robot_ns}/task/state',
            self._task_state_callback, 10,
        )

        self._robot_status_sub = self.create_subscription(
            RobotStatus, f'/{robot_ns}/motion_executor/robot_status',
            self._robot_status_callback, 10,
        )

        # UI Topics - Subscribe
        self._serving_cmd_sub = self.create_subscription(
            String, f'/{ui_ns}/serving_cmd',
            self._handle_serving_cmd, 10,
        )
        self._pause_cmd_sub = self.create_subscription(
            String, f'/{ui_ns}/pause_cmd',
            self._handle_pause_cmd, 10,
        )
        self._safety_cmd_sub = self.create_subscription(
            String, f'/{ui_ns}/safety_cmd',
            self._handle_safety_cmd, 10,
        )

        # UI Topics - Publish
        self._status_pub = self.create_publisher(
            String, f'/{ui_ns}/ui/status', 10,
        )

        self._ui_ns = ui_ns

        self._status_timer = self.create_timer(0.2, self._publish_status)

        self.get_logger().info("UI Bridge node started")
        self.get_logger().info(f"  - Service: /{robot_ns}/task/start")
        self.get_logger().info(f"  - Service: /{robot_ns}/task/stop")
        self.get_logger().info(f"  - Subscribe: /{robot_ns}/task/state")
        self.get_logger().info(f"  - Subscribe: /{robot_ns}/motion_executor/robot_status")
        self.get_logger().info(f"  - Subscribe: /{ui_ns}/serving_cmd")
        self.get_logger().info(f"  - Publish: /{ui_ns}/ui/status")

    # ── TaskController 상태 수신 ────────────────────────────────────
    def _task_state_callback(self, msg: TaskState):
        prev_state = self._task_state
        prev_task = self._task_name

        self._task_state = msg.state
        self._task_name = msg.task_name
        self._progress = msg.progress
        self._task_message = msg.message
        self._current_step = msg.current_step
        self._total_steps = msg.total_steps
        self._current_step_name = msg.current_step_name
        self._module_name = getattr(msg, "current_module_name", "") or ""
        self._module_label = getattr(msg, "current_module_label", "") or ""
        self._module_index = int(getattr(msg, "module_index", 0) or 0)
        self._module_total = int(getattr(msg, "module_total", 0) or 0)

        if prev_state == msg.state and prev_task == msg.task_name:
            return

        self._event_id += 1

        if msg.state == STATE_IDLE and prev_state == STATE_RUNNING:
            task_label = ACTION_LABELS.get(prev_task, prev_task or "작업")
            self._last_message = f"{task_label} 완료되었습니다."
            self._level = "success"
            # complete 이벤트의 task_name 은 prev_task 기준 (현재 프레임의 task_name 은 "")
            self._firebase_sync.enqueue_event({
                "id": datetime.now().strftime("%Y%m%d") + "_" + secrets.token_hex(3),
                "type": "complete",
                "message": f"{task_label} 완료",
                "created_at": kst_now_iso(),
                "mode": self._mode,
                "task_name": prev_task,
                "step_name": self._current_step_name,
                "step_index": int(self._current_step),
                "total_steps": int(self._total_steps),
            })
        elif msg.state == STATE_RUNNING:
            task_label = ACTION_LABELS.get(msg.task_name, msg.task_name)
            self._last_message = f"{task_label} 작업을 시작합니다."
            self._level = "info"
            self._recovery_completed = False
        elif msg.state == STATE_PAUSED:
            self._last_message = "작업이 일시 정지되었습니다."
            self._level = "warning"
        elif msg.state == STATE_STOPPING:
            self._last_message = "작업이 정지 중입니다."
            self._level = "warning"
        elif msg.state == STATE_ERROR:
            self._last_message = f"오류 발생: {msg.message}"
            self._level = "error"
            self._enqueue_event(
                "error",
                f"Task 오류: {msg.message}",
            )

    # ── robot_status_publisher 수신 ────────────────────────────────
    def _robot_status_callback(self, msg: RobotStatus):
        """robot_status_publisher 가 10Hz 로 발행하는 RobotStatus → 캐시 갱신.

        각 필드에 대응하는 *_valid 플래그가 true 일 때만 반영해 stale 값을 내려보내지 않는다.
        안전 정지 진입 시 error event 를 Firebase 에 기록한다.
        """
        if msg.robot_state_valid:
            new_enum = int(msg.robot_state)
            prev_enum = self._prev_robot_state_enum

            prev_safety = prev_enum in DSR_SAFETY_ROBOT_STATES
            new_safety = new_enum in DSR_SAFETY_ROBOT_STATES
            prev_recovery = prev_enum == DSR_RECOVERY_STATE
            new_standby = new_enum == 1  # STANDBY

            # 안전 정지 진입 시 진행률 동결 + safe_stop event 기록.
            # 비상정지가 유발한 SAFE_* 도 Firebase event 만 중복 억제하고,
            # UI 자체는 정상적으로 Phase1 복구 패널을 표시한다 (사용자가 수동으로
            # 복구 방법을 선택). 비상정지 해제(RESET) 만 눌러서 자동 STANDBY
            # 전환을 시도하면 DSR 상태 race 로 복구가 중단되는 사례가 있어,
            # 명시적 "복구 시작하기" 버튼 경로로 되돌렸다.
            if new_safety and not prev_safety:
                self._progress_frozen_for_safety = self._progress
                self._recovery_completed = False
                state_name = ROBOT_STATE_NAMES.get(new_enum, "UNKNOWN")
                if not self._is_emergency:
                    self._enqueue_event(
                        "safe_stop",
                        f"로봇 보호 중단: {state_name}",
                    )
                    self.get_logger().warning(f"Robot entered safety state: {state_name}")
                else:
                    self.get_logger().info(
                        f"Robot entered {state_name} due to emergency stop (event suppressed)"
                    )
            # 안전 정지 해제 시 진행률 동결 해제
            elif not new_safety and prev_safety:
                self._progress_frozen_for_safety = None

            # 복구 완료 감지: SAFE_* → STANDBY 또는 RECOVERY → STANDBY
            if new_standby and (prev_safety or prev_recovery):
                self._recovery_completed = True
                self.get_logger().info("Recovery completed: robot returned to STANDBY")

            self._robot_state_enum = new_enum
            self._robot_state_enum_valid = True
            self._prev_robot_state_enum = new_enum
        if msg.posj_valid:
            self._posj = list(msg.posj)
            self._posj_valid = True
        if msg.tool_force_valid:
            self._tool_force = list(msg.tool_force)
            self._tool_force_valid = True
        if msg.external_joint_torque_valid:
            self._external_joint_torque = list(msg.external_joint_torque)
            self._external_joint_torque_valid = True

    # ── UI 명령 핸들러 ──────────────────────────────────────────────
    def _handle_serving_cmd(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"action": msg.data}

        action = payload.get("action", "")
        label = ACTION_LABELS.get(action, action or "알 수 없는 작업")

        self.get_logger().info(f"Received serving_cmd: {action}")

        if self._is_emergency:
            self._update_status(
                message="비상 정지 상태에서는 작업 요청이 차단됩니다.",
                level="warning",
            )
            return

        if (
            self._robot_state_enum_valid
            and self._robot_state_enum in DSR_SAFETY_ROBOT_STATES
        ):
            label = ROBOT_STATE_NAMES.get(
                self._robot_state_enum, "안전 정지"
            )
            self._update_status(
                message=f"로봇이 {label} 상태입니다. 복구(홈/재시작) 후 다시 시도하세요.",
                level="warning",
            )
            return

        if (
            self._robot_state_enum_valid
            and self._robot_state_enum == DSR_RECOVERY_STATE
        ):
            self._update_status(
                message="수동 복구 모드입니다. '이동 완료' 버튼을 눌러 복구를 완료하세요.",
                level="warning",
            )
            return

        if self._task_state not in (STATE_IDLE, STATE_ERROR):
            msg_text = (
                "로봇이 동작 중이어서 새 작업을 요청할 수 없습니다."
                if self._task_state == STATE_RUNNING
                else "로봇이 일시 정지 상태여서 새 작업을 요청할 수 없습니다."
            )
            self._update_status(message=msg_text, level="warning")
            return

        task_name = ACTION_TO_TASK.get(action, action)
        if not task_name:
            self._update_status(
                message=f"알 수 없는 작업: {action}",
                level="warning",
            )
            return

        if not self._start_client.service_is_ready():
            self._update_status(
                message="Task Controller 서비스에 연결할 수 없습니다.",
                level="error",
            )
            self.get_logger().error("Start service not ready")
            return

        request = StartTask.Request()
        request.task_name = task_name

        self.get_logger().info(f"Starting task: {task_name} (action: {action})")
        future = self._start_client.call_async(request)
        future.add_done_callback(
            lambda f: self._start_done_callback(f, label)
        )
        self._update_status(
            message=f"{label} 요청을 전송했습니다.",
            level="info",
        )

    def _start_done_callback(self, future, label):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f"Task started: {result.message}")
            else:
                self._update_status(
                    message=f"{label} 시작 실패: {result.message}",
                    level="error",
                )
        except Exception as e:  # noqa: BLE001
            self._update_status(
                message=f"서비스 호출 오류: {e}",
                level="error",
            )

    def _handle_pause_cmd(self, msg: String):
        command = msg.data.strip().upper()
        self.get_logger().info(f"Received pause_cmd: {command}")

        if self._is_emergency:
            self._update_status(
                message="비상 정지 상태입니다.",
                level="warning",
            )
            return

        if (
            self._robot_state_enum_valid
            and self._robot_state_enum in DSR_SAFETY_ROBOT_STATES
        ):
            self._update_status(
                message="안전 정지 상태에서는 일시정지/재개를 사용할 수 없습니다. 복구 버튼을 사용하세요.",
                level="warning",
            )
            return

        if (
            self._robot_state_enum_valid
            and self._robot_state_enum == DSR_RECOVERY_STATE
        ):
            self._update_status(
                message="수동 복구 모드에서는 일시정지/재개를 사용할 수 없습니다.",
                level="warning",
            )
            return

        if command == "PAUSE":
            if self._task_state != STATE_RUNNING:
                self._update_status(
                    message="로봇이 동작 중일 때만 일시 정지를 요청할 수 있습니다.",
                    level="warning",
                )
                return
            stop_type = STOP_PAUSE
        elif command == "RESUME":
            if self._task_state != STATE_PAUSED:
                self._update_status(
                    message="일시 정지 상태일 때만 재개를 요청할 수 있습니다.",
                    level="warning",
                )
                return
            stop_type = STOP_RESUME
        else:
            self._update_status(
                message=f"알 수 없는 명령: {command}",
                level="warning",
            )
            return

        if not self._stop_client.service_is_ready():
            self._update_status(
                message="Task Controller 서비스에 연결할 수 없습니다.",
                level="error",
            )
            return

        request = StopTask.Request()
        request.stop_type = stop_type
        self.get_logger().info(f"Sending {command} request (stop_type={stop_type})")
        future = self._stop_client.call_async(request)
        future.add_done_callback(
            lambda f: self._pause_done_callback(f, command)
        )
        self._update_status(
            message=f"{'일시 정지' if command == 'PAUSE' else '재개'} 요청을 전송했습니다.",
            level="info" if command == "RESUME" else "warning",
        )

    def _pause_done_callback(self, future, command):
        try:
            result = future.result()
            if not result.success:
                self._update_status(
                    message=f"{command} 실패: {result.message}",
                    level="error",
                )
        except Exception as e:  # noqa: BLE001
            self._update_status(
                message=f"서비스 호출 오류: {e}",
                level="error",
            )

    def _handle_safety_cmd(self, msg: String):
        command = msg.data.strip().upper()
        self.get_logger().info(f"Received safety_cmd: {command}")

        if command == "EMERGENCY_STOP":
            self._is_emergency = True
            self._update_status(
                message="비상 정지가 활성화되었습니다.",
                level="error",
            )

            # 1) DSR 서비스 직접 호출 — 복구 경로(_handle_to_standby)가
            #    motion_executor 의 execute_task/stop 콜백을 최대 ~20s 점유할 수
            #    있으므로, task_controller → motion_executor 경유로는 긴급정지가
            #    지연되어 "아무 버튼도 안 먹힘"으로 보이는 문제가 있었다.
            #    DSR 의 move_stop / servo_off 는 별도 서비스(엔드포인트)라 큐잉되지
            #    않고 곧바로 컨트롤러에 전달된다.
            #    관련: docs/safe-off-recovery-bug.md "버그 B" / "수정 2"
            if self._dsr_move_stop_cli.service_is_ready():
                mv_req = MoveStop.Request()
                mv_req.stop_mode = 1  # ST_QUICK
                self._dsr_move_stop_cli.call_async(mv_req)
                self.get_logger().warn("EMERGENCY: move_stop(QUICK) sent directly to DSR")
            else:
                self.get_logger().error(
                    "EMERGENCY_STOP: move_stop service not ready — motion may not halt"
                )

            if self._dsr_servo_off_cli.service_is_ready():
                so_req = ServoOff.Request()
                so_req.stop_type = 1  # QUICK (= motion_executor SERVO_OFF_STOP_TYPE_QUICK)
                self._dsr_servo_off_cli.call_async(so_req)
                self.get_logger().warn("EMERGENCY: servo_off(QUICK) sent directly to DSR")
            else:
                self.get_logger().error(
                    "EMERGENCY_STOP: servo_off service not ready — servo may stay ON"
                )

            # 2) task_controller 경유도 유지 — task_state 정리(IDLE 전이)와
            #    executor 내부 상태 리셋용. 이미 fire-and-forget 이라 블로킹 없음.
            #    task_state 와 무관하게 EMERGENCY 를 항상 forward 한다
            #    (STANDBY 에서 눌러도 servo_off 까지 도달해야 하므로).
            if self._stop_client.service_is_ready():
                req = StopTask.Request()
                req.stop_type = STOP_EMERGENCY
                self._stop_client.call_async(req)
            else:
                self.get_logger().warn(
                    "EMERGENCY_STOP: stop_client not ready — task state cleanup skipped"
                )

            self.get_logger().warn("EMERGENCY STOP activated")

        elif command == "RESET":
            # 비상정지 해제 — "_is_emergency" 플래그만 내린다.
            # 로봇은 여전히 SAFE_OFF/SAFE_STOP 상태라 UI 는 Phase1 복구 패널을
            # 띄우고, 사용자가 "자동 복구"/"수동 복구" 중 하나를 직접 선택해
            # STANDBY 로 전환한다. (자동으로 TO_STANDBY 를 쏘던 이전 구현은
            # DSR 의 set_robot_control 상태 전환 race 로 SAFE_OFF 에 갇히는
            # 사례가 있어 되돌림 — docs/ARCHITECTURE.md "비상정지 복구 절차"
            # 참조.)
            self._is_emergency = False
            self._update_status(
                message="비상 정지가 해제되었습니다. 복구 패널에서 복구 방법을 선택하세요.",
                level="success",
            )
            self.get_logger().info("Emergency stop cleared — awaiting manual recovery choice")

        else:
            self._update_status(
                message=f"알 수 없는 안전 명령: {command}",
                level="warning",
            )

    # ── 상태/이벤트 공통 ────────────────────────────────────────────
    def _update_status(self, message: str, level: str = "info"):
        """UI status payload 의 last_message/level 을 갱신."""
        self._event_id += 1
        self._last_message = message
        self._level = level

    def _enqueue_event(self, event_type: str, message: str):
        """의미 있는 이벤트를 Firebase events 경로에 append.

        Args:
            event_type: "complete", "error", "safe_stop" 등
            message: 사람이 읽을 수 있는 이벤트 설명
        """
        event_id = datetime.now().strftime("%Y%m%d") + "_" + secrets.token_hex(3)
        event_payload = {
            "id": event_id,
            "type": event_type,
            "message": message,
            "created_at": kst_now_iso(),
            "mode": self._mode,
            # 실패 지점 디버깅용 task/step 정보
            "task_name": self._task_name,
            "step_name": self._current_step_name,
            "step_index": self._current_step,
            "total_steps": self._total_steps,
        }
        self._firebase_sync.enqueue_event(event_payload)

        # 당일 에러 로컬 버퍼 append (Firebase 가 disabled 여도 세션 내에서는 유지)
        if event_type == "error":
            self._append_today_error(event_payload)

    def _append_today_error(self, event: dict):
        """today_errors 링버퍼에 에러 이벤트 append (오래된 것부터 drop)."""
        self._today_errors.append({
            "id": event.get("id", ""),
            "message": event.get("message", ""),
            "created_at": event.get("created_at", ""),
            "task_name": event.get("task_name", ""),
            "step_name": event.get("step_name", ""),
        })
        if len(self._today_errors) > TODAY_ERRORS_MAX:
            self._today_errors = self._today_errors[-TODAY_ERRORS_MAX:]

    def _bootstrap_from_firebase(self):
        """기동 시 Firebase 에서 당일(KST) 에러 이벤트를 pull 해 링버퍼 복원.

        - `type == "error"` → today_errors 링버퍼 (최근 TODAY_ERRORS_MAX 개)

        현재 실행 모드(virtual/real)와 동일한 이벤트만 필터링한다. 가상 에뮬레이터
        실행 중에는 virtual 이벤트만, 실기 실행 중에는 real 이벤트만 복원된다.
        """
        boundary = kst_today_midnight()
        events = self._firebase_sync.fetch_today_events(
            since=boundary,
            mode_filter=self._mode,
        )
        if not events:
            return

        errors: list[dict] = []
        for ev in events:
            if ev.get("type", "") != "error":
                continue
            errors.append({
                "id": ev.get("id", ""),
                "message": ev.get("message", ""),
                "created_at": ev.get("created_at", ""),
                "task_name": ev.get("task_name", ""),
                "step_name": ev.get("step_name", ""),
            })

        self._today_errors = errors[-TODAY_ERRORS_MAX:]
        self.get_logger().info(
            f"Bootstrapped from Firebase: today_errors={len(self._today_errors)}"
        )

    def _rollover_today_if_needed(self):
        """KST 날짜가 바뀌면 당일 에러 버퍼 리셋."""
        current_date = datetime.now(KST).date()
        if current_date != self._today_kst_date:
            self.get_logger().info(
                f"KST date rolled over: {self._today_kst_date} → {current_date}. "
                "Resetting today_errors."
            )
            self._today_kst_date = current_date
            self._today_errors = []

    def _compute_system_state(self) -> tuple[int, str, str]:
        """Task + Robot 상태를 조합해서 통합 시스템 상태 반환.

        Returns:
            (system_state, system_state_name, robot_status_label)
        """
        rs = self._robot_state_enum
        rs_valid = self._robot_state_enum_valid

        # 1. 비상 정지 (최우선)
        if self._is_emergency:
            return (SystemState.EMERGENCY, "EMERGENCY", "비상 정지")

        # 2. 로봇 연결 안 됨
        if not rs_valid:
            return (SystemState.DISCONNECTED, "DISCONNECTED", "연결 안 됨")

        # 3. DSR 하드웨어 상태 기반 (우선순위 높음)
        if rs == DSR_EMERGENCY_STOP:
            return (SystemState.EMERGENCY, "EMERGENCY", "비상 정지 (H/W)")

        if rs in DSR_SAFE_STOP_1_SERIES:
            label = ROBOT_STATE_NAMES.get(rs, "안전 정지")
            return (SystemState.SAFE_STOP, "SAFE_STOP", f"안전 정지 ({label})")

        if rs in DSR_SAFE_STOP_2_SERIES:
            label = ROBOT_STATE_NAMES.get(rs, "보호 정지")
            return (SystemState.SAFE_STOP_2, "SAFE_STOP_2", f"보호 정지 ({label})")

        if rs == DSR_RECOVERY_STATE:
            return (SystemState.RECOVERY, "RECOVERY", "수동 복구 모드")

        if rs == DSR_NOT_READY:
            return (SystemState.NOT_READY, "NOT_READY", "준비 안 됨")

        # 4. 복구 완료 상태 (SAFE_* → STANDBY 직후, 작업 시작 전)
        if self._recovery_completed and self._task_state == STATE_IDLE:
            return (SystemState.RECOVERED, "RECOVERED", "복구 완료")

        # 5. Task 상태 기반
        if self._task_state == STATE_RUNNING:
            return (SystemState.WORKING, "WORKING", "작업 중")

        if self._task_state == STATE_PAUSED:
            return (SystemState.PAUSED, "PAUSED", "일시 정지")

        if self._task_state == STATE_ERROR:
            return (SystemState.ERROR, "ERROR", "오류")

        # 6. 기본: IDLE
        return (SystemState.IDLE, "IDLE", "대기 상태")

    def _publish_status(self):
        # 자정(KST) 롤오버 시 카운터·에러 버퍼 리셋
        self._rollover_today_if_needed()

        # 통합 시스템 상태 계산
        system_state, system_state_name, system_state_label = self._compute_system_state()

        # task 상태 정보
        active_action_key = ""
        active_action_label = ""
        if self._task_state in (STATE_RUNNING, STATE_PAUSED):
            for action, task in ACTION_TO_TASK.items():
                if task == self._task_name:
                    active_action_key = action
                    break
            if not active_action_key:
                active_action_key = self._task_name
            active_action_label = ACTION_LABELS.get(active_action_key, self._task_name)

        progress_ratio = self._progress
        if self._progress_frozen_for_safety is not None:
            progress_ratio = self._progress_frozen_for_safety

        # 새로운 nested 구조
        system_status = {
            "state": system_state,
            "name": system_state_name,
            "label": system_state_label,
        }

        # 현재 실행 중/최근 task 의 모듈 리스트 (UI step 진행 표시용)
        task_modules = TASK_MODULES.get(self._task_name, [])

        task_status = {
            "state": self._task_state,
            "name": self._task_name,
            "label": active_action_label,
            "progress": int(progress_ratio * 100),
            # 서브태스크 정보 — auto_serving 중에는 rice/tong/sauce 로 전환됨
            "module_name": self._module_name,
            "module_label": self._module_label,
            "module_index": self._module_index,
            "module_total": self._module_total,
            # 현재 task 의 전체 모듈 리스트 (UI 가 진행/남음 렌더링에 사용)
            "modules": task_modules,
            # 세부 phase (샐러드/돈까스 등) 를 UI 에서 파싱할 수 있도록 step_name 도 노출
            "step_name": self._current_step_name,
            "current_step": int(self._current_step),
            "total_steps": int(self._total_steps),
        }

        robot_status = {
            "state": self._robot_state_enum if self._robot_state_enum_valid else None,
            "name": (
                ROBOT_STATE_NAMES.get(self._robot_state_enum, "UNKNOWN")
                if self._robot_state_enum_valid else None
            ),
            "posj": (
                [round(x, 2) for x in self._posj] if self._posj_valid else None
            ),
            "tool_force": (
                [round(x, 3) for x in self._tool_force] if self._tool_force_valid else None
            ),
            "external_joint_torque": (
                [round(x, 3) for x in self._external_joint_torque]
                if self._external_joint_torque_valid else None
            ),
        }

        message = {
            "text": self._last_message,
            "level": self._level,
        }

        payload = {
            "system_status": system_status,
            "task_status": task_status,
            "robot_status": robot_status,
            "message": message,
            "mode": self._mode,
            # 당일(KST) 에러 이벤트 — 새로고침해도 Firebase 에서 bootstrap 복원
            "today_errors": list(self._today_errors),
            "updated_at": kst_now_iso(),
        }

        # Firebase 스냅샷 sync (내부에서 중복 억제 + heartbeat 주기 관리).
        self._firebase_sync.enqueue_snapshot(payload)

        # UI 로 내보낼 payload 에는 sync 상태 필드도 추가 (home_ui.js 기대 스키마).
        payload_with_sync = {
            **payload,
            **self._firebase_sync.build_status_fields(),
        }
        out = String()
        out.data = json.dumps(payload_with_sync, ensure_ascii=False)
        self._status_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = UiBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down UI Bridge")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
