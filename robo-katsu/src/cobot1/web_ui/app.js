const { useState, useEffect, useRef } = React;
const { createRosClient, htm } = window;
const html = htm.bind(React.createElement);

const DEFAULT_ROSBRIDGE_URL = 'ws://localhost:9090';
const ROSBRIDGE_STORAGE_KEY = 'robot_ui_rosbridge_url';
const UI_NAMESPACE = 'm0609';
// task_controller 는 launch 시 robot_namespace(기본 dsr01) 아래에 task/start·stop 을 연다.
// ui_bridge·토픽(/m0609/…)과 혼동하지 말 것.
const ROBOT_NAMESPACE_STORAGE_KEY = 'robot_ui_robot_namespace';
const STATUS_TOPIC = `/${UI_NAMESPACE}/ui/status`;
const SERVING_CMD_TOPIC = `/${UI_NAMESPACE}/serving_cmd`;
const PAUSE_CMD_TOPIC = `/${UI_NAMESPACE}/pause_cmd`;
const SAFETY_CMD_TOPIC = `/${UI_NAMESPACE}/safety_cmd`;
const START_TASK_SERVICE_TYPE = 'cobot_interfaces/srv/StartTask';
const STOP_TASK_SERVICE_TYPE = 'cobot_interfaces/srv/StopTask';
const STOP_TYPES = Object.freeze({
  NORMAL: 0,
  IMMEDIATE: 1,
  EMERGENCY: 2,
  PAUSE: 3,
  RESUME: 4,
  SAFE_STOP_RECOVER: 5,  // legacy, TO_STANDBY와 동일
  TO_STANDBY: 6,         // 자동 복구: SAFE_* → STANDBY
  TO_RECOVERY: 7,        // 수동 복구 진입: SAFE_*2 → RECOVERY
  RECOVERY_DONE: 8,      // 수동 복구 완료: RECOVERY → STANDBY
});

// 통합 시스템 상태 (ui_bridge.SystemState와 동기화)
const SYSTEM_STATE = Object.freeze({
  // 정상 상태 (0-9)
  IDLE: 0,
  WORKING: 1,
  PAUSED: 2,
  // 복구 필요 상태 (10-19)
  SAFE_STOP: 10,      // 1계열 안전 정지
  SAFE_STOP_2: 11,    // 2계열 보호 정지
  EMERGENCY: 12,      // 비상 정지
  RECOVERY: 13,       // 수동 복구 모드
  // 복구 완료 후 (20-29)
  RECOVERED: 20,      // 복구 완료, 후속 동작 대기
  // 에러/비정상 (30-39)
  ERROR: 30,
  NOT_READY: 31,
  DISCONNECTED: 32,
});

const SYSTEM_STATE_LABELS = {
  [SYSTEM_STATE.IDLE]: '대기',
  [SYSTEM_STATE.WORKING]: '작업 중',
  [SYSTEM_STATE.PAUSED]: '일시 정지',
  [SYSTEM_STATE.SAFE_STOP]: '안전 정지',
  [SYSTEM_STATE.SAFE_STOP_2]: '보호 정지',
  [SYSTEM_STATE.EMERGENCY]: '비상 정지',
  [SYSTEM_STATE.RECOVERY]: '수동 복구 모드',
  [SYSTEM_STATE.RECOVERED]: '복구 완료',
  [SYSTEM_STATE.ERROR]: '오류',
  [SYSTEM_STATE.NOT_READY]: '준비 안 됨',
  [SYSTEM_STATE.DISCONNECTED]: '연결 안 됨',
};

const OP_STATE_LABELS = {
  IDLE: '대기',
  WORKING: '동작 중',
  PAUSED: '일시 정지',
};
const ROBOT_STATE_LABELS = {
  INITIALIZING: '초기화 중',
  STANDBY: '대기 중',
  MOVING: '이동 중',
  SAFE_OFF: '서보 꺼짐',
  TEACHING: '티칭 모드',
  SAFE_STOP: '안전 정지',
  EMERGENCY_STOP: '비상 정지',
  HOMMING: '원점 복귀 중',
  RECOVERY: '복구 모드',
  SAFE_STOP2: '보호 정지 2',
  SAFE_OFF2: '서보 꺼짐 2',
  NOT_READY: '준비 안 됨',
  UNKNOWN: '상태 수신 대기',
};
// DSR 복구 대상 enum. SAFE_STOP(5) 진입 후 수 초 내에 서보가 떨어지면서
// SAFE_OFF(3) 로 전이하는 경우가 잦아, 사용자가 버튼을 누르는 타이밍에 이미
// SAFE_OFF 로 와 있는 시나리오가 생긴다. 네 상태 모두 동일한 복구 경로
// (set_safe_stop_reset_type + set_robot_mode AUTONOMOUS) 로 해제 가능하므로
// UI 트리거 조건에 함께 포함한다.
const SAFE_STOP_STATES = new Set([
  'SAFE_STOP',
  'SAFE_STOP2',
  'SAFE_OFF',
  'SAFE_OFF2',
]);
// 2계열 안전 정지: 수동 복구(RECOVERY 경유) 가능
const SAFE_STOP_2_SERIES = new Set(['SAFE_STOP2', 'SAFE_OFF2']);
// Progress 게이지 freeze 대상 system_state 집합.
// SAFE_STOP/EMERGENCY/RECOVERY 는 물론 RECOVERED(후속 동작 대기) 구간에서도
// 중단 직전 % 를 유지해 사용자가 어디서 멈췄는지 인지할 수 있게 한다.
// 다음 task 가 실제로 RUNNING 으로 전환되면 다시 갱신이 허용된다.
const PROGRESS_FREEZE_SYSTEM_STATES = new Set([
  SYSTEM_STATE.SAFE_STOP,
  SYSTEM_STATE.SAFE_STOP_2,
  SYSTEM_STATE.EMERGENCY,
  SYSTEM_STATE.RECOVERY,
  SYSTEM_STATE.RECOVERED,
]);
// RECOVERY 모드: 손으로 로봇 이동 가능한 상태
const RECOVERY_STATE = 'RECOVERY';
// 복구 버튼은 ui_bridge 를 거치지 않고 직접 서비스를 호출하므로,
// motion_executor.TASK_REGISTRY 에 등록된 실제 task 이름을 써야 한다.
const DEFAULT_RESTART_TASK = 'auto_serving';

// UI action → motion_executor task 이름 매핑.
// ui_bridge.ACTION_TO_TASK 와 동기화 필요.
const ACTION_TO_TASK = {
  auto_serving: 'auto_serving',
  home: 'home',
  rice: 'rice',
  tong: 'tong',
  sauce: 'sauce',
};

const resolveTaskName = (actionOrTask) =>
  ACTION_TO_TASK[actionOrTask] || actionOrTask;

// SAFE_STOP 진입 시점의 (root task, module) 을 캡처해 재시작 시 올바른 범위로
// 재개한다. 중단 맥락에 따라 다음 규칙을 적용:
//
//   root = auto_serving 인 경우 (연속 배식 중단)
//     interrupted module = rice  → resume_from_rice  (rice → tong → sauce)
//     interrupted module = tong  → resume_from_tong  (tong → sauce)
//     interrupted module = sauce → resume_from_sauce (sauce 만)
//
//   root 가 단일 task (rice/tong/sauce/home) 인 경우
//     → recovery_<root> (동일 단일 모듈 재개)
//
// 즉 "중단 모듈 + 그 이후 남은 모듈" 을 한 번에 이어서 실행.
const RECOVERY_TASK_FOR = {
  auto_serving: 'recovery_auto_serving',
  rice: 'recovery_rice',
  tong: 'recovery_tong',
  sauce: 'recovery_sauce',
  home: 'recovery_home',
};
// auto_serving 중 중단된 module 이름 → 해당 모듈부터 sauce 까지 연속 실행하는 task
const RESUME_FROM_FOR = {
  rice: 'resume_from_rice',
  tong: 'resume_from_tong',
  sauce: 'resume_from_sauce',
};
// auto_serving 이어서 재시작 시 UI 표시용 라벨.
// "마저 진행하기를 누르면 …" 톤으로 일관화. 라벨 조각은 MODULE_LABELS 와 동일한
// 한국어 표기를 유지한다 (🍚 밥 / 🍱 샐러드·돈까스 / 🍯 소스).
const RESUME_DESC_FOR = {
  resume_from_rice: '마저 진행하기를 누르면 🍚 밥부터 이어서 진행합니다 (밥 → 샐러드·돈까스 → 소스)',
  resume_from_tong: '마저 진행하기를 누르면 🍱 샐러드·돈까스부터 이어서 진행합니다 (샐러드·돈까스 → 소스)',
  resume_from_sauce: '마저 진행하기를 누르면 🍯 소스부터 이어서 진행합니다',
};
// resume_from_* 실행 중 우측 Task Steps 패널에 "이미 완료된 것으로 표시할" 모듈 리스트.
// payload 의 `taskModules` 는 motion_executor 가 실제 실행하는 모듈만 포함(예:
// resume_from_tong → [gripper_open, tong, sauce]) 하기 때문에, 사용자가 auto_serving
// 의 전체 흐름을 인지할 수 있도록 앞쪽에 "건너뛴" 모듈을 virtualDone 로 prepend 한다.
// gripper_open 은 payload 쪽에 이미 있으므로 여기 포함하지 않는다.
const SKIPPED_BEFORE = {
  resume_from_rice: [],
  resume_from_tong: ['rice'],
  resume_from_sauce: ['rice', 'tong'],
};
const DEFAULT_RECOVERY_TASK = 'recovery_auto_serving';

// 같은 auto_serving 흐름인지 판정.
// - `auto_serving`            : 정상 시작
// - `recovery_auto_serving`   : 외부 복구 task (auto_serving 재실행)
// - `resume_from_rice|tong|sauce` : 1회 이상 이미 resume 된 상태
// 이 세 가지 모두 "rice→tong→sauce 연속 배식" 흐름이므로 2차 이상 중단되어도
// 현재 module 로부터 sauce 까지 이어서 실행되도록 resume_from_* 을 선택한다.
const isAutoServingRoot = (root = '') =>
  root === 'auto_serving' ||
  root === 'recovery_auto_serving' ||
  root.startsWith('resume_from_');

// 중단 시점 컨텍스트(root/module) → 재시작할 task 이름.
// 빈 인자는 안전하게 DEFAULT 로 fallback.
const resolveResumeTask = (root = '', module = '') => {
  // gripper_open 은 실제 "작업" 이 아니라 안전 해제 단계이므로 모듈 이름으로 잡지 않는다.
  const effectiveModule = module && module !== 'gripper_open' ? module : '';

  // auto_serving 계열(처음 시작 / recovery_auto_serving / resume_from_*) 에서
  // 중단된 경우에는 항상 resume_from_<current_module> 로 이어간다.
  // 이 분기가 없으면 2차 복구 때 root 가 `resume_from_tong` 이라 아래
  // `RECOVERY_TASK_FOR[module]` 로 폴백돼 tong 만 재실행되고 sauce 가 빠진다.
  if (isAutoServingRoot(root) && RESUME_FROM_FOR[effectiveModule]) {
    return RESUME_FROM_FOR[effectiveModule];
  }
  if (RECOVERY_TASK_FOR[effectiveModule]) return RECOVERY_TASK_FOR[effectiveModule];
  if (RECOVERY_TASK_FOR[root]) return RECOVERY_TASK_FOR[root];
  return DEFAULT_RECOVERY_TASK;
};

// 서브태스크(모듈) → 아이콘 매핑. ui_bridge 가 module_name 을 그대로 payload 로 내려준다.
const MODULE_ICONS = {
  gripper_open: '🖐',
  home: '🏠',
  rice: '🍚',
  tong: '🍱',
  sauce: '🍯',
};
const MODULE_LABELS = {
  gripper_open: '그리퍼 해제',
  home: '홈 복귀',
  rice: '밥',
  tong: '샐러드·돈까스',
  sauce: '소스',
};

// tong 내부 세부 phase (step_name 접두어 기반)
// - salad_*  / grip_salad / release_salad / plate_*salad* → 샐러드
// - pork1_*/pork2_* / grip_pork* / release_pork* → 돈까스 (1/2 통합 표시)
// 그 외(집게 집기/반납/ready) 는 null → module_label 만 표시
const getTongPhase = (stepName = '') => {
  if (!stepName) return null;
  if (stepName.includes('salad')) return '샐러드';
  if (stepName.startsWith('pork1') || stepName.startsWith('pork2')
      || stepName.includes('pork')) return '돈까스';
  return null;
};

// 현재 진행 중인 서브태스크 display 계산.
const describeSubTask = (taskStatus = {}) => {
  const moduleName = taskStatus.module_name || '';
  const moduleLabel =
    taskStatus.module_label || MODULE_LABELS[moduleName] || '';
  const icon = MODULE_ICONS[moduleName] || '🥣';

  let phase = null;
  if (moduleName === 'tong') {
    phase = getTongPhase(taskStatus.step_name || '');
  }

  return { moduleName, moduleLabel, icon, phase };
};

const safeLocalStorage = {
  get(key) {
    try {
      return window.localStorage.getItem(key);
    } catch (err) {
      return null;
    }
  },
  set(key, value) {
    try {
      window.localStorage.setItem(key, value);
    } catch (err) {
      // Ignore storage failures in restricted browser contexts.
    }
  },
};

const resolveRobotNamespace = () => {
  const params = new URLSearchParams(window.location.search);
  const fromQuery = (params.get('robot_ns') || params.get('robot') || '').trim();
  if (fromQuery) {
    const cleaned = fromQuery.replace(/^\/+|\/+$/g, '');
    safeLocalStorage.set(ROBOT_NAMESPACE_STORAGE_KEY, cleaned);
    return cleaned;
  }
  return safeLocalStorage.get(ROBOT_NAMESPACE_STORAGE_KEY) || 'dsr01';
};

const ROBOT_NAMESPACE = resolveRobotNamespace();
const TASK_START_SERVICE = `/${ROBOT_NAMESPACE}/task/start`;
const TASK_STOP_SERVICE = `/${ROBOT_NAMESPACE}/task/stop`;

const formatClock = (dateLike) => {
  const date = dateLike instanceof Date ? dateLike : new Date(dateLike);

  if (Number.isNaN(date.getTime())) {
    return '--:--:--';
  }

  return date.toLocaleTimeString([], {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
  });
};

// 하단 "상세 정보" 패널용 — payload 의 system_status/task_status/robot_status 같은
// 단일 단계 객체를 key/value 한 줄씩 노출. 중첩 object/array 는 JSON 한 줄 요약으로.
const formatRawValue = (v) => {
  if (v === null || v === undefined) return '-';
  if (Array.isArray(v)) return `[${v.map(formatRawValue).join(', ')}]`;
  if (typeof v === 'object') return JSON.stringify(v);
  if (typeof v === 'number' && !Number.isInteger(v)) return v.toFixed(3);
  if (typeof v === 'boolean') return v ? 'true' : 'false';
  return String(v);
};

const resolveRosbridgeUrl = () => {
  const params = new URLSearchParams(window.location.search);
  const urlFromQuery = params.get('rosbridge');

  if (urlFromQuery) {
    safeLocalStorage.set(ROSBRIDGE_STORAGE_KEY, urlFromQuery);
    return urlFromQuery;
  }

  return safeLocalStorage.get(ROSBRIDGE_STORAGE_KEY) || DEFAULT_ROSBRIDGE_URL;
};

const parseStatusPayload = (message) => {
  if (!message || typeof message.data !== 'string') {
    return null;
  }

  try {
    return JSON.parse(message.data);
  } catch (err) {
    return null;
  }
};

const formatRobotStateName = (stateName) => ROBOT_STATE_LABELS[stateName] || stateName || '상태 수신 대기';

const sleep = (ms) => new Promise((resolve) => window.setTimeout(resolve, ms));

const CircularGauge = ({ progress, statusInfo, subTask, pulseIcon }) => {
  const size = 280;
  const strokeWidth = 20;
  const normalizedProgress = Math.max(0, Math.min(progress, 100));
  const radius = (size - strokeWidth) / 2;
  const circumference = radius * 2 * Math.PI;
  const offset = circumference - (normalizedProgress / 100) * circumference;

  // 작업 중일 때는 서브태스크 아이콘/라벨로 덮어써 현재 어떤 모듈(밥/샐러드·돈까스/소스)이
  // 진행되는지 한 눈에 보이도록 한다. isWorking 외에는 기본 statusInfo 를 사용.
  const showSubTask = Boolean(subTask && subTask.label);
  const icon = showSubTask ? subTask.icon : statusInfo.icon;
  const badge = showSubTask ? subTask.label : statusInfo.label;
  const phaseTag = showSubTask ? subTask.phase : '';
  // standby 등 "대기" 상태에서 아이콘만 은은하게 펄스
  const iconClass = pulseIcon && !showSubTask ? 'text-6xl mb-1 animate-pulse' : 'text-6xl mb-1';

  return html`
    <div className="relative flex items-center justify-center" style=${{ width: size, height: size }}>
      <svg width=${size} height=${size} className="transform -rotate-90">
        <circle
          cx=${size / 2}
          cy=${size / 2}
          r=${radius}
          stroke="#e2e8f0"
          strokeWidth=${strokeWidth}
          fill="transparent"
        />
        <circle
          cx=${size / 2}
          cy=${size / 2}
          r=${radius}
          stroke=${statusInfo.color}
          strokeWidth=${strokeWidth}
          fill="transparent"
          strokeDasharray=${circumference}
          strokeDashoffset=${offset}
          strokeLinecap="round"
          style=${{ transition: 'stroke-dashoffset 0.5s ease-out' }}
        />
      </svg>
      <div className="absolute flex flex-col items-center justify-center text-center">
        <span className=${iconClass}>${icon}</span>
        <div className="text-5xl font-black text-slate-800 leading-none">
          ${normalizedProgress}
          <span className="text-xl ml-1 text-slate-400">%</span>
        </div>
        <div
          className="mt-2 px-4 py-1 rounded-full text-white font-bold text-xs"
          style=${{ backgroundColor: statusInfo.color }}
        >
          ${badge}
        </div>
        ${phaseTag && html`
          <div className="mt-1 px-3 py-0.5 rounded-full bg-white/80 border border-slate-200 text-slate-600 font-black text-[11px] tracking-wide">
            ${phaseTag}
          </div>
        `}
      </div>
    </div>
  `;
};

const App = () => {
  const [bridgeUrl] = useState(resolveRosbridgeUrl);
  const [connected, setConnected] = useState(false);
  const [robotStatus, setRobotStatus] = useState('연결 확인 중...');
  const [opState, setOpState] = useState('IDLE');
  const [progress, setProgress] = useState(0);
  const [activeActionKey, setActiveActionKey] = useState('');
  const [activeAction, setActiveAction] = useState('');
  const [isEmergency, setIsEmergency] = useState(false);
  const [robotStateName, setRobotStateName] = useState('UNKNOWN');
  // 통합 시스템 상태 (ui_bridge에서 계산)
  const [systemState, setSystemState] = useState(SYSTEM_STATE.DISCONNECTED);
  const [systemStateName, setSystemStateName] = useState('DISCONNECTED');
  const [logs, setLogs] = useState([]);
  // 당일(KST) 에러 이벤트 — ui_bridge 가 Firebase 에서 bootstrap 해서 내려준다.
  // 새로고침해도 유지됨. payload.today_errors 로 매 tick 덮어쓴다.
  const [todayErrors, setTodayErrors] = useState([]);
  // 현재 task 의 모듈(스텝) 리스트 — 진행/완료/남음 렌더링용.
  const [taskModules, setTaskModules] = useState([]);
  const [moduleIndex, setModuleIndex] = useState(0);
  const [moduleTotal, setModuleTotal] = useState(0);
  const [lastCompletedMessage, setLastCompletedMessage] = useState('아직 완료된 작업이 없습니다.');
  const [lastCompletedAt, setLastCompletedAt] = useState('--:--:--');
  const [lastErrorMessage, setLastErrorMessage] = useState('보고된 에러가 없습니다.');
  const [lastErrorAt, setLastErrorAt] = useState('--:--:--');
  const [lastUpdatedAt, setLastUpdatedAt] = useState('--:--:--');
  // 2단계 복구 UI 상태
  // recoveryPhase: 'idle' | 'phase1' | 'manual' | 'phase2'
  //   - idle: 정상 상태 또는 SAFE_STOP 진입 직후 (phase1으로 자동 전이)
  //   - phase1: 복구 방법 선택 (자동/수동)
  //   - manual: RECOVERY 모드 — 손으로 이동 후 "이동 완료" 대기
  //   - phase2: STANDBY 도달 후 후속 동작 선택 (홈/재시작)
  const [recoveryPhase, setRecoveryPhase] = useState('idle');
  const [recoveryBusy, setRecoveryBusy] = useState('');
  const [recoveryError, setRecoveryError] = useState('');
  // 상단 안내 토스트를 사용자가 X 버튼으로 닫았을 때의 key. 같은 (systemState, recoveryPhase,
  // connected) 조합 동안에는 숨김 유지, 조합이 바뀌면 key 가 달라져 자동 재노출.
  const [dismissedToastKey, setDismissedToastKey] = useState('');
  // 하단 "상세 정보" 패널용 raw payload 스냅샷. onmessage 에서 매 tick 덮어쓴다.
  const [rawStatus, setRawStatus] = useState({
    system_status: {},
    task_status: {},
    robot_status: {},
  });
  const clientRef = useRef(null);
  const lastEventIdRef = useRef(null);
  const latestStatusRef = useRef(null);
  const connectedRef = useRef(false);
  // SAFE_STOP 진입 시점의 (root task, 중단 module) 을 {root, module} 객체로 캡처.
  // '재시작' 버튼에서 resolveResumeTask(root, module) 로 이어서 실행할 task 결정.
  //   예) root='auto_serving', module='tong' → resume_from_tong (tong → sauce)
  //   예) root='tong',         module='tong' → recovery_tong   (tong 만)
  // SAFE_STOP 을 벗어나면(WORKING 전이 시) 리셋.
  const interruptedTaskRef = useRef({ root: '', module: '' });
  // 현재 서브태스크 표시용 state
  const [subTaskLabel, setSubTaskLabel] = useState('');
  const [subTaskIcon, setSubTaskIcon] = useState('');
  const [subTaskPhase, setSubTaskPhase] = useState('');

  useEffect(() => {
    connectedRef.current = connected;
  }, [connected]);

  const rememberEvent = (message, type, eventTime) => {
    if (type === 'success' && message.includes('완료')) {
      setLastCompletedMessage(message);
      setLastCompletedAt(formatClock(eventTime));
      return;
    }

    if (type === 'error') {
      setLastErrorMessage(message);
      setLastErrorAt(formatClock(eventTime));
    }
  };

  const addLog = (message, type = 'info', eventTime = new Date()) => {
    const timestamp = formatClock(eventTime);
    setLogs((prev) => [{ timestamp, message, type }, ...prev].slice(0, 8));
    rememberEvent(message, type, eventTime);
  };

  const callRosService = async (serviceName, serviceType, requestPayload) => {
    if (!clientRef.current) {
      throw new Error('Controller 연결이 아직 준비되지 않았습니다.');
    }

    if (!connectedRef.current) {
      throw new Error('Controller node와 연결되지 않았습니다.');
    }

    return clientRef.current.callService(serviceName, serviceType, requestPayload);
  };

  const requestStopTask = async (stopType) =>
    callRosService(TASK_STOP_SERVICE, STOP_TASK_SERVICE_TYPE, { stop_type: stopType });

  const requestStartTask = async (taskName) =>
    callRosService(TASK_START_SERVICE, START_TASK_SERVICE_TYPE, {
      task_name: taskName,
      task_id: '',
    });

  const ensureServiceSuccess = (response, fallbackMessage) => {
    if (response && response.success) {
      return response;
    }

    throw new Error(response?.message || fallbackMessage);
  };

  // SAFE_STOP 해제 요청(stop_type=5) 이후 로봇이 SAFE_STOP 에서 실제로 빠져나올
  // 때까지 폴링. motion_executor 가 set_robot_mode(AUTONOMOUS) 를 성공적으로
  // 호출한 뒤에도 DSR 컨트롤러가 상태를 STANDBY 로 전환하는 데 수백 ms 지연이
  // 있을 수 있어, ui_bridge 가 내려주는 robot.state_name 이 SAFE_STOP 을 벗어날
  // 때까지 기다린 뒤 후속 StartTask 를 보낸다.
  const waitForRobotStateLeavingSafeStop = (timeoutMs = 5000) =>
    new Promise((resolve, reject) => {
      const deadline = Date.now() + timeoutMs;

      const tick = () => {
        const stateName = latestStatusRef.current?.robot?.state_name || '';
        if (stateName && !SAFE_STOP_STATES.has(stateName)) {
          resolve(stateName);
          return;
        }

        if (Date.now() >= deadline) {
          reject(new Error('SAFE_STOP 상태가 해제되지 않았습니다.'));
          return;
        }

        window.setTimeout(tick, 150);
      };

      tick();
    });

  useEffect(() => {
    let isMounted = true;
    let unsubscribeStatus = null;
    const client = createRosClient(bridgeUrl);
    clientRef.current = client;

    client
      .connect({
        onConnection: () => {
          if (!isMounted) return;

          connectedRef.current = true;
          setConnected(true);
          setRobotStatus('브리지 연결 완료');
          setLastUpdatedAt(formatClock(new Date()));
          addLog(`ROS bridge 연결 성공: ${bridgeUrl}`, 'success');
        },
        onError: () => {
          if (!isMounted) return;

          connectedRef.current = false;
          setConnected(false);
          setRobotStatus('브리지 연결 오류');
          addLog('Controller node 연결에 실패했습니다.', 'error');
        },
        onClose: () => {
          if (!isMounted) return;

          connectedRef.current = false;
          setConnected(false);
          setRobotStatus('브리지 연결 종료');
        },
      })
      .then(() => {
        if (!isMounted) {
          client.disconnect();
          return;
        }

        unsubscribeStatus = client.subscribe(STATUS_TOPIC, 'std_msgs/String', (message) => {
          if (!isMounted) return;

          const payload = parseStatusPayload(message);
          if (!payload) return;

          const previousPayload = latestStatusRef.current;
          latestStatusRef.current = payload;

          const eventTime = payload.updated_at ? new Date(payload.updated_at) : new Date();
          setLastUpdatedAt(formatClock(eventTime));

          // 새로운 nested 구조 파싱
          const systemStatus = payload.system_status || {};
          const taskStatus = payload.task_status || {};
          const robotStatus = payload.robot_status || {};
          const messageObj = payload.message || {};

          // robot_status (라벨 표시용)
          if (robotStatus.label || robotStatus.name) {
            setRobotStatus(robotStatus.label || robotStatus.name);
          }

          // op_state (system_status.name으로 대체)
          if (systemStatus.name) {
            setOpState(systemStatus.name);
          }

          // progress 갱신 — 다음 상태에서는 이전 값을 유지(freeze):
          //   1) robot_state 기준 SAFE_STOP_STATES (SAFE_STOP/SAFE_OFF 계열)
          //   2) system_state 기준 PROGRESS_FREEZE_SYSTEM_STATES (EMERGENCY/RECOVERY/RECOVERED 포함)
          // RECOVERED 구간까지 freeze 를 연장해, 복구 완료 후 "마저 진행하기/홈으로"
          // 선택 화면에서도 중단 당시 진행률이 게이지에 그대로 보이게 한다.
          const rawCurState = robotStatus.name || '';
          const freezeByRobotState = SAFE_STOP_STATES.has(rawCurState);
          const freezeBySystemState = typeof systemStatus.state === 'number'
            && PROGRESS_FREEZE_SYSTEM_STATES.has(systemStatus.state);
          if (typeof taskStatus.progress === 'number'
              && !freezeByRobotState
              && !freezeBySystemState) {
            setProgress(taskStatus.progress);
          }

          // is_emergency (system_status.state 기반 계산)
          const isEmergencyState = systemStatus.state === SYSTEM_STATE.EMERGENCY;
          setIsEmergency(isEmergencyState);

          // 현재 task 의 모듈 리스트 (진행 단계 표시)
          if (Array.isArray(taskStatus.modules)) {
            setTaskModules(taskStatus.modules);
          }
          if (typeof taskStatus.module_index === 'number') {
            setModuleIndex(taskStatus.module_index);
          }
          if (typeof taskStatus.module_total === 'number') {
            setModuleTotal(taskStatus.module_total);
          }

          // 당일 에러 로그 (Firebase 에서 bootstrap 된 것 + 세션 내 신규)
          if (Array.isArray(payload.today_errors)) {
            setTodayErrors(payload.today_errors);
          }

          // 하단 "상세 정보" 패널용 raw 스냅샷 (system/task/robot status 원본).
          setRawStatus({
            system_status: systemStatus || {},
            task_status: taskStatus || {},
            robot_status: robotStatus || {},
          });

          // active_action_key / active_action
          if (taskStatus.name !== undefined) {
            setActiveActionKey(taskStatus.name || '');
          }
          if (taskStatus.label !== undefined) {
            setActiveAction(taskStatus.label || '');
          }

          // 서브태스크 (module_name/label + tong 내 phase)
          const sub = describeSubTask(taskStatus);
          setSubTaskLabel(sub.moduleLabel);
          setSubTaskIcon(sub.icon);
          setSubTaskPhase(sub.phase || '');

          // robotStateName
          if (robotStatus.name != null) {
            setRobotStateName(robotStatus.name !== '' ? robotStatus.name : 'UNKNOWN');
          }

          // 통합 시스템 상태 업데이트
          if (typeof systemStatus.state === 'number') {
            setSystemState(systemStatus.state);
          }
          if (systemStatus.name) {
            setSystemStateName(systemStatus.name);
          }

          // 메시지 로그 (message.text 변경 시 기록)
          const prevMessage = previousPayload?.message?.text;
          const curMessage = messageObj.text;
          if (curMessage && curMessage !== prevMessage) {
            addLog(curMessage, messageObj.level || 'info', eventTime);
          }

          // system_state 기반 복구 단계 관리 (단순화)
          const prevSystemState = previousPayload?.system_status?.state;
          const curSystemState = systemStatus.state;

          // SAFE_STOP/SAFE_STOP_2 진입 → phase1 (복구 방법 선택)
          if (
            (curSystemState === SYSTEM_STATE.SAFE_STOP || curSystemState === SYSTEM_STATE.SAFE_STOP_2) &&
            prevSystemState !== SYSTEM_STATE.SAFE_STOP &&
            prevSystemState !== SYSTEM_STATE.SAFE_STOP_2
          ) {
            // 중단 시점의 root task 와 서브모듈을 모두 캡처. auto_serving 중
            // 이었다면 root='auto_serving', module='tong' 형태로 저장되어
            // '재시작' 시 tong 부터 sauce 까지 연속 실행하도록 resolveResumeTask 가
            // resume_from_tong 을 선택한다.
            const capturedRoot =
              previousPayload?.task_status?.name ||
              taskStatus.name ||
              '';
            const capturedModule =
              previousPayload?.task_status?.module_name ||
              taskStatus.module_name ||
              '';
            interruptedTaskRef.current = {
              root: capturedRoot,
              module: capturedModule,
            };
            setRecoveryPhase('phase1');
            setRecoveryError('');
          }
          // RECOVERY 모드 진입 → manual (손으로 이동)
          else if (curSystemState === SYSTEM_STATE.RECOVERY && prevSystemState !== SYSTEM_STATE.RECOVERY) {
            setRecoveryPhase('manual');
            setRecoveryError('');
          }
          // RECOVERED 상태 진입 → phase2 (후속 동작 선택)
          else if (curSystemState === SYSTEM_STATE.RECOVERED && prevSystemState !== SYSTEM_STATE.RECOVERED) {
            setRecoveryPhase('phase2');
            setRecoveryError('');
          }
          // WORKING 상태 진입 → idle (작업 시작됨)
          else if (curSystemState === SYSTEM_STATE.WORKING && prevSystemState !== SYSTEM_STATE.WORKING) {
            setRecoveryPhase('idle');
            interruptedTaskRef.current = { root: '', module: '' };
          }
        });
      })
      .catch(() => {
        if (!isMounted) return;

        connectedRef.current = false;
        setConnected(false);
        setRobotStatus('초기화 오류');
        addLog('ROS 라이브러리 초기화에 실패했습니다.', 'error');
      });

    return () => {
      isMounted = false;
      if (unsubscribeStatus) {
        unsubscribeStatus();
      }
      client.disconnect();
    };
  }, [bridgeUrl]);

  const requestAction = (action, label) => {
    if (!clientRef.current) {
      addLog('Controller 연결이 아직 준비되지 않았습니다.', 'warning');
      return;
    }

    if (!connected) {
      addLog('Controller node와 연결되지 않았습니다.', 'warning');
      return;
    }

    if (isEmergency) {
      addLog('비상 정지 상태에서는 작업 요청이 차단됩니다.', 'warning');
      return;
    }

    if (opState !== 'IDLE') {
      const lockedMessage =
        opState === 'WORKING'
          ? '로봇이 동작 중이어서 새 작업을 요청할 수 없습니다.'
          : '로봇이 일시 정지 상태여서 새 작업을 요청할 수 없습니다.';
      addLog(lockedMessage, 'warning');
      return;
    }

    try {
      clientRef.current.publish(SERVING_CMD_TOPIC, 'std_msgs/String', {
        data: JSON.stringify({ action }),
      });

      addLog(`${label} 작업을 요청했습니다.`, 'info');
    } catch (err) {
      addLog(`${label} 요청 전송 중 에러가 발생했습니다.`, 'error');
    }
  };

  const handlePause = () => {
    if (!clientRef.current) {
      addLog('Controller 연결이 아직 준비되지 않았습니다.', 'warning');
      return;
    }

    if (!connected) {
      addLog('Controller node와 연결되지 않았습니다.', 'warning');
      return;
    }

    if (isEmergency) {
      addLog('비상 정지 상태입니다. 추가 정지 요청은 보낼 수 없습니다.', 'warning');
      return;
    }

    if (opState !== 'WORKING') {
      const pauseMessage =
        opState === 'PAUSED'
          ? '이미 일시 정지 상태입니다.'
          : '로봇이 동작 중일 때만 일시 정지를 요청할 수 있습니다.';
      addLog(pauseMessage, 'warning');
      return;
    }

    try {
      clientRef.current.publish(PAUSE_CMD_TOPIC, 'std_msgs/String', {
        data: 'PAUSE',
      });

      addLog('일시 정지 요청을 전달했습니다.', 'warning');
    } catch (err) {
      addLog('일시 정지 요청 전송에 실패했습니다.', 'error');
    }
  };

  const handleResume = () => {
    if (!clientRef.current) {
      addLog('Controller 연결이 아직 준비되지 않았습니다.', 'warning');
      return;
    }

    if (!connected) {
      addLog('Controller node와 연결되지 않았습니다.', 'warning');
      return;
    }

    if (opState !== 'PAUSED') {
      addLog('일시 정지 상태일 때만 재개를 요청할 수 있습니다.', 'warning');
      return;
    }

    try {
      clientRef.current.publish(PAUSE_CMD_TOPIC, 'std_msgs/String', {
        data: 'RESUME',
      });

      addLog('작업 재개 요청을 전달했습니다.', 'info');
    } catch (err) {
      addLog('작업 재개 요청 전송에 실패했습니다.', 'error');
    }
  };

  const handleEmergency = () => {
    if (!clientRef.current) {
      addLog('Controller 연결이 아직 준비되지 않았습니다.', 'warning');
      return;
    }

    const newState = !isEmergency;
    const action = newState ? 'EMERGENCY_STOP' : 'RESET';

    try {
      clientRef.current.publish(SAFETY_CMD_TOPIC, 'std_msgs/String', {
        data: action,
      });

      if (newState) {
        addLog('비상 정지 버튼이 눌렸습니다!', 'error');
      } else {
        addLog('안전 모드가 해제되었습니다.', 'success');
      }
    } catch (err) {
      addLog('비상 정지 명령 전송 에러', 'error');
    }
  };

  // === Phase 1: 복구 방법 선택 ===

  // 자동 복구: SAFE_* → STANDBY (stop_type=6)
  const handleAutoRecovery = async () => {
    setRecoveryBusy('auto');
    setRecoveryError('');

    try {
      const res = await requestStopTask(STOP_TYPES.TO_STANDBY);
      ensureServiceSuccess(res, '자동 복구 요청에 실패했습니다.');
      addLog('자동 복구 요청을 보냈습니다. STANDBY로 전환 중...', 'warning');
      // 상태 전이는 subscription에서 감지해서 phase2로 전환
    } catch (err) {
      setRecoveryError(err.message || '자동 복구 요청에 실패했습니다.');
      addLog(`자동 복구 실패: ${err.message}`, 'error');
    } finally {
      setRecoveryBusy('');
    }
  };

  // 수동 복구 진입: SAFE_*2 → RECOVERY (stop_type=7)
  const handleManualRecoveryStart = async () => {
    setRecoveryBusy('manual');
    setRecoveryError('');

    try {
      const res = await requestStopTask(STOP_TYPES.TO_RECOVERY);
      ensureServiceSuccess(res, '수동 복구 모드 진입에 실패했습니다.');
      addLog('RECOVERY 모드 진입 중... 로봇을 손으로 안전한 위치로 이동하세요.', 'warning');
      // 상태 전이는 subscription에서 감지해서 manual phase로 전환
    } catch (err) {
      setRecoveryError(err.message || '수동 복구 모드 진입에 실패했습니다.');
      addLog(`수동 복구 진입 실패: ${err.message}`, 'error');
    } finally {
      setRecoveryBusy('');
    }
  };

  // === Manual Phase: 이동 완료 ===

  // 수동 복구 완료: RECOVERY → STANDBY (stop_type=8)
  const handleManualRecoveryDone = async () => {
    setRecoveryBusy('done');
    setRecoveryError('');

    try {
      const res = await requestStopTask(STOP_TYPES.RECOVERY_DONE);
      ensureServiceSuccess(res, '이동 완료 처리에 실패했습니다.');
      addLog('이동 완료! STANDBY로 전환 중...', 'info');
      // 상태 전이는 subscription에서 감지해서 phase2로 전환
    } catch (err) {
      setRecoveryError(err.message || '이동 완료 처리에 실패했습니다.');
      addLog(`이동 완료 실패: ${err.message}`, 'error');
    } finally {
      setRecoveryBusy('');
    }
  };

  // === Phase 2: 후속 동작 선택 ===

  // 홈으로 이동 — recovery_home 은 그리퍼 해제 후 홈으로 이동 (TASK_REGISTRY 참고).
  // 평상 시 "🏠 홈으로" 버튼도 home 태스크가 [gripper_open, task_home] 조합이라
  // 자동으로 그리퍼가 열린 뒤 이동한다.
  const handleGoHome = async () => {
    setRecoveryBusy('home');
    setRecoveryError('');

    try {
      const res = await requestStartTask('recovery_home');
      ensureServiceSuccess(res, '홈 이동 요청에 실패했습니다.');
      addLog('그리퍼 해제 후 홈 위치로 이동합니다.', 'info');
      setRecoveryPhase('idle');
      interruptedTaskRef.current = { root: '', module: '' };
    } catch (err) {
      setRecoveryError(err.message || '홈 이동 요청에 실패했습니다.');
      addLog(`홈 이동 실패: ${err.message}`, 'error');
    } finally {
      setRecoveryBusy('');
    }
  };

  // 작업 재시작 — 중단된 (root, module) 기준으로 이어서 실행할 task 를 선택.
  //   root='auto_serving', module='tong' → resume_from_tong (tong → sauce)
  //   root='tong',         module='tong' → recovery_tong   (tong 만)
  const handleRestartTask = async () => {
    setRecoveryBusy('restart');
    setRecoveryError('');

    const { root: capturedRoot = '', module: capturedModule = '' } =
      interruptedTaskRef.current || {};
    const fallbackRoot = capturedRoot || activeActionKey || '';
    const taskName = resolveResumeTask(fallbackRoot, capturedModule);

    try {
      const res = await requestStartTask(taskName);
      ensureServiceSuccess(res, `${taskName} 작업 시작에 실패했습니다.`);
      addLog(`${taskName} 작업을 시작합니다.`, 'info');
      setRecoveryPhase('idle');
      interruptedTaskRef.current = { root: '', module: '' };
    } catch (err) {
      setRecoveryError(err.message || '작업 재시작에 실패했습니다.');
      addLog(`작업 재시작 실패: ${err.message}`, 'error');
    } finally {
      setRecoveryBusy('');
    }
  };

  // 복구 취소 (phase2에서 idle로 돌아가기)
  const handleCancelRecovery = () => {
    setRecoveryPhase('idle');
    setRecoveryError('');
    interruptedTaskRef.current = { root: '', module: '' };
    addLog('복구를 취소하고 대기 상태로 돌아갑니다.', 'info');
  };

  // system_state 기반 파생 상태 (단순화)
  const isRecoveryBusy = Boolean(recoveryBusy);
  const isInRecoveryFlow = recoveryPhase !== 'idle';

  // 복구 필요 상태 판단 (10-19, EMERGENCY 제외)
  const needsRecovery = (systemState >= 10 && systemState < 20) 
                        && systemState !== SYSTEM_STATE.EMERGENCY;
  const isSafeStop = systemState === SYSTEM_STATE.SAFE_STOP || systemState === SYSTEM_STATE.SAFE_STOP_2;
  const is2Series = systemState === SYSTEM_STATE.SAFE_STOP_2;
  const isRecoveryMode = systemState === SYSTEM_STATE.RECOVERY;
  const isRecovered = systemState === SYSTEM_STATE.RECOVERED;
  const isEmergencyState = systemState === SYSTEM_STATE.EMERGENCY;

  // 안내 토스트 key — (상태, 복구 phase, 연결) 조합이 바뀌면 값이 달라져
  // 사용자가 X 로 닫았던 숨김이 자동 해제된다.
  const toastKey = `${systemState}|${recoveryPhase}|${connected ? 1 : 0}`;
  const toastShouldRender =
    (!connected || isEmergencyState || needsRecovery || isRecovered) &&
    dismissedToastKey !== toastKey;

  // 정상 상태 판단
  const isIdle = systemState === SYSTEM_STATE.IDLE;
  const isWorking = systemState === SYSTEM_STATE.WORKING;
  const isPaused = systemState === SYSTEM_STATE.PAUSED;

  // 버튼 활성화 조건 (단순화)
  const canRequestTask = connected && isIdle && !isRecoveryBusy && !isInRecoveryFlow;
  const canPauseTask = connected && isWorking && !isRecoveryBusy;
  const canRunBasicAction = canRequestTask;

  // 복구 패널 표시 조건
  const showRecoveryPanel = needsRecovery || isRecovered || isInRecoveryFlow;

  const currentStateLabel = SYSTEM_STATE_LABELS[systemState] || systemStateName || '알 수 없음';
  const currentActionLabel = activeAction || '준비 완료';

  // CircularGauge용 theme 객체
  const theme = (() => {
    if (!connected) {
      return { label: '오프라인', icon: '📡', color: '#94a3b8' };
    }
    if (isEmergencyState) {
      return { label: '비상 정지', icon: '🚨', color: '#ef4444' };
    }
    if (isSafeStop) {
      return { label: '안전 정지', icon: '🛑', color: '#f59e0b' };
    }
    if (isRecoveryMode) {
      return { label: '복구 모드', icon: '🤚', color: '#f59e0b' };
    }
    if (isRecovered) {
      return { label: '복구 완료', icon: '🛠', color: '#10b981' };
    }
    if (isPaused) {
      return { label: '일시 정지', icon: '⏸️', color: '#f59e0b' };
    }
    if (isWorking) {
      return { label: '가동 중', icon: '🥣', color: '#3b82f6' };
    }
    return { label: '대기 중', icon: '⋯', color: '#64748b' };
  })();

  // Status Guide 토스트 텍스트 소스.
  // 보호모드/EMERGENCY/연결끊김/복구모드/복구완료 상태에서 헤더 아래 중앙에 고정 토스트로 노출된다.
  // ─────────────────────────────────────────────────────────────
  // 안내 문구 수정 위치: 아래 statusGuide 객체의 각 분기(eyebrow/description).
  // description 은 "발생 원인 + 다음 행동" 을 담은 한 문장으로 작성한다.
  // ─────────────────────────────────────────────────────────────
  const statusGuide = (() => {
    if (!connected) {
      return {
        eyebrow: 'Connection',
        icon: '📡',
        description: 'ROS bridge 연결을 기다리는 중입니다. 네트워크가 정상화되면 자동으로 복구됩니다.',
        cardClass: 'border-slate-200 bg-white text-slate-700',
        badgeClass: 'bg-slate-100 text-slate-500',
      };
    }

    if (isEmergencyState) {
      return {
        eyebrow: 'Emergency',
        icon: '🚨',
        description: '비상 정지가 활성화되어 모든 작업이 차단되었습니다. 주변 안전을 확인한 뒤 헤더의 긴급 정지 해제 버튼을 눌러주세요.',
        cardClass: 'border-red-200 bg-red-50 text-red-900',
        badgeClass: 'bg-red-100 text-red-600',
      };
    }

    if (isSafeStop) {
      return {
        eyebrow: 'Safety Stop',
        icon: '🛑',
        description: '안전 모드가 활성화되었습니다. 좌측 복구 시작하기 버튼을 눌러 복구해주세요.',
        cardClass: 'border-amber-200 bg-amber-50 text-amber-900',
        badgeClass: 'bg-amber-100 text-amber-700',
      };
    }

    if (isRecoveryMode) {
      return {
        eyebrow: 'Recovery Mode',
        icon: '🤚',
        description: '수동 복구 모드입니다. 로봇을 손으로 안전한 위치로 이동한 뒤 좌측 "이동 완료" 버튼을 눌러주세요.',
        cardClass: 'border-amber-200 bg-amber-50 text-amber-900',
        badgeClass: 'bg-amber-100 text-amber-700',
      };
    }

    if (isRecovered) {
      return {
        eyebrow: 'Recovered',
        icon: '🛠',
        description: '복구가 완료되었습니다. 좌측 "마저 진행하기" 로 중단 지점부터 이어서 진행하거나 "홈으로" 를 눌러 정리할 수 있습니다.',
        cardClass: 'border-emerald-200 bg-emerald-50 text-emerald-950',
        badgeClass: 'bg-emerald-100 text-emerald-700',
      };
    }

    if (isWorking) {
      return {
        eyebrow: 'In Progress',
        icon: '🥣',
        description: `${currentActionLabel} 을 수행 중입니다 (${progress}%). 필요 시 좌측 일시 정지 버튼으로 즉시 중단할 수 있습니다.`,
        cardClass: 'border-blue-200 bg-blue-50 text-blue-950',
        badgeClass: 'bg-blue-100 text-blue-700',
      };
    }

    if (isPaused) {
      return {
        eyebrow: 'Paused',
        icon: '⏸️',
        description: '작업이 일시 정지되었습니다. 좌측 재개 버튼으로 이어서 진행하거나 홈 복귀로 정리하세요.',
        cardClass: 'border-orange-200 bg-orange-50 text-orange-950',
        badgeClass: 'bg-orange-100 text-orange-700',
      };
    }

    if (robotStateName === 'HOMMING') {
      return {
        eyebrow: 'Returning Home',
        icon: '🏠',
        description: '로봇이 홈 위치로 복귀 중입니다. 복귀가 끝나면 다음 작업을 시작할 수 있습니다.',
        cardClass: 'border-indigo-200 bg-indigo-50 text-indigo-950',
        badgeClass: 'bg-indigo-100 text-indigo-700',
      };
    }

    return {
      eyebrow: 'Ready',
      icon: '✅',
      description: `현재 로봇 상태는 ${formatRobotStateName(robotStateName)} 이며 새 작업 요청을 받을 수 있습니다.`,
      cardClass: 'border-emerald-200 bg-emerald-50 text-emerald-950',
      badgeClass: 'bg-emerald-100 text-emerald-700',
    };
  })();

  // 헤더 비상 버튼 스타일
  const emergencyButtonClass = !connected
    ? 'bg-slate-100 text-slate-300 border border-slate-100'
    : isEmergency
      ? 'bg-emerald-600 text-white border border-emerald-700 hover:bg-emerald-500 shadow-lg shadow-emerald-100'
      : 'bg-red-50 text-red-600 border border-red-200 hover:bg-red-100 shadow-lg shadow-red-100';

  // 재시작 버튼에 어떤 task 를 실행할지 UI 로 표시하기 위한 파생 값들.
  //   - resumeTaskName : 실제 호출할 task 이름 (예: resume_from_tong)
  //   - interruptedModuleLabel : 뱃지에 표시할 한국어 라벨 (예: 🍱 샐러드·돈까스)
  //   - restartSubLabel: 재시작 버튼 하단 설명 문구
  const _interrupted = interruptedTaskRef.current || { root: '', module: '' };
  const _fallbackRoot = _interrupted.root || activeActionKey || '';
  const resumeTaskName = resolveResumeTask(_fallbackRoot, _interrupted.module);
  const _effectiveModule =
    _interrupted.module && _interrupted.module !== 'gripper_open'
      ? _interrupted.module
      : _fallbackRoot;
  const interruptedModuleIcon = MODULE_ICONS[_effectiveModule] || '🥣';
  const interruptedModuleLabel = MODULE_LABELS[_effectiveModule] || _effectiveModule || '';
  const isResumingAutoServing = resumeTaskName.startsWith('resume_from_');
  const restartSubLabel = isResumingAutoServing
    ? RESUME_DESC_FOR[resumeTaskName] || '마저 진행하기를 누르면 자동 배식을 이어서 진행합니다'
    : interruptedModuleLabel
      ? `마저 진행하기를 누르면 ${interruptedModuleIcon} ${interruptedModuleLabel}을 다시 시작합니다`
      : '마저 진행하기를 누르면 자동 배식을 다시 시작합니다'
  // Phase 1: 복구 방법 선택 패널
  // 뱃지/에러 배치는 버튼 아래 — 뱃지 유무에 따라 버튼 세로 위치가 흔들리지 않도록.
  const phase1Panel = html`
    <div className="flex flex-col gap-3">
      <button
        onClick=${handleAutoRecovery}
        disabled=${isRecoveryBusy}
        className=${`w-full py-5 rounded-2xl shadow-lg transition-all active:scale-95 flex flex-row items-center justify-center gap-3 ${
          isRecoveryBusy
            ? 'bg-gray-200 text-gray-400 shadow-none'
            : 'bg-blue-600 hover:bg-blue-500 text-white shadow-blue-100'
        }`}
      >
        <span className="text-2xl">⚡</span>
        <span className="text-lg font-black">
          ${recoveryBusy === 'auto' ? '복구 중...' : '복구 시작하기'}
        </span>
      </button>
      ${is2Series && html`
        <button
          onClick=${handleManualRecoveryStart}
          disabled=${isRecoveryBusy}
          className=${`w-full py-4 rounded-2xl shadow-lg transition-all active:scale-95 flex flex-row items-center justify-center gap-3 ${
            isRecoveryBusy
              ? 'bg-gray-200 text-gray-400 shadow-none'
              : 'bg-amber-500 hover:bg-amber-400 text-white shadow-amber-100'
          }`}
        >
          <span className="text-2xl">🤚</span>
          <span className="text-lg font-black">
            ${recoveryBusy === 'manual' ? '진입 중...' : '수동 복구'}
          </span>
        </button>
      `}
      ${interruptedModuleLabel && html`
        <div className="rounded-xl border border-amber-200 bg-amber-50 px-3 py-2 text-center text-sm font-semibold text-amber-700">
          중단된 작업 · ${interruptedModuleIcon} ${interruptedModuleLabel}
          ${isAutoServingRoot(_interrupted.root) && _interrupted.module && _interrupted.module !== _interrupted.root
            ? ' (자동 배식 중)'
            : ''}
        </div>
      `}
      ${recoveryError && html`
        <div className="rounded-2xl border border-red-100 bg-red-50 px-4 py-3 text-sm font-semibold text-red-600">
          ${recoveryError}
        </div>
      `}
    </div>
  `;

  // Manual Phase: RECOVERY 모드 — 이동 완료 대기. (안내 문구는 상단 Toast 가 담당)
  const manualPanel = html`
    <div className="flex flex-col gap-3">
      <button
        onClick=${handleManualRecoveryDone}
        disabled=${isRecoveryBusy}
        className=${`w-full py-5 rounded-2xl shadow-lg transition-all active:scale-95 flex flex-row items-center justify-center gap-3 ${
          isRecoveryBusy
            ? 'bg-gray-200 text-gray-400 shadow-none'
            : 'bg-emerald-600 hover:bg-emerald-500 text-white shadow-emerald-100'
        }`}
      >
        <span className="text-2xl">✅</span>
        <span className="text-xl font-black">
          ${recoveryBusy === 'done' ? '처리 중...' : '이동 완료'}
        </span>
      </button>
      ${recoveryError && html`
        <div className="rounded-2xl border border-red-100 bg-red-50 px-4 py-3 text-sm font-semibold text-red-600">
          ${recoveryError}
        </div>
      `}
    </div>
  `;

  // Phase 2: 후속 동작 선택 패널. (안내 문구는 상단 Toast 가 담당)
  // 좌: 마저 진행하기(primary green), 우: 홈으로(subtle white — 헤더 홈 버튼과 동일 톤).
  const phase2Panel = html`
    <div className="flex flex-col gap-3">
      <div className="grid grid-cols-2 gap-3">
        <button
          onClick=${handleRestartTask}
          disabled=${isRecoveryBusy}
          className=${`py-4 rounded-2xl shadow-lg transition-all active:scale-95 flex flex-row items-center justify-center gap-2 ${
            isRecoveryBusy
              ? 'bg-gray-200 text-gray-400 shadow-none'
              : 'bg-emerald-600 hover:bg-emerald-500 text-white shadow-emerald-100'
          }`}
        >
          <span className="text-xl">▶️</span>
          <span className="text-base font-black">
            ${recoveryBusy === 'restart' ? '시작 중...' : '마저 진행하기'}
          </span>
        </button>
        <button
          onClick=${handleGoHome}
          disabled=${isRecoveryBusy}
          className=${`py-4 rounded-2xl border transition-all active:scale-95 flex flex-row items-center justify-center gap-2 ${
            isRecoveryBusy
              ? 'bg-slate-100 border-transparent text-slate-300'
              : 'bg-white hover:bg-slate-50 border-slate-200 text-slate-600 shadow-sm'
          }`}
        >
          <span className="text-xl">🏠</span>
          <span className="text-base font-black">
            ${recoveryBusy === 'home' ? '이동 중...' : '홈으로'}
          </span>
        </button>
      </div>
      ${restartSubLabel && html`
        <div className="rounded-xl bg-emerald-50 border border-emerald-100 px-3 py-2 text-center text-xs font-bold text-emerald-700">
          ${restartSubLabel}
        </div>
      `}
      ${recoveryError && html`
        <div className="rounded-2xl border border-red-100 bg-red-50 px-4 py-3 text-sm font-semibold text-red-600">
          ${recoveryError}
        </div>
      `}
    </div>
  `;

  // 복구 패널 선택 (system_state 기반)
  const recoveryPanel = 
    isRecoveryMode || recoveryPhase === 'manual' ? manualPanel :
    isRecovered || recoveryPhase === 'phase2' ? phase2Panel :
    phase1Panel;

  const primaryControlPanel = showRecoveryPanel
    ? recoveryPanel
    : isWorking
      ? html`
          <button
            onClick=${handlePause}
            disabled=${!canPauseTask}
            className="w-full py-5 bg-orange-500 hover:bg-orange-400 text-white rounded-2xl shadow-lg shadow-orange-100 transition-all active:scale-95 flex flex-row items-center justify-center gap-3"
          >
            <span className="text-2xl">⏸️</span>
            <span className="text-xl font-black">일시 정지</span>
          </button>
        `
      : isPaused
        ? html`
            <button
              onClick=${handleResume}
              className="w-full py-5 bg-green-600 hover:bg-green-500 text-white rounded-2xl shadow-lg shadow-green-100 transition-all active:scale-95 flex flex-row items-center justify-center gap-3 disabled:bg-gray-200 disabled:text-gray-400 disabled:shadow-none"
            >
              <span className="text-2xl">▶️</span>
              <span className="text-xl font-black">재개</span>
            </button>
          `
        : html`
            <button
              onClick=${() => requestAction('auto_serving', '자동 배식')}
              disabled=${!canRequestTask}
              className=${`w-full py-5 rounded-2xl shadow-lg transition-all active:scale-95 flex flex-row items-center justify-center gap-3 ${
                canRequestTask ? 'bg-blue-600 hover:bg-blue-500 text-white shadow-blue-100' : 'bg-gray-200 text-gray-400'
              }`}
            >
              <span className="text-2xl">🥣</span>
              <span className="text-xl font-black">작업 시작</span>
            </button>
          `;

  // 하단 "상세 정보" — payload 의 system_status / task_status / robot_status 를
  // key/value rows 로 그대로 덤프. 디버깅용. 중첩 값은 formatRawValue 로 한 줄 요약.
  const RAW_STATUS_KEYS = ['system_status', 'task_status', 'robot_status'];

  const logItems = logs.length
    ? logs.map((log, index) => html`
        <div
          key=${`${log.timestamp}-${index}`}
          className="flex gap-3 text-[11px] p-3 bg-white rounded-xl border border-slate-100 items-center"
        >
          <span className="text-slate-400 font-mono shrink-0">${log.timestamp}</span>
          <span
            className=${`font-bold ${
              log.type === 'error'
                ? 'text-red-500'
                : log.type === 'warning'
                  ? 'text-orange-600'
                  : log.type === 'success'
                    ? 'text-emerald-600'
                    : 'text-slate-700'
            }`}
          >
            ${log.message}
          </span>
        </div>
      `)
    : [
        html`
          <div key="empty-log" className="text-center text-slate-300 py-10 text-sm font-medium">
            활동 기록이 없습니다.
          </div>
        `,
      ];

  // 홈 복귀 버튼은 IDLE 일 때만 노출. PAUSED/EMERGENCY/SAFE_STOP/RECOVERY 에서는
  // 각각 전용 컨트롤(일시정지 해제·복구 패널·토스트)이 동작하므로 disabled 상태의
  // 홈 버튼이 중복 노출되는 것을 방지한다.
  const showHomeButton = isIdle && connected;

  // 오른쪽 패널: 현재 task 의 모듈 진행 상황 (진행/완료/남음 bullet list)
  //
  // - 완료(index < moduleIndex): 초록 체크 + 라벨 strikethrough + dim + "완료" 태그
  // - 진행(index === moduleIndex): 파란 ▶ + bold primary + "진행 중" 태그
  // - 남음(index > moduleIndex): 회색 ○ + dim
  //
  // resume_from_tong/sauce 는 motion_executor 가 실제로 실행하지 않는 이전 모듈(rice 등)
  // 을 "이미 완료된 것" 으로 보여주기 위해 SKIPPED_BEFORE 기반으로 가상 prepend 한다
  // (auto_serving 도중 SAFE_STOP → 복구 시, 이미 끝난 rice 가 건너뛴 것이 아니라 완료됐으므로
  // "완료" 태그로 동일 표기). gripper_open 은 payload 에 이미 포함돼 있으므로 그 뒤에 삽입해
  // auto_serving 과 동일한 [gripper_open, rice, tong, sauce] 순서를 재현한다.
  //
  // 작업이 진행 중이 아니면(effectiveTaskModules 가 비어 있으면) 준비 상태 placeholder 표시.
  const skippedBefore = SKIPPED_BEFORE[activeActionKey] || [];
  const virtualDoneModules = skippedBefore.map((name) => ({
    name,
    label: MODULE_LABELS[name] || name,
    icon: MODULE_ICONS[name] || '',
    __virtualDone: true,
  }));
  const effectiveTaskModules = (() => {
    if (!Array.isArray(taskModules) || taskModules.length === 0) return taskModules || [];
    if (virtualDoneModules.length === 0) return taskModules;
    const [head, ...rest] = taskModules;
    return head && head.name === 'gripper_open'
      ? [head, ...virtualDoneModules, ...rest]
      : [...virtualDoneModules, ...taskModules];
  })();
  const hasSteps = effectiveTaskModules.length > 0;
  // 실제 실행 모듈 내의 active index 는 moduleIndex. virtual 모듈 갯수만큼 표시 offset 을
  // 더해야 effectiveTaskModules 내 위치와 일치한다.
  const virtualOffset = virtualDoneModules.length > 0 ? virtualDoneModules.length : 0;
  const rawActiveIndex = hasSteps && (isWorking || isPaused) ? moduleIndex : -1;
  const activeModuleIndex = rawActiveIndex >= 0 && virtualOffset > 0
    // gripper_open(= index 0) 뒤에 virtual 삽입이므로, rawActiveIndex 가 0 이면 그대로,
    // 1 이상이면 virtualOffset 만큼 밀린다.
    ? (rawActiveIndex === 0 ? 0 : rawActiveIndex + virtualOffset)
    : rawActiveIndex;
  const totalSteps = hasSteps ? effectiveTaskModules.length : 0;
  const stepHeading = hasSteps
    ? `${activeAction || currentActionLabel} · ${Math.max(1, Math.min(activeModuleIndex + 1, totalSteps))}/${totalSteps}`
    : '대기 중';

  const renderStepRow = (mod, index) => {
    const isVirtualDone = !!mod.__virtualDone;
    const isDone = isVirtualDone || (activeModuleIndex >= 0 && index < activeModuleIndex);
    const isActive = !isVirtualDone && activeModuleIndex >= 0 && index === activeModuleIndex;
    const marker = isDone ? '✓' : isActive ? '▶' : '○';
    const rowClass = isActive
      ? 'bg-blue-50 border-blue-200 text-blue-800'
      : isDone
        ? 'bg-white border-slate-100 text-slate-400 line-through'
        : 'bg-white border-slate-100 text-slate-400';
    const markerClass = isActive
      ? 'text-blue-600 font-black'
      : isDone
        ? 'text-emerald-500 font-black'
        : 'text-slate-300 font-black';
    return html`
      <div
        key=${`${mod.name}-${index}`}
        className=${`flex items-center gap-3 px-4 py-3 rounded-2xl border ${rowClass} transition-colors`}
      >
        <span className=${`text-lg w-5 text-center ${markerClass}`}>${marker}</span>
        <span className="text-xl">${mod.icon || MODULE_ICONS[mod.name] || '•'}</span>
        <span className=${`flex-1 text-sm ${isActive ? 'font-black' : 'font-semibold'}`}>
          ${mod.label || MODULE_LABELS[mod.name] || mod.name}
        </span>
        ${isActive && html`
          <span className="text-[10px] font-mono text-blue-500">진행 중</span>
        `}
        ${isDone && html`
          <span className="text-[10px] font-mono text-emerald-500">완료</span>
        `}
      </div>
    `;
  };

  const taskStepsPanel = html`
    <div className="w-full rounded-[28px] border border-slate-200 bg-white px-5 py-5 shadow-sm">
      <div className="flex items-center justify-between mb-3">
        <h4 className="text-[11px] font-black uppercase tracking-widest text-slate-400">Task Steps</h4>
        <span className="text-[11px] font-bold text-slate-500">${stepHeading}</span>
      </div>
      ${hasSteps
        ? html`
            <div className="space-y-2">
              ${effectiveTaskModules.map((mod, i) => renderStepRow(mod, i))}
            </div>
          `
        : html`
            <div className="text-center py-8 text-slate-300 text-sm font-medium">
              작업이 시작되면 단계가 여기에 표시됩니다.
            </div>
          `}
    </div>
  `;

  // 당일(KST) 에러 로그 패널: Firebase 에서 bootstrap 되어 새로고침해도 유지.
  const todayErrorItems = (todayErrors || [])
    .slice()
    .reverse()
    .slice(0, 20)
    .map((err, idx) => html`
      <div
        key=${`${err.id || err.created_at || idx}-${idx}`}
        className="flex gap-3 text-[11px] p-3 bg-white rounded-xl border border-red-100 items-start"
      >
        <span className="text-red-400 font-mono shrink-0 w-16">
          ${formatClock(err.created_at)}
        </span>
        <div className="flex-1 min-w-0">
          <div className="text-red-600 font-bold truncate">${err.message || '(메시지 없음)'}</div>
          ${(err.task_name || err.step_name) && html`
            <div className="text-slate-400 text-[10px] mt-1 truncate">
              task: ${err.task_name || '-'} · step: ${err.step_name || '-'}
            </div>
          `}
        </div>
      </div>
    `);

  return html`
    <div className="min-h-screen w-full font-sans text-slate-900">
      <!-- Header Bar (sticky, 단일 타이틀 + Home(조건부) + Safety 버튼) -->
      <header className="sticky top-0 z-40 bg-white border-b border-slate-200 px-6 py-3 flex items-center justify-between gap-6 shadow-sm">
        <div className="flex items-center gap-3">
          <div className=${`w-3 h-3 rounded-full ${connected ? 'bg-green-500 animate-pulse' : 'bg-red-500'}`}></div>
          <h1 className="text-lg font-black tracking-tight">배식 로봇 대시보드</h1>
        </div>
        <div className="flex items-center gap-3">
          ${showHomeButton && html`
            <button
              onClick=${() => requestAction('recovery_home', '홈 위치 이동 (그리퍼 해제)')}
              disabled=${!canRequestTask}
              className=${`rounded-2xl px-4 py-2.5 font-black transition-all flex items-center justify-center gap-2 border ${
                canRequestTask
                  ? 'bg-white hover:bg-slate-50 border-slate-200 text-slate-600 shadow-sm'
                  : 'bg-slate-100 border-transparent text-slate-300'
              }`}
            >
              <span className="text-xl">🏠</span>
              <span className="text-sm">홈으로</span>
            </button>
          `}
          <button
            onClick=${handleEmergency}
            disabled=${!connected}
            className=${`min-w-[180px] rounded-2xl px-4 py-2.5 font-black transition-all flex items-center justify-center gap-3 ${emergencyButtonClass}`}
          >
            <span className="text-2xl">${isEmergency ? '✅' : '🚨'}</span>
            <span className="text-sm">${isEmergency ? '긴급 정지 해제' : '긴급 정지'}</span>
          </button>
        </div>
      </header>

      <!-- 상태 안내 Toast — 보호모드/EMERGENCY/연결끊김/복구모드/복구완료 동안 고정 노출.
           헤더 아래 중앙 정렬, 가로 폭은 최대 4xl. 상태 해제 시 자동 소멸.
           우측 X 버튼으로 닫으면 같은 상태 조합 동안은 숨김 (toastKey 기반).
           레이아웃: [아이콘] [EYEBROW 태그 / 합쳐진 한 문장] [X]. 텍스트 수정은 statusGuide 객체에서 한다. -->
      ${toastShouldRender && html`
        <div className="fixed top-[72px] left-1/2 -translate-x-1/2 z-30 w-[calc(100%-3rem)] max-w-4xl px-0 pointer-events-none">
          <div className=${`mt-3 rounded-2xl border px-5 py-3 shadow-lg pointer-events-auto flex items-start gap-4 ${statusGuide.cardClass}`}>
            <div className=${`shrink-0 rounded-xl px-3 py-2 text-xl font-black ${statusGuide.badgeClass}`}>
              ${statusGuide.icon}
            </div>
            <div className="flex-1 min-w-0">
              <p className="text-[10px] font-black uppercase tracking-widest opacity-70">${statusGuide.eyebrow}</p>
              <p className="mt-1 text-sm font-semibold leading-relaxed">${statusGuide.description}</p>
            </div>
            <button
              type="button"
              aria-label="알림 닫기"
              onClick=${() => setDismissedToastKey(toastKey)}
              className="shrink-0 -mr-1 -mt-1 h-7 w-7 rounded-full text-current opacity-60 hover:opacity-100 hover:bg-black/5 transition-opacity flex items-center justify-center text-sm font-black"
            >
              ✕
            </button>
          </div>
        </div>
      `}

      <!-- Main Content -->
      <main className="max-w-[1600px] mx-auto px-6 py-6 pb-10">
        <!-- Hero Section -->
        <section className="bg-white rounded-[32px] border border-slate-100 shadow-sm p-6 md:p-8">
          <div className="grid grid-cols-1 xl:grid-cols-[1.6fr_1fr] gap-8 items-start">
            <!-- Left Column: Gauge + Buttons -->
            <div className="flex flex-col items-center justify-center">
              <${CircularGauge}
                progress=${progress}
                statusInfo=${theme}
                subTask=${isWorking
                  ? { label: subTaskLabel, icon: subTaskIcon, phase: subTaskPhase }
                  : null}
                pulseIcon=${isIdle && connected}
              />

              <!-- Primary Control Panel -->
              <!-- '홈으로' 버튼은 헤더 우측(Emergency 버튼 좌측)으로 이동. 조건도 헤더에서 관리한다. -->
              <div className="mt-8 w-full max-w-md">
                ${primaryControlPanel}
              </div>
            </div>

            <!-- Right Column: Task Steps (Status Guide 는 상단 Toast 로 이동) -->
            <div className="flex flex-col justify-start gap-6">
              ${taskStepsPanel}
            </div>
          </div>
        </section>

        <!-- Activity Logs Section -->
        <section className="mt-8">
          <div className="bg-white rounded-[32px] border border-slate-200 p-5 shadow-sm">
            <div className="flex items-center justify-between gap-3 mb-4">
              <h3 className="text-xs font-black text-slate-400 uppercase tracking-widest ml-1">Activity Logs</h3>
              <span className="text-[10px] font-mono text-slate-400">updated ${lastUpdatedAt}</span>
            </div>
            <div className="space-y-2 max-h-[200px] overflow-y-auto status-scrollbar">
              ${logItems}
            </div>
          </div>
        </section>

        <!-- 당일(KST) 에러 로그 패널 — ui_bridge 가 Firebase 에서 bootstrap. 새로고침해도 유지. -->
        <section className="mt-8">
          <div className="bg-white rounded-[32px] border border-red-100 p-5 shadow-sm">
            <div className="flex items-center justify-between gap-3 mb-4">
              <h3 className="text-xs font-black text-red-500 uppercase tracking-widest ml-1">
                오늘의 에러 (${todayErrors.length})
              </h3>
              <span className="text-[10px] font-mono text-slate-400">KST 기준 당일 누적</span>
            </div>
            <div className="space-y-2 max-h-[240px] overflow-y-auto status-scrollbar">
              ${todayErrorItems.length
                ? todayErrorItems
                : html`
                    <div className="text-center text-slate-300 py-8 text-sm font-medium">
                      오늘 기록된 에러가 없습니다.
                    </div>
                  `}
            </div>
          </div>
        </section>

        <!-- 상세 정보 (raw) — payload 의 system/task/robot status 를 key/value 로 그대로 덤프 -->
        <section className="mt-8">
          <div className="bg-white rounded-[32px] border border-slate-200 p-5 shadow-sm">
            <div className="flex items-center justify-between gap-3 mb-4">
              <h3 className="text-xs font-black text-slate-400 uppercase tracking-widest ml-1">상세 정보 (raw)</h3>
              <span className="text-[10px] font-mono text-slate-400">/${ROBOT_NAMESPACE}/ui/status</span>
            </div>
            <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
              ${RAW_STATUS_KEYS.map((key) => {
                const data = rawStatus[key] || {};
                const entries = Object.entries(data);
                return html`
                  <div key=${key} className="rounded-2xl border border-slate-100 bg-slate-50 p-4">
                    <p className="text-[10px] font-black uppercase tracking-widest text-slate-400 mb-2">${key}</p>
                    ${entries.length
                      ? html`
                          <div className="space-y-1 text-[11px] font-mono">
                            ${entries.map(([k, v]) => html`
                              <div key=${k} className="flex gap-2">
                                <span className="text-slate-400 shrink-0">${k}:</span>
                                <span className="text-slate-800 break-all">${formatRawValue(v)}</span>
                              </div>
                            `)}
                          </div>
                        `
                      : html`
                          <p className="text-[11px] font-mono text-slate-300">(empty)</p>
                        `}
                  </div>
                `;
              })}
            </div>
          </div>
        </section>
      </main>
    </div>
  `;
};

const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(html`<${App} />`);
