# rclpy Executor와 DSR_ROBOT2 공존 트러블슈팅

`cobot1/motion_executor` 구현 과정에서 마주친 **"start는 되는데 pause → stop
이후 hang된다"** / **"start부터 check_motion이 timeout된다"** 문제와, 그 원인
분석·해결 과정을 정리한 문서다. 핵심 주제는 **rclpy의 Executor 모델**과
**DSR_ROBOT2 Python wrapper의 동기 호출 방식**이 한 노드 위에서 어떻게
충돌하는지, 그리고 이를 어떻게 분리해야 하는지다.

---

## 1. 요구사항 & 증상

### 요구사항

- 외부(UI/CLI)에서 `StartTask` / `StopTask` 서비스로 task 실행 제어
- `pause` → `resume` 또는 `pause` → `stop` 이 모두 지원돼야 함
- `stop` 이후에는 `start`만 가능 (resume 불가)
- 실행 중 `RobotStateRt` 토픽을 10Hz로 퍼블리시 (UI·controller 공용 데이터 소스)
- 위 모든 기능이 하나의 노드(`motion_executor`)로 제공

### 증상 (시간 순)


| 시도                                                           | 현상                                                                                                         |
| ------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------- |
| 1차: `DSR_ROBOT2.check_motion()` 사용 + `MultiThreadedExecutor` | `start` / `pause` / `resume` / `stop` 단독은 OK. `**start → pause → stop` 후 `start`가 hang** (CLI 5s timeout). |
| 2차: 직접 `CheckMotion` 서비스 클라이언트 + task generation counter 도입  | **첫 `start`부터 `check_motion kept timing out; aborting wait` 에러**. 로봇이 전혀 움직이지 않음.                          |


---

## 2. 사전 지식: rclpy Executor 모델

### 2.1 `Executor.add_node`

`/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py`:

```python
def add_node(self, node: 'Node') -> bool:
    with self._nodes_lock:
        if node not in self._nodes:
            self._nodes.add(node)
            node.executor = self        # ← 노드의 executor 포인터를 덮어쓴다
            self._guard.trigger()
```

**중요한 불변 조건**: 한 노드는 **하나의 executor만** 소유자(`node.executor`)로
가져야 한다. 두 executor가 각자의 `_nodes` set에 같은 노드를 갖고 있어도, 노드
쪽 `executor` 속성은 마지막으로 `add_node` 한 executor로 덮어씌워진다.

### 2.2 `rclpy.spin_until_future_complete(node, future)`

```python
def spin_until_future_complete(node, future, executor=None, timeout_sec=None):
    executor = get_global_executor() if executor is None else executor
    node_added = False
    try:
        node_added = executor.add_node(node)           # ← 글로벌 executor에 임시 add
        executor.spin_until_future_complete(future, timeout_sec)
    finally:
        if node_added:
            executor.remove_node(node)
```

- `executor=None`이면 **글로벌 `SingleThreadedExecutor`** 를 가져와 사용.
- 호출 시점에 노드를 글로벌 executor에 add → future 완료까지 spin → remove.
- 이 시간 동안 **해당 노드의 모든 callback(서비스·타이머·서브스크립션)** 이
글로벌 executor 쪽으로 유입된다.

### 2.3 `MultiThreadedExecutor` vs `SingleThreadedExecutor`

- **SingleThreaded**: 하나의 스레드가 wait-set을 `wait` 하고, ready callback을 직접
실행. Blocking callback이 있으면 그 사이 다른 callback은 막힘.
- **MultiThreaded**: wait-set spin은 하나의 스레드, 실행은 worker pool 분산.
`ReentrantCallbackGroup` + 여러 thread면 서비스·타이머 병렬 처리 가능.

공통: 내부적으로 `_rclpy.WaitSet`에 노드의 entity handle들을 add해서 `wait`한다.
**동일 handle을 두 wait-set에 동시에 추가**하면 rclpy가 정의하지 않는 상태가
되고, 실제로 이벤트(서비스 응답 도착 등)가 어느 wait-set에서 처리될지 경쟁이
생긴다.

---

## 3. DSR_ROBOT2 Python Wrapper의 동작 방식

`doosan-robot2/dsr_common2/imp/DSR_ROBOT2.py`는 **모듈 import 시점에**
`DR_init.__dsr__node` 를 잡아서 모든 DSR2 서비스 클라이언트를 생성한다.

```python
import DR_init
g_node = DR_init.__dsr__node         # ← import 타이밍에 한 번 캡처
_srv_name_prefix = ''                # ← 빈 문자열 → 노드 namespace 상속
_ros2_movej   = g_node.create_client(MoveJoint, "motion/move_joint")
_ros2_movel   = g_node.create_client(MoveLine,  "motion/move_line")
_ros2_check_motion = g_node.create_client(CheckMotion, "motion/check_motion")
# ... 수십 개 더
```

그리고 모든 호출 함수는 **동기 패턴**이다:

```python
def check_motion():
    req = CheckMotion.Request()
    future = _ros2_check_motion.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)   # ← 핵심
    ...
```

즉 DSR_ROBOT2는:

- `g_node`가 **어느 executor에도 속해 있지 않다**는 가정으로 설계됨.
- 호출 순간에만 **글로벌 SingleThreadedExecutor**가 잠깐 이 노드를 가져가
wait-set에 올리고 → 응답 받고 → 내려놓는다.

Doosan 공식 예제 (`single_robot_simple.py`)도 이 전제를 따른다:

```python
node = rclpy.create_node('single_robot_simple_py', namespace=ROBOT_ID)
DR_init.__dsr__node = node
# ← node를 executor에 add하지 않음. rclpy.spin(node)도 없음.
while rclpy.ok():
    movej(p2, vel=100, acc=100)   # 메인 스레드에서 직접 호출
```

---

## 4. 왜 우리 1차 구현이 깨졌는가

1차 구현의 요약:

```python
node = MotionExecutorNode()                    # namespace=dsr01, 서비스/타이머 여럿
DR_init.__dsr__node = node                     # ← DSR용 g_node로도 등록
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```

그리고 task thread 안에서:

```python
from DSR_ROBOT2 import amovej, check_motion
amovej(JReady, vel=60, acc=60)                 # 내부에서 spin_until_future_complete(node, ...)
while not ctx.check_stop():
    if check_motion() == 0: break              # 내부에서 spin_until_future_complete(node, ...)
    time.sleep(0.05)
```

**발생 현상**:

- `amovej` 호출 → 글로벌 SingleThreadedExecutor가 `add_node(node)` 시도. 이미
MultiThreaded가 `_nodes`에 가지고 있지만, 글로벌 쪽은 **자기 `_nodes`에 없으면
True 리턴하고 추가한다**. `node.executor` 포인터는 **글로벌로 덮어씌워짐**.
- 동시에 MultiThreaded는 여전히 `executor.spin()` 루프에서 이 노드의 wait-set에
대한 참조를 유지한 채 돌고 있다.
- 같은 client handle이 두 wait-set에 올라가 있어 **서비스 response가 한 쪽에만
디스패치**된다. 타이밍상 대부분 글로벌 쪽이 먼저 집어서 `spin_until_future_complete`
가 정상 리턴한다 → **대부분 동작한다**.
- `pause → stop` 시나리오에서는 순서가 꼬인다: `move_pause` 응답은
MultiThreaded가 받고(= `_on_move_pause_done`), task thread는 `check_motion`에
걸려 글로벌 executor를 spin 중. 이때 `move_stop` 서비스 콜백(`_on_stop`)은
MultiThreaded에서 실행되는데, 여기서 보낸 `move_stop` 응답은 또 어디로 갈지
불명확. 결과적으로 `_stop_requested=True` 신호는 정상 전파되지만 `check_motion`
의 future가 done되지 않아 task thread가 hang → watchdog이 3초 후 강제 IDLE로
돌려도 task thread는 여전히 살아있는 zombie → 다음 `start`가 경합으로 실패.

한 줄 요약: **동일한 `Node` 인스턴스를 두 executor가 공유하면 안 된다**.

---

## 5. 왜 2차 구현도 깨졌는가

"그럼 DSR_ROBOT2.check_motion() 대신 내가 직접 `CheckMotion` 클라이언트를
만들어 타임아웃을 주면 MultiThreaded 쪽에서 안전하게 처리되겠지" 라고 생각했다.

```python
self._check_motion_cli = self.create_client(CheckMotion, "/dsr01/motion/check_motion")
def wait_motion(self):
    fut = self._check_motion_cli.call_async(CheckMotion.Request())
    deadline = time.time() + 0.5
    while not fut.done() and time.time() < deadline: time.sleep(0.02)
    ...
```

**하지만 여전히 깨진다**. 이유:

- task thread는 **그 전에** `set_robot_mode`, `set_tool`, `set_tcp`, `amovej` 를
반드시 호출한다. 이들 함수는 전부 `rclpy.spin_until_future_complete(g_node, ...)`.
- 첫 번째 `set_robot_mode` 호출 순간부터 `node.executor`가 **글로벌
SingleThreadedExecutor**로 바뀌고, 같은 handle이 두 wait-set에 동시 등록된다.
- 이후 내 `_check_motion_cli`가 보낸 request의 response가 도착해도, wait-set
경쟁 때문에 **MultiThreaded가 그것을 집어오지 못한다**. future는 영원히
`done()` 이 False. 0.5s timeout × 20회 = 10초 후 `check_motion kept timing out; aborting wait` 에러.

즉 client를 누가 만들었는지와 무관하다. `**g_node`를 MultiThreaded에 넣은
순간부터 DSR_ROBOT2 쓰면 안 된다**.

---

## 6. 해결: Two-Node 아키텍처

한 프로세스에 **노드 2개**를 둔다. 각 노드의 "소속 executor" 는 서로 다르며,
그것이 **rclpy의 원래 불변 조건을 지킨다**.

```
┌─────────────────────────────────────────────────────────────┐
│ motion_executor (process)                                   │
│                                                             │
│  ┌──────────────────────────┐    ┌──────────────────────┐   │
│  │ control node             │    │ dsr_node             │   │
│  │ (MultiThreadedExecutor)  │    │ (어느 executor에도   │   │
│  │                          │    │  소속되지 않음)      │   │
│  │ - execute_task/start     │    │                      │   │
│  │ - execute_task/stop      │    │ DR_init.__dsr__node  │   │
│  │ - execute_task/state     │    │      = dsr_node      │   │
│  │ - motion_executor/       │    │                      │   │
│  │     robot_state          │    │ DSR_ROBOT2 함수가    │   │
│  │ - MoveStop/Pause/Resume  │    │ 호출될 때만 글로벌   │   │
│  │   (클라이언트)           │    │ SingleThreaded가     │   │
│  │ - ReadDataRt (10Hz 폴링) │    │ 잠깐 add/spin/remove │   │
│  └──────────────────────────┘    └──────────────────────┘   │
│        ▲                                     ▲              │
│        │ shared state (threading.Lock)       │              │
│        │                                     │              │
│        └────────────────┬────────────────────┘              │
│                         │                                   │
│                  ┌──────┴───────┐                           │
│                  │ task thread  │ (단 하나)                 │
│                  │              │                           │
│                  │ DSR_ROBOT2   │ amovej / amovel /         │
│                  │ 함수 직접    │ check_motion /            │
│                  │ 호출         │ set_tool / ...            │
│                  └──────────────┘                           │
└─────────────────────────────────────────────────────────────┘
```

### 6.1 control node

- `rclpy.node.Node("motion_executor", namespace="dsr01")`
- `MultiThreadedExecutor(num_threads=4)` 에 add
- ROS 외부 I/F + DSR2 명령 보내기용 클라이언트 (MoveStop/Pause/Resume/ReadDataRt)
- 서비스/타이머 응답이 모두 이 MultiThreaded worker에서 처리됨. 다른 노드와
wait-set을 공유하지 않음.

### 6.2 dsr_node

- `rclpy.create_node("motion_executor_dsr", namespace="dsr01")`
- `DR_init.__dsr__node = dsr_node`
- **어떤 executor에도 `add_node` 하지 않음**. `rclpy.spin(dsr_node)` 도 절대
호출하지 않음.
- DSR_ROBOT2 함수가 호출되는 순간에만 글로벌 SingleThreadedExecutor가 이 노드를
가져갔다가 돌려준다.

### 6.3 task thread

- `control_node._on_start` 콜백이 `threading.Thread` 를 하나 spawn.
- DSR_ROBOT2 함수 호출은 전부 이 스레드에서만. 글로벌 SingleThreadedExecutor가
싱글스레드이므로 **두 task thread가 동시에 DSR_ROBOT2 함수를 호출하면 안 됨**.
우리 구조는 동시에 task를 실행하지 않으니 문제없음.
- `ctx.check_stop()` → control node의 shared state를 lock으로 읽음.
- `ctx.wait_motion()` 구현:
  ```python
  while not check_stop():
      if DSR_ROBOT2.check_motion() == 0:   # ← dsr_node로만 경유
          return True
      time.sleep(0.05)
  return False
  ```

### 6.4 pause / resume / stop 경로

모두 **control node** 의 DSR2 서비스 클라이언트 (`MovePause`, `MoveResume`,
`MoveStop`) 를 `call_async` 로 전송한다. 응답 콜백도 control node =
MultiThreaded에서 실행. **dsr_node와 완전히 분리**.

- **Pause (RUNNING 중)**: `MovePause`. Task thread는 `check_motion()` 이 BUSY를
리턴하므로 `wait_motion` 루프에서 자연 대기 (50ms sleep).
- **Resume (PAUSED 중)**: `MoveResume`. DSR이 모션 재개 → `check_motion` 이 곧
IDLE(0) 리턴 → `wait_motion` 정상 탈출.
- **Stop (RUNNING / PAUSED)**: shared state의 `_stop_requested=True` 세팅 후
`MoveStop`. PAUSED 상태였다면 **먼저 `MoveResume` 을 날려 DSR pause 상태를 풀고**
완료 콜백에서 `MoveStop` 을 보낸다 (paused 모션에 바로 stop을 주면 DSR 동작이
불안정한 사례 확인). Task thread의 `check_stop()` 이 True가 되면서 `wait_motion`
이 False 리턴 → task `run()` 이 `return False` → `_finish(IDLE)`.

### 6.5 상태 기계

```
        start           pause            resume
  IDLE ───────► RUNNING ──────► PAUSED ──────► RUNNING
    ▲              │                │
    │              │ stop           │ stop
    │              ▼                ▼
    └──────────── STOPPING ◄────────┘
           (move_stop 완료 또는
            watchdog 3s 후 IDLE)
```

- **STOPPING → IDLE 이후 `resume` 은 불가** (`Task not paused` 응답).
오직 `start`만 가능 → 사용자 요구사항 충족.

---

## 6.6 Gotcha: `launch_ros.Node(name=...)` 이 프로세스 내 모든 노드 이름을 덮는다

두 노드 구조를 성공적으로 구현한 뒤 다음 경고가 찍혔다:

```
[rcl.logging_rosout]: Publisher already registered for provided node name.
If this is due to multiple nodes with the same name then all logs for that
logger name will go out over the existing publisher.
```

그리고 로그 prefix가 둘 다 `[dsr01.motion_executor]` 로 나왔다 — 분명히
`rclpy.create_node("motion_executor_dsr", ...)` 라고 적었는데도 `_dsr` 접미사가
사라진 채.

### 원인

launch 파일:

```python
motion_executor = Node(
    package="cobot1",
    executable="motion_executor",
    name="motion_executor",         # ← 이것
    namespace=ns,
    ...
)
```

`launch_ros.actions.Node(name=...)` 는 내부적으로
`--ros-args -r __node:=motion_executor` CLI 인자를 프로세스에 전달한다. 이 인자는
rclpy가 `rclpy.init(args=args)` 시점에 **global arguments** 로 저장해 두고,
이후 **모든 `Node` 생성**에서 기본으로 적용해 노드 이름을 덮어쓴다.

결과: `rclpy.create_node("motion_executor_dsr", ...)` 라 해도 실제 이름은
`motion_executor` 로 remap → 두 노드의 이름 충돌 → `rcl_logging_rosout`이
"이미 해당 이름으로 publisher가 등록됨" 경고. `/rosout` 토픽으로는 둘 중 하나의
로그만 나가고, 다른 노드 로그는 stdout에만 찍힌다.

### 해결

보조 노드(`dsr_node`) 는 `use_global_arguments=False` 로 생성해 launch remap을
차단한다:

```python
dsr_node = rclpy.create_node(
    "motion_executor_dsr", namespace=ROBOT_ID,
    use_global_arguments=False,     # launch의 __node:= remap을 무시
)
```

- control_node: `Node(name="motion_executor")` 의 remap 수용 (대외적으로 고정된
노드 이름을 launch에서 선언할 수 있어야 하므로).
- dsr_node: 내부 구현 세부사항이므로 launch가 이름을 바꿔서는 안 됨.
`use_global_arguments=False` 로 remap 체인에서 제외.

### 교훈

- launch 파일의 `Node(name=...)` 는 **프로세스 내 모든 rclpy 노드에 영향**을
준다. 한 프로세스에 여러 노드를 넣는 구조에서는 반드시 의식할 것.
- 같은 이유로 **parameter overrides, remap rules, log level** 등도 global
arguments 를 경유해 모든 노드에 적용된다. 보조 노드에서 격리가 필요하면
`use_global_arguments=False` 를 사용한다.
- 유일한 단서가 "로그 prefix가 둘 다 같은 노드 이름으로 찍힌다" 였다. 로거
이름은 노드 FQN을 그대로 따라가므로, **"내 코드에 적은 이름대로 안 나온다면
remap이 개입한 것"** 을 의심.

---

## 7. 구현 체크리스트

신규 `motion_executor.py` 작성 시 반드시 확인:

1. [ ] `DR_init.__dsr__node` 에 할당하는 노드는 **별도 노드**인가?
2. [ ] 그 노드를 **어떤 executor에도 `add_node` 하지 않는가**?
3. [ ] `rclpy.spin(dsr_node)` / `executor.add_node(dsr_node)` 가 코드 어디에도
  없는가?
4. [ ] control node 만 `MultiThreadedExecutor` 에 add 되어 있는가?
5. [ ] DSR_ROBOT2 함수 호출은 **task thread** 에서만 일어나는가?
6. [ ] Task thread는 **동시에 한 개만** 존재하는가? (DSR_ROBOT2가 쓰는 글로벌
  SingleThreadedExecutor는 재진입 금지)
7. [ ] `MovePause/Resume/Stop/ReadDataRt` 는 control node의 클라이언트로
  `call_async` 하며, 응답은 콜백으로 처리하는가? (`spin_until_future_complete`
      를 control node에서 부르면 또 동일 문제 재현)
8. [ ] shared state는 `threading.Lock` 으로 보호되고, task thread와 control
  node의 콜백 양쪽에서만 접근하는가?
9. [ ] 보조 노드(dsr_node)는 `use_global_arguments=False` 로 생성되어 launch의
  `Node(name=...)` remap 영향을 받지 않는가?

---

## 8. 교훈

1. **rclpy의 Node 소유 executor는 1개**라는 불변 조건을 절대 깨지 말 것.
  `add_node` 소스 5줄만 읽어도 알 수 있는 사실인데, MultiThread로 문제를
   우회하려다 놓친다.
2. 외부 Python wrapper(여기선 DSR_ROBOT2)가 **동기식 `spin_until_future_complete`
  를 쓴다면** 그 라이브러리의 `g_node` 는 **내 executor에 넣으면 안 된다**.
   라이브러리를 직접 수정하지 않는 한 공존 불가.
3. 문제를 "내 코드의 race condition"으로 가정하고 lock / generation / watchdog
  을 덧붙이는 방향은 시간 낭비. wait-set은 rclpy 내부 상태라 Python lock으로
   보호할 수 없다.
4. ROS2 설계 습관: **외부 I/F 노드**와 **드라이버 I/F 노드**는 쓰레드 모델이
  다르면 무조건 분리하라. 같은 프로세스에 두 노드 두는 비용은 거의 없다.
5. 의심될 때는 **벤더 예제 코드부터** 읽어라. `single_robot_simple.py` 가
  `rclpy.spin()` 을 호출하지 않는다는 사실 하나로 설계 전제가 다 드러난다.

---

## 10. `ReadDataRt` 폴링 → 개별 monitoring 서비스 4종 병렬 폴링으로 마이그레이션

### 10.1 기존 구조의 실패

최초 구현은 `control_node` 에서 `/{ns}/realtime/read_data_rt` (ReadDataRt
서비스) 를 10Hz `call_async` 로 폴링하고 결과를 그대로
`motion_executor/robot_state` (`dsr_msgs2/RobotStateRt`) 토픽으로 재방송했다.
이 구조는 두 가지 이유로 실전에서 안 됐다:

1. **Virtual 모드에서 0 만 반환**. `dsr_hardware2/src/dsr_hw_interface2.cpp`
  는 `mode == "real"` 일 때만 `Drfl.connect_rt_control()` +
   `Drfl.start_rt_control()` 을 호출한다 (가상 컨트롤러는 RT 스트림 미지원).
   RT 스트림이 시작되지 않은 상태에서 `dsr_controller2` 의 `read_data_rt_cb`
   는 nullptr 체크 없이 `Drfl->read_data_rt()` 를 그대로 응답에 복사하므로
   **내부 zeroed 캐시가 그대로** 나간다.
2. **RT 모드에서도 한 서비스에 전 필드 몰빵 → 지연/정지 전파**.
  `RobotStateRt` 는 6-float 배열 20여 개 + 3개 6×6 행렬을 담고 있어
   직렬화 비용이 크다. `_rt_inflight` 래치로 한 번에 한 호출만 보내는 구조
   인데, 컨트롤러가 잠깐 바빠도 응답 지연이 쌓여 **모든 필드가 동시에
   멎는다**. 10Hz 틱 여러 번 연속 skip 되는 현상.

### 10.2 교체 설계

`read_data_rt` 의존을 버리고, DRCF `OnMonitoringDataExCB` 가 채우는 캐시 위에
얹힌 **개별 서비스 4종** 을 병렬 폴링한다. 이 캐시는 virtual/real 모두에서
갱신된다.


| 필드                              | 서비스                                     | srv 타입                     |
| ------------------------------- | --------------------------------------- | -------------------------- |
| `robot_state` (enum)            | `/{ns}/system/get_robot_state`          | `GetRobotState`            |
| `posj` [deg, 6]                 | `/{ns}/aux_control/get_current_posj`    | `GetCurrentPosj`           |
| `tool_force` [N·Nm, 6, BASE]    | `/{ns}/aux_control/get_tool_force`      | `GetToolForce` (req.ref=0) |
| `external_joint_torque` [Nm, 6] | `/{ns}/aux_control/get_external_torque` | `GetExternalTorque`        |


각 서비스는 **독립된 `inflight` 래치** 를 가진다. 한 필드가 느려도 다른
필드는 계속 업데이트된다. 4개 모두 `ReentrantCallbackGroup` 에 묶여
`MultiThreadedExecutor` 의 워커 풀에서 응답이 병렬 처리된다.

### 10.3 토픽 계약 변경

- **삭제**: `motion_executor/robot_state` (`dsr_msgs2/RobotStateRt`)
- **추가**: `motion_executor/robot_status` (`cobot_interfaces/RobotStatus`)
  - 측정값: `robot_state`, `posj[6]`, `tool_force[6]`, `external_joint_torque[6]`
  - `*_valid` 플래그: 해당 필드가 한 번이라도 정상 응답으로 갱신됐는지
  - `last_updated_ns`: 마지막 성공 응답 시각 (폴링이 멈췄는지 구분용)

소비자 3곳을 같이 갱신: `task_controller_node.py`, `task_cli.py status`,
(향후) `ui_bridge.py`.

### 10.4 유지해야 하는 제약

이번 변경도 §6 의 two-node 불변식을 그대로 지킨다:

- 상태 폴링 4개 클라이언트는 **별도 프로세스** `robot_status_publisher` 에만 존재.
motion_executor 쪽에는 존재하지 않는다.
- `DSR_ROBOT2` 파이썬 래퍼의 `get_current_posj()` / `get_tool_force()` 등을
직접 부르면 안 된다. 이 래퍼들은 여전히 `rclpy.spin_until_future_complete(g_node, ...)`
를 쓰기 때문. srv 타입만 가져다 **ROS 서비스를 직접** `call_async` 한다.
- task thread / dsr_node / DSR_ROBOT2 사용 경로는 그대로.

### 10.5 상태 폴링을 별도 프로세스로 분리한 이유 (실증)

시도 순서:

1. motion_executor 의 control_node 에 4개 클라이언트 추가 + `ReentrantCallbackGroup`
  - `MultiThreadedExecutor` → **전 필드 ok=0 타임아웃**. idle 상태에서도 실패.
2. `add_done_callback` 안 먹는 줄 알고 `future.done()` 수동 drain 추가 → 동일.
3. race 때문인 줄 알고 `RLock` + atomic dispatch 로 재작성 → 동일.
4. **진단**: 별도 셸에서 `ros2 service call /dsr01/system/get_robot_state ...`
  직접 호출은 즉답이 정상으로 돌아옴. 서버는 완전히 정상.

결론: motion_executor 프로세스 내부 상태(MT executor + DSR_ROBOT2 wrapper 의
`rclpy.spin_until_future_complete(dsr_node, ...)` 임시 Executor 점유 + task
thread)와 상태 폴링 client 의 wait-set 이 같은 rmw context 에서 간섭. 경로는
특정하지 못했지만 분리 후 문제 재현 안 됨.

해결: **상태 폴링만 별도 프로세스**(`cobot1/robot_status_publisher.py`) 로 분리.
DSR_ROBOT2 import 없음, dsr_node 없음, task thread 없음 → 순수 MT executor.
퍼블리시 토픽 경로는 기존과 동일 (`/{ns}/motion_executor/robot_status`)
이므로 구독자 쪽은 수정 없음.

### 10.6 외력 safety 동작의 훅 지점

두 가지 옵션:

(a) motion_executor (control_node) 가 `motion_executor/robot_status` 를 추가
  **구독**해 `tool_force` / `external_joint_torque` 임계값 초과 시 직접
  `MoveStop` 클라이언트를 호출.

(b) `task_controller` 가 구독(이미 하고 있음)해 정책 판정 후 `task/stop`
  (stop_type=1) 을 호출. 레이어링이 더 깔끔.

권장은 (b). motion_executor 는 "명령 수신·태스크 실행" 만 담당하고, 상태
판단·정책은 controller 가 가짐 → 역할 경계가 문서와 일치.

---

## 11. 참고 파일

- `/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py`
(`spin_until_future_complete`, `get_global_executor`)
- `/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py`
(`Executor.add_node`, `spin_until_future_complete`)
- `src/doosan-robot2/dsr_common2/imp/DSR_ROBOT2.py`
(모든 DSR API의 `rclpy.spin_until_future_complete(g_node, future)` 패턴)
- `src/doosan-robot2/dsr_common2/imp/DR_init.py` (`__dsr__node`, `__dsr__id`,
`__dsr__model`)
- `src/doosan-robot2/dsr_example2/dsr_example/dsr_example/simple/single_robot_simple.py`
(벤더 공식 사용 패턴 예시)
- `src/doosan-robot2/dsr_controller2/src/dsr_controller2.cpp`
(`read_data_rt_cb`, `get_current_posj_cb`, `get_tool_force_cb`,
`get_external_torque_cb`, `get_robot_state_cb` 의 실제 서빙 지점)
- `src/doosan-robot2/dsr_hardware2/src/dsr_hw_interface2.cpp`
(`mode != "virtual"` 에서만 `connect_rt_control` + `start_rt_control`)
- `src/cobot1/cobot1/motion_executor.py` (본 프로젝트의 두-노드 구현)
- `src/cobot1/cobot1/robot_status_publisher.py` (§10 상태 폴링 전용 프로세스)
- `src/cobot_interfaces/msg/RobotStatus.msg` (현재 토픽 스키마)

