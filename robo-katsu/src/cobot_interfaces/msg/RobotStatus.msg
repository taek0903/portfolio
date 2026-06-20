# motion_executor가 DSR2 개별 서비스(GetRobotState / GetCurrentPosj /
# GetToolForce / GetExternalTorque)를 10Hz로 병렬 폴링해 합쳐 내보내는
# 프로젝트 전용 상태 스냅샷.
#
# DSR의 read_data_rt는 real 모드에서만 스트림이 시작되고 virtual에서는 0만
# 반환해 쓸 수 없다. 이 메시지는 두 모드 모두에서 동작하는 monitoring 캐시
# 기반 서비스들만 사용한다. 자세한 배경은
# docs/rclpy-executor-dsr2-troubleshooting.md 를 참고.

# robot_state enum (GetRobotState.srv 와 동일 의미)
uint8 ROBOT_STATE_INITIALIZING   = 0
uint8 ROBOT_STATE_STANDBY        = 1
uint8 ROBOT_STATE_MOVING         = 2
uint8 ROBOT_STATE_SAFE_OFF       = 3
uint8 ROBOT_STATE_TEACHING       = 4
uint8 ROBOT_STATE_SAFE_STOP      = 5
uint8 ROBOT_STATE_EMERGENCY_STOP = 6
uint8 ROBOT_STATE_HOMMING        = 7
uint8 ROBOT_STATE_RECOVERY       = 8
uint8 ROBOT_STATE_SAFE_STOP2     = 9
uint8 ROBOT_STATE_SAFE_OFF2      = 10
uint8 ROBOT_STATE_NOT_READY      = 15
uint8 ROBOT_STATE_UNKNOWN        = 255

# --- 측정값 ---
uint8      robot_state                     # 위 ROBOT_STATE_* 중 하나
float64[6] posj                            # 현재 관절각 [deg]
float64[6] tool_force                      # TCP 외력/모멘트 (BASE 기준) [N, Nm]
float64[6] external_joint_torque           # 각 관절 외력 토크 [Nm]

# --- 갱신 여부 플래그 ---
# 해당 필드가 **한 번이라도 정상 응답으로 갱신**된 적이 있는지.
# 서비스 준비 전/통신 실패 시 false 로 남아 소비자가 구분할 수 있게 한다.
bool robot_state_valid
bool posj_valid
bool tool_force_valid
bool external_joint_torque_valid

# --- 타임스탬프 ---
# stamp: 이 메시지가 publish된 시각.
# last_updated_ns: 마지막 *성공* 응답이 돌아왔을 때 control_node 의 clock.
#   (모니터링용 — 폴링이 멈췄는지 판단)
builtin_interfaces/Time stamp
int64 last_updated_ns
