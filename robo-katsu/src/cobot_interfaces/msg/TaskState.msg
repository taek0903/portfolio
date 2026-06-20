# Task state constants
uint8 IDLE = 0
uint8 RUNNING = 1
uint8 STOPPING = 2
uint8 ERROR = 3
uint8 PAUSED = 4

# Current state
uint8 state

# Task information
string task_name
string task_id

# Progress information
float32 progress              # 0.0 ~ 1.0
uint32 current_step
uint32 total_steps
string current_step_name

# Robot position (optional, for monitoring)
float64[6] current_posj       # Joint position [deg]
float64[6] current_posx       # Task position [mm, deg]

# Status message
string message

# Composite task info (auto_serving 처럼 여러 모듈을 연속 실행할 때 현재 서브태스크)
# - 단일 모듈 task 는 module_name == task_name, module_index=0, module_total=1
# - auto_serving 중에는 module_name ∈ {"rice","tong","sauce"} 로 변함
string current_module_name
string current_module_label
uint32 module_index
uint32 module_total

# Timestamp
builtin_interfaces/Time stamp
