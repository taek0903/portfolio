import rclpy
import DR_init
import time

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 40
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_robot_mode, set_robot_mode

    # Tool과 TCP 설정시 매뉴얼 모드로 변경해서 진행
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)  # 설정 안정화를 위해 잠시 대기
    
    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}") 
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)


def perform_task():
    """로봇이 수행할 작업"""
    print("Performing task...")
    
    # 이동 및 힘 제어에 필요한 모든 함수 임포트
    from DSR_ROBOT2 import (
        posx, movej, movel, set_ref_coord, wait, 
        get_current_posj,
        release_compliance_ctrl, release_force,
        check_force_condition, task_compliance_ctrl,
        set_desired_force, get_tool_force,
        DR_FC_MOD_REL, DR_AXIS_Z, DR_TOOL
    )

    # 초기 위치 설정
    JReady = [-0.068, -43.036, 131.449, -0.220, -0.221, 0.312]

    # 1. 처음 위치로 이동
    print("1. 처음 위치(JReady)로 이동...")
    movej(JReady, vel=VELOCITY, acc=ACC)
    wait(0.5)

    # 2. 1번 조인트(J1) 이동
    print("2. 1번 조인트(J1) -20도 이동 중...")
    cur_j = get_current_posj()
    
    if isinstance(cur_j, tuple):
        cur_j = cur_j[0]
        
    cur_j = [float(cur_j[i]) for i in range(6)]
    
    # 1번 조인트(인덱스 0)에 20도 빼기
    cur_j[0] -= 20.0  
    
    movej(cur_j, vel=VELOCITY, acc=ACC)
    wait(1.0) # 이동 후 안정화 대기

    # 3. 순응 제어 시작 (아래로 하강하며 바닥 감지)
    print("3. 순응 제어 시작 (바닥 감지)...")
    set_ref_coord(1) # Tool 좌표계 설정
    task_compliance_ctrl(stx=[1000, 1000, 200, 200, 200, 200])
    wait(0.5) 
    
    # Z축 방향으로 15N의 힘을 주며 하강
    set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    # 힘 조건 확인 (바닥 감지 루프)
    while True:
        ret = check_force_condition(DR_AXIS_Z, min=0, max=12)
        force_list = get_tool_force(DR_TOOL) 
        fz = abs(force_list[2])   
        
        print(f"Z축 힘 대기 중... 현재 힘: {fz:.2f} N")
        
        if ret == -1 or ret == 1: 
            print("바닥 감지 완료.")
            break
        wait(0.2)

    # 4. 힘 제어 해제 및 종료
    print("4. 힘 제어 해제 및 대기...")
    release_force()
    release_compliance_ctrl()
    wait(1.0) 
    
    print("✅ 1번 조인트 이동 후 순응 제어 하강 작업 완료!")
    

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_and_force_node", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        
        # 💡 작업 수행 루프(while True)를 제거하여 딱 한 번만 실행되도록 변경
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()