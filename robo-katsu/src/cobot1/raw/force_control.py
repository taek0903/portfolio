import time
import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도
VELOCITY = 20
ACC = 20

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_robot_mode, set_robot_mode

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2) 
    
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}") 
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print("#" * 50)


def perform_task():
    """로봇이 수행할 작업"""
    print("Performing force control task...")
    from DSR_ROBOT2 import (
        release_compliance_ctrl, release_force,
        check_force_condition,
        task_compliance_ctrl,
        set_desired_force,
        set_ref_coord,
        movej,
        movel,        # 👈 직선 이동을 위해 movel 추가
        movesx,
        wait,
        DR_FC_MOD_REL,
        DR_MV_MOD_REL, # 👈 상대 좌표 이동 모드
        DR_AXIS_Z,
        DR_TOOL,       # 👈 툴 좌표계 기준
        posx,
        get_tool_force
    )

    # 1. 초기 위치(JReady)로 이동
    JReady = [0, -10, 70, 0, 90, 0]
    print("1. JReady 위치로 이동...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 2. Z축으로 100mm 하강 (상대 좌표 이동)
    # [X, Y, Z, Rx, Ry, Rz] -> Z축 방향으로 100mm 이동
    print("2. Z축 방향으로 250mm 하강 중...")
    movel([0, 0, 250, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL, ref=DR_TOOL)
    wait(0.5) # 이동 후 안정화 대기

    # 3. 힘 제어 시작 (하강 후 바닥 감지)
    print("3. 힘 제어 시작...")
    set_ref_coord(1) 
    task_compliance_ctrl(stx=[1000, 1000, 200, 200, 200, 200])
    wait(0.5) 
    set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    # 힘 조건 확인
    while True:
        ret = check_force_condition(DR_AXIS_Z, min=0, max=12)
        force_list = get_tool_force(DR_TOOL) 
        fz = abs(force_list[2])   
        
        print(f"Z축 힘 대기 중... 현재 힘: {fz:.2f} N")
        
        if ret == -1 or ret == 1: 
            print("바닥 감지 완료.")
            break
        wait(0.2)

    # 4. 힘 제어 해제
    print("4. 힘 제어 해제 및 대기...")
    release_force()
    release_compliance_ctrl()
    wait(1.0) 

    # 5. MoveSX: 반원 궤적으로 퍼올리기
    print("🚀 5. MoveSX: 반원 궤적으로 퍼올리기 시작!")
    
    # 반지름 조절: 숫자를 줄이면 더 작은 반원이 됩니다.
    p1 = posx([-15, 0, 15, 0, -10, 0]) 
    p2 = posx([-60, 0, 60, 0, -50, 0])

    movesx([p1, p2], vel=15, acc=15, mod=DR_MV_MOD_REL, ref=DR_TOOL)
    
    wait(3.0)
    print("✅ 모든 작업 완료!")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("scooping_force_node", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task()
        
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()