import time
import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도
VELOCITY = 100
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


def gripper(action):
    """그리퍼 제어 공통 함수"""
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(1, 0)
    set_digital_output(2, 0)
    set_digital_output(3, 0)
    set_digital_output(4, 0)
    wait(0.1) 

    if action == "GRIP_BASIC":
        set_digital_output(1, 1)
        set_digital_output(2, 1)
    elif action == "RELEASE":
        set_digital_output(2, 1)
    wait(1.5) 


def pick_up_tool():
    """본 작업 전, 데스크에서 주걱(도구)을 집어오는 사전 작업"""
    from DSR_ROBOT2 import movej, movel, posx

    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([288.70, 219.31, 311.70, 113.48, 179.96, 113.0])
    pos2 = posx([281.07, 218.15, 168.10, 40.36, 179.76, 40.20])

    print("\n[사전 작업] 주걱 장착을 시작합니다.")
    movej(JReady, vel=VELOCITY, acc=ACC)
    gripper("RELEASE") 

    movel(pos1, vel=VELOCITY, acc=ACC)
    movel(pos2, vel=VELOCITY, acc=ACC)
    gripper("GRIP_BASIC")
    movel(pos1, vel=VELOCITY, acc=ACC)
    print("✅ 주걱 장착 완료!\n")


def perform_task():
    """본 작업 (밥 푸기 및 힘 제어)"""
    from DSR_ROBOT2 import (
        release_compliance_ctrl, release_force,
        check_force_condition, task_compliance_ctrl,
        set_desired_force, set_ref_coord,
        movej, movel, movesx, wait, posx,
        DR_FC_MOD_REL, DR_MV_MOD_REL, DR_AXIS_Z, DR_TOOL,
        get_tool_force, get_current_posj
    )

    JReady = [0, -10, 70, 0, 90, 0]
    print("1. JReady 위치 확인...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    print("2. Z축 방향으로 250mm 하강 중...")
    movej([11.66, 23.74, 60.51, 4.64, 75.15, 8.15], vel=VELOCITY, acc=ACC)
    wait(0.5) 

    print("3. 힘 제어 시작...")
    set_ref_coord(1) 
    task_compliance_ctrl(stx=[1000, 1000, 200, 200, 200, 200])
    wait(0.5) 
    set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    while True:
        ret = check_force_condition(DR_AXIS_Z, min=0, max=10)
        force_list = get_tool_force(DR_TOOL) 
        fz = abs(force_list[2])   
        if ret == -1 or ret == 1: 
            print("바닥 감지 완료.")
            break
        wait(0.2)

    release_force()
    release_compliance_ctrl()
    wait(1.0) 

    print("🚀 5. MoveSX: 반원 궤적으로 퍼올리기 시작!")
    p1  = posx([-15, 0, 15, 0, -10, 0])
    p1_1 = posx([-10, 0, 10, 0, -10, 0])
    p1_2 = posx([-10, 0, 13, 0, -12, 0])
    movesx([p1, p1_1, p1_2], vel=15, acc=15, mod=DR_MV_MOD_REL, ref=DR_TOOL)

    movel([0, 0, -100, 0, 0, 0], vel=15, acc=ACC, mod=DR_MV_MOD_REL, ref=DR_TOOL)

    cur = get_current_posj()
    if isinstance(cur, tuple): cur = cur[0]
    target = [cur[0], cur[1] - 30, cur[2], cur[3], cur[4], cur[5]]
    movej(target, vel=15, acc=30)
    wait(1.0)

    plate = [-13.19, 22.44, 128.57, -11.92, -59.73, 18.29]
    movej(plate, vel=VELOCITY, acc=ACC)

    cur_j = get_current_posj()
    if isinstance(cur_j, tuple): cur_j = cur_j[0]
    next_j = [float(val) for val in cur_j]
    next_j[5] -= 90.0
    movej(next_j, vel=VELOCITY, acc=ACC)
    wait(1.0)
    print("✅ 밥 푸기 작업 완료!")

    plate = [-11.5, 16.93, 127.69, -11.28, -53.39, -71.13]
    movej(plate, vel=VELOCITY, acc=ACC)

def test_return_tool():
    """도구 반납 단독 테스트 함수 (배식 완료 자세부터 시작)"""
    from DSR_ROBOT2 import movej, movel, movejx, posx, wait, set_ref_coord
    set_ref_coord(0)

    # ---------------------------------------------------------
    # [1] 테스트 환경 초기 세팅: 식판 위치로 이동
    # ---------------------------------------------------------
    print("\n[테스트 준비] 식판 배식 완료 직후 상태로 로봇을 먼저 세팅합니다.")
    plate = [-11.5, 16.93, 127.69, -11.28, -53.39, -71.13]
    movej(plate, vel=VELOCITY, acc=ACC)
    wait(0.5)
    print("✅ 테스트 준비 완료: 배식 직후 위치에 도달했습니다.\n")


    # ---------------------------------------------------------
    # [2] 본 반납 작업 시작
    # ---------------------------------------------------------
    JReady = [0, -10, 70, 0, 90, 0]
    JFinal = [0, 0, 90, 0, 90, 0]

    pos1 = posx([304.70, 219.31, 311.70, 113.48, 179.96, 113.0])
    pos2 = posx([304.15, 219.77, 190.26, 22.35, -179.92, 21.90])
    pos3 = posx([290.11, 212.02, 171.43, 28.01, -179.88, 27.29])
    print("[사후 작업] 도구 반납 및 복귀를 시작합니다.")

    # 1. 지정된 좌표로 이동


    # 2. 원점(JReady) 경유
    print("2. JReady 경유")
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 3. 반납 위치(pos1, pos2)로 이동 및 그리퍼 해제
    print("3. 데스크 진입 및 도구 내려놓기")
    movel(pos1, vel=VELOCITY, acc=ACC) 
    movel(pos2, vel=VELOCITY, acc=ACC)  
    gripper("RELEASE")
    movel(pos3, vel=VELOCITY, acc=ACC)
    # 4. 상승 후 최종 위치 복귀
    print("4. 최종 복귀")
    movel(pos1, vel=VELOCITY, acc=ACC)
    movej(JFinal, vel=VELOCITY, acc=ACC)
    print("✅ 도구 반납 및 최종 원점 복귀 테스트 완료!")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("scooping_force_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        pick_up_tool()   # 1. 도구 집기
        perform_task()   # 2. 메인 작업
        test_return_tool()    # 3. 도구 반납
        
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()


  