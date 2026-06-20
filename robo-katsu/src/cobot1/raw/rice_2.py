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
    node = rclpy.create_node("test_return_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        
        # 오직 세팅 및 반납 동작만 단독 실행
        test_return_tool()
        
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()