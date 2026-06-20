import rclpy
import DR_init
import time

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도
VELOCITY = 60
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
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
    print(f"ROBOT_MODE (0:수동, 1:자동): {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)


def perform_task():
    from DSR_ROBOT2 import posx, movej, movel, wait, set_digital_output 

    def gripper(action):
        # set_digital_output(1, 0)
        # set_digital_output(2, 0)
        # set_digital_output(3, 0)
        # wait(0.1) 

        if action == "GRIP_BASIC":
            print("  ➡️ [그리퍼] 조금 닫기")    # 35mm
            set_digital_output(1, 1)
            set_digital_output(2, 0)
            set_digital_output(3, 0)
        elif action == "RELEASE":
            print("  ⬅️ [그리퍼] 완전히 열기")   #75mm
            set_digital_output(1, 0)
            set_digital_output(2, 1)
            set_digital_output(3, 0)
        elif action == "GRIP_TIGHT":
            print("  ⏩ [그리퍼] 꽉 닫기")   #0mm
            set_digital_output(1, 1)
            set_digital_output(2, 1)
            set_digital_output(3, 0)

        wait(1.5) 

    # 위치 좌표 설정
    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([273.96, -248.55, 314.37, 128.50, 179.98, 128.52]) # 집게 위로 이동
    pos2 = posx([275.48, -252.13, 148.32, 136.44, 179.77, 136.72]) # 집게 바로 아래
    # 1번 돈까스 좌표
    pos3 = posx([467.28, 159.75, 329.71, 22.73, 176.34, 23.24])     # 1번 돈까스 위
    pos4 = posx([477.64, 163.29, 200.25, 23.11, 176.60, 23.72]) # 1번 돈까스 아래

    pos5 = posx([453.93, 11.44, 338.32, 112.2, 177.64, 112.11]) # 식판으로 가는 중간 (원점)
    pos6 = posx([652.65, 16.23, 317.19, 4.07, 157.35, 3.72]) # 식판의 공중 위쪽
    pos7 = posx([669.94, 16.05, 290.75, 3.88, 156.47, 3.66]) # 식판 바로 위 이동 (여기서 돈까스 놓기)
    # 2번 돈까스 좌표
    pos8 = posx([520.80, 161.72, 327.75, 104.54, 178.06, 104.91]) # 2번 돈까스 위로 이동
    pos9 = posx([523.32, 169.27, 198.30, 106.93, 177.95, 107.49]) # 2번 돈까스 바로 아래
    pos6_1 = posx([646.89, -28.32, 307.62, 0.03, 147.99, 3.56]) # 식판의 공중 위쪽
    pos7_1 = posx([638.04, -32.76, 252.58, 178.53, -152.99, -178.09]) # 식판 바로 위 이동 (여기서 돈까스 놓기)
    # 샐러드 좌표
    pos10 = posx([497.34, -169.84, 296.22, 155.26, -136.53, 178.28]) # 샐러드 공중위로 이동
    pos11 = posx([530.00, -230.28, 137.70, 144.66, -155.31, 168.53]) # 샐러드 아래   
    pos6_2 = posx([724.82, 17.10, 283.40, 0.87, 150.10, 11.78])  # 식판 공중위
    pos7_2 = posx([748.22, 17.51, 236.21, 0.86, 151.72, 11.77]) # 식판 아래(샐러드 놓기)
    


    print("\n🚀 작업을 시작합니다...")
# 집게 가져오기
    print("1. JReady로 이동")
    movej(JReady, vel=VELOCITY, acc=ACC)
    gripper("RELEASE") 

    print("2. pos1으로 이동 (집게 위)")
    movel(pos1, vel=VELOCITY, acc=ACC)

    print("3. pos2로 이동 (집게 접근)")
    movel(pos2, vel=VELOCITY, acc=ACC)

    print("4. 그리퍼 살짝 닫기 (집게 파지)")
    gripper("GRIP_BASIC")

    print("5. 다시 pos1으로 이동 (집게 들어올리기)")
    movel(pos1, vel=VELOCITY, acc=ACC)

    print("원점")
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 샐러드 집기
    movel(pos10, vel=VELOCITY, acc=ACC)
    movel(pos11, vel=VELOCITY, acc=ACC)
    gripper("GRIP_TIGHT")  # 샐러드 집고
    movel(pos11, vel=VELOCITY, acc=ACC)
    movel(pos10, vel=VELOCITY, acc=ACC)
    print("샐러드 들어올리기 완료, 식판으로 이동 시작...")
    movel(pos6_2, vel=VELOCITY, acc=ACC)
    movel(pos7_2, vel=VELOCITY, acc=ACC)
    gripper("GRIP_BASIC")  # 샐러드 놓고
    movel(pos6_2, vel=VELOCITY, acc=ACC)
    movel(pos10, vel=VELOCITY, acc=ACC)
    print("원점")
    movej(JReady, vel=VELOCITY, acc=ACC)
##############################################################
# 돈까스 1 픽업
    print("6. pos3으로 이동 (돈까스 위)")
    movel(pos3, vel=VELOCITY, acc=ACC)

    print("7. pos4로 이동 (돈까스 접근)")
    movel(pos4, vel=VELOCITY, acc=ACC)

    print("8. 그리퍼 꽉 닫기 (돈까스 집기)")
    gripper("GRIP_TIGHT")

    print("9. 다시 pos3으로 이동 (돈까스 들어올리기)")
    movel(pos3, vel=VELOCITY, acc=ACC)
# 돈까스 1 식판에 이동 (pos6, pos7)
    print("10. pos5로 이동 (식판위로 이동)")
    movel(pos5, vel=VELOCITY, acc=ACC)

    print("10. pos6로 이동 (식판 한참위로 이동)")
    movel(pos6, vel=VELOCITY, acc=ACC)

    print("10. pos7로 이동 (식판 바로위으로 이동)")
    movel(pos7, vel=VELOCITY, acc=ACC)

    print("11. 그리퍼 살짝 닫기 (돈까스 놓기)")
    gripper("GRIP_BASIC")

    print("10. pos5로 이동 (식판위로 이동)")
    movel(pos6, vel=VELOCITY, acc=ACC)

    print("10. pos5로 이동 (식판위로 이동)")
    movel(pos5, vel=VELOCITY, acc=ACC)
######################################################################
# 돈까스 2 픽업
    print("10. pos5로 이동 (식판위로 이동)")
    movel(pos8, vel=VELOCITY, acc=ACC)

    print("10. pos5로 이동 (식판위로 이동)")
    movel(pos9, vel=VELOCITY, acc=ACC)

    print("8. 그리퍼 꽉 닫기 (돈까스 집기)")
    gripper("GRIP_TIGHT")

    print("10. pos5로 이동 (식판위로 이동)")
    movel(pos8, vel=VELOCITY, acc=ACC)

    print("10. pos5로 이동 (식판위로 이동)")
    movel(pos5, vel=VELOCITY, acc=ACC)
# 돈까스 2 식판에 이동 (pos6, pos7)
    print("10. pos5로 이동 (식판위로 이동)")
    movel(pos6_1, vel=VELOCITY, acc=ACC)

    print("10. pos5로 이동 (식판위로 이동)")
    movel(pos7_1, vel=VELOCITY, acc=ACC)

    print("11. 그리퍼 살짝 닫기 (돈까스 놓기)")
    gripper("GRIP_BASIC")

    print("10. pos6로 이동 (식판위로 이동)")
    movel(pos6_1, vel=VELOCITY, acc=ACC)

    print("10. pos5로 이동 (식판위로 이동)")
    movel(pos5, vel=VELOCITY, acc=ACC)
##################################################################################




# --- 여기서부터 집게 반납하는 부분입니다 ---
    print("13. pos1으로 이동 (집게 반납 위치 위)")
    movel(pos1, vel=VELOCITY, acc=ACC)

    print("14. pos2로 이동 (집게 내려놓기)")
    movel(pos2, vel=VELOCITY, acc=ACC)

    print("15. 그리퍼 완전히 열기 (집게 반납)")
    gripper("RELEASE")

    print("16. 다시 pos1으로 이동 (집게 위로 복귀)")
    movel(pos1, vel=VELOCITY, acc=ACC)

    print("17. JReady로 이동 (최종 원점 복귀)")
    movej(JReady, vel=VELOCITY, acc=ACC)
    # # -----------------------------------------
    
    print("✅ 모든 작업이 완료되었습니다!")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("tong", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task() 

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()