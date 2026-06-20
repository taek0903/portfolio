import rclpy
import DR_init
import time

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY = 40
ACC = 60

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("full_scoop_and_pour", namespace=ROBOT_ID)

    # DR_init 설정
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        set_robot_mode, ROBOT_MODE_AUTONOMOUS, 
        movej, movejx, movel, wait, get_current_posj, posx, posj,
        set_digital_output, DR_MV_MOD_REL, DR_BASE
    )

    def gripper(action):
        """그리퍼 제어 함수"""
        set_digital_output(1, 0)
        set_digital_output(2, 0)
        set_digital_output(3, 0)
        set_digital_output(4, 0)
        wait(0.1) 
        if action == "GRIP":
            set_digital_output(1, 1) # 임의의 그리퍼 닫힘 핀 번호
        elif action == "RELEASE":
            set_digital_output(2, 1) # 임의의 그리퍼 열림 핀 번호
        wait(1.0)

    try:
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        wait(1.0)

        # -----------------------------------------------------------
        # 📍 [임의 좌표 설정] 나중에 실제 로봇 위치에 맞게 수정하세요.
        # 기본 하방을 보는 각도: Rx=0, Ry=180, Rz=0
        # -----------------------------------------------------------
        JReady = [0, 0, 90, 0, 90, 0] # 대기 위치
        
        # [국자 위치]
        p1 = posj(15.79, 22.54, 119.70, -75.26, 94.73, 53.54)
        p2 = posj(8.76, 22.20, 121.17, -80.44, 90.58, 53.55)
        p3 = posj(8.26, 11.60, 114.92, -81.49, 93.04, 35.97)
        p4 = posj(-9.40, 20.14, 88.76, 65.89, 25.21, 13.27)
        p5 = posj(-7.99, 20.39, 87.19, 64.34, 21.86, 190.44)

        # -----------------------------------------------------------
        # 🚀 [시퀀스 시작]
        # -----------------------------------------------------------
        print("\n🚀 통합 소스 배식 시퀀스를 시작합니다.")
        movej(JReady, vel=VELOCITY, acc=ACC)

        # 1. 국자 장착
        print("1. pos1 이동 (국자 위)")
        movej(p1, vel=VELOCITY, acc=ACC)
        print("1. pos1 이동 (국자 위)")
        movej(p2, vel=VELOCITY, acc=ACC)
        print("1. pos1 이동 (국자 위)")
        movej(p3, vel=VELOCITY, acc=ACC)
        print("1. pos1 이동 (국자 위)")
        movej(p4, vel=VELOCITY, acc=ACC)
        print("1. pos1 이동 (국자 위)")
        movej(p5, vel=VELOCITY, acc=ACC)
        

    except KeyboardInterrupt:
        print("\n중단되었습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()