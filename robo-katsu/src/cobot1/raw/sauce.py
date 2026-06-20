import rclpy
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 60
ACC = 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_robot_mode
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)


def perform_task():
    # 💡 amove_periodic 임포트 추가
    from DSR_ROBOT2 import posx, movej, movel, wait, set_digital_output, move_periodic, amove_periodic, DR_BASE, get_current_posx, get_digital_input, get_digital_output

    # 💡 wait_time 파라미터 추가 (기본값 3초)
    def gripper(action, wait_time=3.0):
        if action == "sauce":
            set_digital_output(1, 0)
            set_digital_output(2, 0)
            set_digital_output(3, 1)
        elif action == "TIGHT":
            # 001 (1=0, 2=0, 3=1) 적용
            set_digital_output(1, 1)
            set_digital_output(2, 1)
            set_digital_output(3, 1)
        elif action == "GRIP_RELEASE":
            set_digital_output(1, 0)
            set_digital_output(2, 1)
            set_digital_output(3, 0)
            
        wait(wait_time) # 지정된 시간만큼 대기

    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([371.54, -262.40, 307.33, 134.64, 179.61, 135.07]) #소스통 위로 이동
    pos2 = posx([371.35, -262.77, 173.15, 138.52, 179.50, 139.01]) #소스통 집기 바로 전 위치
    pos3 = posx([652.17, -19.67, 286.20, 8.13, 156.13, -22.47]) #소스통 바로 위 
    

    movej(JReady, vel=VELOCITY, acc=ACC)
    print("0. 원위치")

    gripper("GRIP_RELEASE")
    print("1. JReady로 이동")

    movel(pos1, vel=VELOCITY, acc=ACC)
    print("2. pos1로 이동")

    movel(pos2, vel=VELOCITY, acc=ACC)
    print("3. pos2로 이동")

    gripper("sauce")
    print("4. 소스통 잡기(68mm)")


    movel(pos1, vel=VELOCITY, acc=ACC)
    print("5. pos1로 이동")

    movel(pos3, vel=VELOCITY, acc=ACC)
    print("6. pos3로 이동")

    # ----------------------------------------------------
    # 💡 7 & 8 완벽한 동시 실행 구간
    # ----------------------------------------------------
    print("7 & 8. 소스 짜며 흔들기 (동시 시작)")
    
    # 1. 흔들기 시작 (amove_periodic을 사용하여 즉시 다음 코드로 넘어감)
    amove_periodic(amp=[20, 25, 0, 0, 0, 0], period=[1.6, 3.2, 1.6, 0, 0, 0], atime=3.1, repeat=2, ref=DR_BASE)
    
    # 2. 흔들기 시작과 동시에 대기시간 없이(0.0초) 즉각 그리퍼를 조임
    gripper("TIGHT", wait_time=0.0)
    
    # 3. 로봇이 흔들기를 끝낼 때까지(3.1초 * 2회 = 6.2초) 대기
    wait(10.2)
    # ----------------------------------------------------

    gripper("sauce")
    print("9. 소스통 잡기(68mm)")
    
    # 복귀 궤적 이어서 수행
    movel(pos1, vel=VELOCITY, acc=ACC)
    print("10. pos1로 이동")

    movel(pos2, vel=VELOCITY, acc=ACC)
    print("11. pos2로 이동")

    gripper("GRIP_RELEASE")  # 집게 놓기
    print("12. 소스통 잡기(68mm)")

    movel(pos1, vel=VELOCITY, acc=ACC)
    print("13. pos1로 이동")
    
    movej(JReady, vel=VELOCITY, acc=ACC)
    print("14. 원위치")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_periodic", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()