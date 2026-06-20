import rclpy
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

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
    from DSR_ROBOT2 import posx, movej, movel, wait, set_digital_output, move_periodic, DR_BASE, get_current_posx, get_digital_input, get_digital_output


    def gripper(action):
        set_digital_output(1, 0)
        set_digital_output(2, 0)
        set_digital_output(3, 0)
        wait(0.1)

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
        wait(3.0) 

    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([370.74, -259.40, 561.74, 110.66, 179.83, 111.44]) #소스통 위로 이동
    pos2 = posx([370.30, -261.49, 400.50, 122.48, 179.77, 123.35]) #소스통 집기 바로 전 위치
    pos3 = posx([552.5, 17.51, 490.76, 169.74, -156.47, 156.06]) #소스통 바로 위 
    

    movej(JReady, vel=VELOCITY, acc=ACC)
    gripper("GRIP_RELEASE")

    movel(pos1, vel=VELOCITY, acc=ACC)
    movel(pos2, vel=VELOCITY, acc=ACC)

    gripper("sauce")


    print("1. JReady로 이동")
    movel(pos1, vel=VELOCITY, acc=ACC)
    movel(pos3, vel=VELOCITY, acc=ACC)
    
    gripper("TIGHT")

    # 반복 동작 (2회 완료 후 다음 코드로 진행)
    move_periodic(amp=[20, 20, 0, 0, 0, 0], period=[3.2, 1.6, 1.6, 0, 0, 0], atime=3.1, repeat=2, ref=DR_BASE)

    # # 제자리에서 Z축만 pos1의 높이(260.62)로 수직 상승
    # cur_posx = get_current_posx()
    # if isinstance(cur_posx, tuple):
    #     cur_pos = cur_posx[0]
    # else:
    #     cur_pos = cur_posx
        
    # pos_z_up = [cur_pos[0], cur_pos[1], pos1[2], cur_pos[3], cur_pos[4], cur_pos[5]]
    gripper("GRIP_RELEASE1")
    gripper("GRIP_RELEASE2")
    gripper("sauce")
    # movel(pos_z_up, vel=VELOCITY, acc=ACC)
    
    # 복귀 궤적 이어서 수행
    movel(pos1, vel=VELOCITY, acc=ACC)
    movel(pos2, vel=VELOCITY, acc=ACC)
    gripper("GRIP_RELEASE")  # 집게 놓기
    movel(pos1, vel=VELOCITY, acc=ACC)
    movej(JReady, vel=VELOCITY, acc=ACC)


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