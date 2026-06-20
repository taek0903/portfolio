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
        movej, movejx, movel, wait, get_current_posj, posx,
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
        pos1 = posx([300, 200, 400, 0, 180, 0]) # 국자 상공
        pos2 = posx([300, 200, 200, 0, 180, 0]) # 국자 픽업/반납 위치 (수직 하강)
        
        # [소스통 위치]
        pos3 = posx([400, 0, 400, 0, 180, 0]) # 소스통 상공
        pos4 = posx([400, 0, 200, 0, 180, 0]) # 소스통 내부 (수직 하강)
        
        # 💡 [핵심] 뒤로 10도 기울인 상태(Ry=170)를 유지하며 이동할 좌표들
        pos3_scooped = posx([400, 0, 400, 0, 170, 0]) # 소스를 뜨고 올라온 소스통 상공
        pos5_scooped = posx([300, -200, 400, 0, 170, 0]) # 식판 가는 중간 상공
        pos6_scooped = posx([300, -200, 250, 0, 170, 0]) # 식판 위
        
        # [식판 상공 원상복구]
        pos5 = posx([300, -200, 400, 0, 180, 0]) # 소스를 다 뿌리고 각도를 수평으로 복구한 식판 중간 상공

        # -----------------------------------------------------------
        # 🚀 [시퀀스 시작]
        # -----------------------------------------------------------
        print("\n🚀 통합 소스 배식 시퀀스를 시작합니다.")
        movej(JReady, vel=VELOCITY, acc=ACC)

        # 1. 국자 장착
        print("1. pos1 이동 (국자 위)")
        movejx(pos1, vel=VELOCITY, acc=ACC)
        print("2. pos2 하강 (국자 접근)")
        movel(pos2, vel=VELOCITY, acc=ACC)
        print("-> 그리퍼 잡기")
        gripper("GRIP")
        print("3. pos1 복귀 (국자 들기)")
        movel(pos1, vel=VELOCITY, acc=ACC)

        # 2. 소스 뜨기 (앞뒤 스윙)
        print("4. pos3 이동 (소스통 위)")
        movejx(pos3, vel=VELOCITY, acc=ACC)
        print("5. pos4 하강 (소스통 진입)")
        movel(pos4, vel=VELOCITY, acc=ACC)
        
        print("6. 🌟 뒤로 10도 스윙 (J5 제어 - 소스 뜨기)")
        cur_j = get_current_posj()
        if isinstance(cur_j, tuple): cur_j = cur_j[0]
        scoop_j = [float(val) for val in cur_j]
        scoop_j[4] -= 10.0  # 5번 관절(앞뒤 스윙) 10도 조절 (세팅에 따라 +일 수도 있습니다)
        movej(scoop_j, vel=VELOCITY, acc=ACC)
        wait(0.5)

        print("7. 다시 pos3으로 상승 (기울인 상태 유지)")
        # pos3_scooped를 사용하여 소스가 쏟아지지 않게 기울임(Ry)을 유지하며 상승
        movel(pos3_scooped, vel=VELOCITY, acc=ACC)

        # 3. 식판으로 이동 및 소스 뿌리기 (좌우 스윙)
        print("8. pos5 이동 (식판 가는 중간 상공 - 기울임 유지)")
        movejx(pos5_scooped, vel=VELOCITY, acc=ACC)
        print("9. pos6 하강 (식판 위 - 기울임 유지)")
        movel(pos6_scooped, vel=VELOCITY, acc=ACC)

        print("10. 🌟 좌로 10도 스윙 (J4 제어 - 소스 붓기 기울임)")
        cur_j = get_current_posj()
        if isinstance(cur_j, tuple): cur_j = cur_j[0]
        pour_j = [float(val) for val in cur_j]
        pour_j[3] -= 10.0  # 4번 관절(좌우 스윙) 10도 조절
        movej(pour_j, vel=VELOCITY, acc=ACC)
        wait(0.5)

        print("11. Y축으로 살짝 이동하며 뿌리기 (pos7 역할)")
        # 현재 기울어진 각도를 그대로 유지하며 Y축으로 100mm 상대 이동
        movel([0, 100, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL, ref=DR_BASE)
        wait(1.0) # 다 떨어질 때까지 대기

        # 4. 국자 반납
        print("12. 다시 pos5로 이동 (수평 원상복구)")
        # pos5(Ry=180)로 이동하면 J4의 좌우 스윙과 J5의 앞뒤 스윙이 모두 풀리며 깔끔하게 수직으로 정렬됩니다.
        movel(pos5, vel=VELOCITY, acc=ACC)

        print("13. pos1 이동 (반납 위치 상공)")
        movejx(pos1, vel=VELOCITY, acc=ACC)
        print("14. pos2 하강 (반납 위치)")
        movel(pos2, vel=VELOCITY, acc=ACC)
        print("-> 그리퍼 풀기")
        gripper("RELEASE")
        print("15. 다시 pos1 이동 (빈 팔 상승)")
        movel(pos1, vel=VELOCITY, acc=ACC)

        print("16. 초기 위치 복귀")
        movej(JReady, vel=VELOCITY, acc=ACC)

        print("✅ 소스 배식 시퀀스 완전 종료!")

    except KeyboardInterrupt:
        print("\n중단되었습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()