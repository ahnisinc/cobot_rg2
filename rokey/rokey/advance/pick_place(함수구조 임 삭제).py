import rclpy
from rclpy.node import Node
import time
import copy

import DR_init

# =========================
# 기본 설정
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

ON, OFF = 1, 0

FORCE_THRESHOLD = 5.0
DOWN_DISTANCE = 200

gripper_status = 0

pos1 = None
pos2 = None


# =========================
# ROS2 Node
# =========================
class DSRNode(Node):
    def __init__(self):
        super().__init__("pick_place_node", namespace=ROBOT_ID)


# =========================
# 그리퍼 함수 (DSR_ROBOT2는 필요 시 내부 import)
# =========================
def open_gripper():
    from DSR_ROBOT2 import set_digital_output

    global gripper_status

    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    time.sleep(1.0)

    gripper_status = 0


def close_gripper():
    from DSR_ROBOT2 import set_digital_output

    global gripper_status

    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    time.sleep(1.0)

    gripper_status = 1


# =========================
# force 기반 하강
# =========================
def move_down_until_contact():
    from DSR_ROBOT2 import amovel, movel, get_tool_force, get_current_posx

    target = copy.deepcopy(pos1)
    target[2] -= DOWN_DISTANCE

    amovel(target, 20, 20)

    while True:
        force = get_tool_force()
        if force[2] > FORCE_THRESHOLD:
            movel(pos1, 20, 20)
            contact = get_current_posx()[0]
            return contact


# =========================
# pick
# =========================
def pick():
    from DSR_ROBOT2 import movel

    global gripper_status

    movel(pos1, 50, 50)
    move_down_until_contact()

    if gripper_status == 0:
        close_gripper()

    movel(pos1, 50, 50)


# =========================
# place
# =========================
def place():
    from DSR_ROBOT2 import movel

    global gripper_status

    movel(pos2, 50, 50)
    move_down_until_contact()

    if gripper_status == 1:
        open_gripper()

    movel(pos2, 50, 50)


# =========================
# main
# =========================
def main():
    global pos1, pos2

    rclpy.init()

    node = DSRNode()

    DR_init.__dsr__node = node

    # DSR_ROBOT2 import (여기서 해야 g_node 정상 연결)
    from DSR_ROBOT2 import posx

    # 좌표 생성
    pos1 = posx(574.07, 149.69, 340.0, 0, -180, 0)
    pos2 = posx(276.04, 147.85, 340.75, 0, -180, 0)

    time.sleep(2)  # ROS2 service 안정화

    open_gripper()

    while rclpy.ok():
        pick()
        place()


if __name__ == "__main__":
    main()
