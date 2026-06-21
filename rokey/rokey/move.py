import rclpy
import DR_init

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"
VELOCITY, ACC = 20, 20

DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model= ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_1")
    set_tcp("GripperDA_v1")

    JReady= posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    posx1 = posx([350.0, 34.5, 500.0, 45.0, 180.0, 45.0])
    posx2 = posx([350.0, 34.5, 500.0, 45.0, 90.0, 45.0])
    posx3 = posx([350.0, 34.5, 500.0, 0.0, 90.0, 45.0])

    homej = posj([0.0, 0.0, 9.0, 0.0, 30.0, 0.0])

    print(f"Moving to joint position: {JReady}")
    movej(JReady, vel=VELOCITY, acc=ACC)

    print(f"Moving to task position: {posx1}")
    movel(posx1, vel=VELOCITY, acc=ACC)
    movel(posx2, vel=VELOCITY, acc=ACC)
    movel(posx3, vel=VELOCITY, acc=ACC)

    movej(homej, vel=VELOCITY, acc=ACC)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
