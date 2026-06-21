import time
import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_force_control", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            release_force,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_1")
    set_tcp("GripperDA_v1")

    pos = posx([496.06, 93.46, 96.92, 20.75, 179.00, 19.09])
    JReady = posj([0, 0, 90, 0, 90, 0])

    # while rclpy.ok():
    for i in range(5):
        print(f"Cycle {i+1}")

        force_enabled = False

        try:

            print(f"Moving to joint position: {JReady}")
            movej(JReady, vel=VELOCITY, acc=ACC)

            print(f"Moving to task position: {pos}")
            movel(pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)

            print("Starting task_compliance_ctrl")
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            time.sleep(0.5)

            force_enabled = True

            print("Starting set_desired_force")
            set_desired_force(
                fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL
            )

            # 외력 절대값이 5N 이상이면 True
            # 외력 절대값이 5N 미만이면 False
            start_time = time.time()
            while not check_force_condition(DR_AXIS_Z, min=5, ref=DR_BASE):
                print("Waiting for an external force greater than 5 ")
                if time.time() - start_time > 10:
                    raise Exception("Force detection timeout")

                time.sleep(0.1)
        except Exception as e:
            print(e)

        finally:
            if force_enabled:
                print("Starting release_force")
                release_force()
                time.sleep(0.5)

                print("Starting release_compliance_ctrl")
                release_compliance_ctrl()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
