import rclpy
import threading
import time
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger

# Doosan Robot Config
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m1013"

# Import DR_init and configure robot ID/model
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# Thread flags
pause_flag = threading.Event()
pause_flag.set()  # Robot is allowed to run initially

stop_flag = threading.Event()

# ROS logger
logger = get_logger('dsr_motion')

# Clients
pause_service_client = None
resume_service_client = None

# -- Motion Thread --
def robot_motion():
    from DSR_ROBOT2 import amovej, amovel, set_velx, set_accx, set_robot_mode
    from DSR_ROBOT2 import posj, posx
    from DSR_ROBOT2 import DR_BASE, ROBOT_MODE_AUTONOMOUS

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    set_velx(30, 20)
    set_accx(80, 30)

    velx = [50, 50]
    accx = [100, 100]

    home = posj(90.0, 0.0, 90.0, 0.0, 90.0, 0.0)
    positions = [
        posx(227.900, 389.740, 451.650, 25.76, -179.93, -64.10),
        posx(23.03, 464.74, 395.40, 25.67, -179.93, -64.19),
        posx(-55.59, 389.74, 409.40, 25.77, -179.93, -64.09),
        posx(-55.59, 488.490, 357.030, 25.73, -179.93, -64.13)
    ]

    count = 0
    max_count = 5

    while rclpy.ok() and count < max_count and not stop_flag.is_set():
        logger.info(f"[Cycle {count + 1}] Sending motions...")

        amovej(home, vel=50, acc=50)
        wait_with_pause_check(3)

        for pos in positions:
            if stop_flag.is_set():
                return
            amovel(pos, velx, accx, ref=DR_BASE)
            wait_with_pause_check(3)

        amovej(home, vel=50, acc=50)
        wait_with_pause_check(3)

        count += 1
        logger.info(f"Completed cycle {count}")

    logger.info("Motion thread exiting.")


# -- Pause-aware Wait --
def wait_with_pause_check(duration):
    elapsed = 0
    interval = 0.1
    while elapsed < duration:
        if stop_flag.is_set():
            return
        if not pause_flag.is_set():
            time.sleep(0.1)
            continue
        time.sleep(interval)
        elapsed += interval


# -- Service Functions --
from dsr_msgs2.srv import MovePause, MoveResume

def send_pause_service_request(node):
    global pause_service_client

    if pause_service_client is None:
        pause_service_client = node.create_client(MovePause,'/dsr01/motion/move_pause')

    if not pause_service_client.wait_for_service(timeout_sec=3.0):
        logger.warn("Pause service not available.")
        return

    req = MovePause.Request()
    future = pause_service_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

    if future.result():
        logger.info(f"Pause success: {future.result().success}")
        pause_flag.clear()
    else:
        logger.error("Pause service call failed.")


def send_resume_service_request(node):
    global resume_service_client

    if resume_service_client is None:
        resume_service_client = node.create_client(MoveResume,'/dsr01/motion/move_resume')

    if not resume_service_client.wait_for_service(timeout_sec=20.0):
        logger.warn("Resume service not available.")
        return

    req = MoveResume.Request()
    future = resume_service_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)

    if future.result():
        logger.info(f"Resume success: {future.result().success}")
        pause_flag.set()
    else:
        logger.error("Resume service call failed.")


# -- User Control Thread --
def user_control():
    node = DR_init.__dsr__node
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    while rclpy.ok() and not stop_flag.is_set():
        cmd = input("[1: Pause / 2: Resume / 0: Exit] > ").strip().lower()

        if cmd == "1":
            send_pause_service_request(node)
        elif cmd == "2":
            send_resume_service_request(node)
        elif cmd == "0":
            stop_flag.set()
            logger.info("Exiting program...")
        else:
            logger.info("Invalid command. Use '1', '2', or '0'.")


# -- Main --
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('dsr_threaded_control', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    motion_thread = threading.Thread(target=robot_motion)
    control_thread = threading.Thread(target=user_control)

    motion_thread.start()
    control_thread.start()

    motion_thread.join()
    control_thread.join()

    node.destroy_node()
    rclpy.shutdown()
    logger.info("Shutdown complete.")


if __name__ == "__main__":
    main()
