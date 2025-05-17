import rclpy
import threading
import time
import sys
import os

from rclpy.logging import get_logger
from dsr_msgs2.srv import DrlPause

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m1013"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# Thread-safe flag for pausing
pause_flag = threading.Event()
pause_flag.set()  # Start as unpaused

# For stopping the thread
stop_flag = threading.Event()

logger = get_logger('dsr_motion')

# Global variable for service client, initialized on demand
pause_service_client = None # client to be created only once  and reused for subsequent calls

def send_pause_service_request(node):
    global pause_service_client

    if pause_service_client is None:
        pause_service_client = node.create_client(DrlPause, f'/{ROBOT_ID}/{ROBOT_MODEL}/dsr01/drl/drl_pause')

    # Wait briefly for service to be available
    if not pause_service_client.wait_for_service(timeout_sec=1.0):
        logger.warn("Pause service not available, skipping service call.")
        return

    req = DrlPause.Request()
    future = pause_service_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if future.result():
        logger.info(f"Pause service responded: success={future.result().success}")
    else:
        logger.error("Pause service call failed or timed out.")

def robot_motion():
    from DSR_ROBOT2 import print_ext_result, movej, movel, set_velx, set_accx, set_robot_mode
    from DSR_ROBOT2 import posj, posx
    from DSR_ROBOT2 import DR_BASE, ROBOT_MODE_AUTONOMOUS

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    set_velx(30, 20)
    set_accx(80, 30)

    velx = [50, 50]
    accx = [100, 100]

    p1 = posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)

    positions = [
        posx(600, 600, 600, 0, 175, 0),
        posx(500, 700, 500, 0, 175, 0),
        posx(400, 600, 400, 0, 175, 0),
        posx(-300, 300, 300, 0, 175, 0),
        posx(200, 300, 200, 0, 175, 0),
    ]

    count = 0
    max_count = 3

    while rclpy.ok() and count < max_count and not stop_flag.is_set():
        logger.info(f"[Cycle {count+1}] Waiting for resume signal...")

        pause_flag.wait()  # Block here if paused

        movej(p1, vel=100, acc=100)
        for pos in positions:
            pause_flag.wait()
            movel(pos, velx, accx, ref=DR_BASE)
        movej(p1, vel=100, acc=100)

        count += 1
        print("The count is", count)

    print("Motion finished")

def user_control(node):
    while rclpy.ok() and not stop_flag.is_set():
        cmd = input("[1=Pause, 2=Resume, 0=Exit] > ").strip().lower()

        if cmd == "1":
            pause_flag.clear()
            logger.info("Motion Paused.")
            send_pause_service_request(node)  # Call pause service here
        elif cmd == "2":
            pause_flag.set()
            logger.info("Motion Resumed.")
        elif cmd == "0":
            stop_flag.set()
            pause_flag.set()  # In case it's paused
            logger.info("Exiting program...")
            break
        else:
            logger.info("Invalid command. Use '1', '2', or '0'.")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('dsr_threaded_control', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # Start motion and control threads
    motion_thread = threading.Thread(target=robot_motion)
    control_thread = threading.Thread(target=user_control, args=(node,))

    motion_thread.start()
    control_thread.start()

    motion_thread.join()
    control_thread.join()

    node.destroy_node()
    rclpy.shutdown()
    print("Shutdown complete.")

if __name__ == "__main__":
    main()
