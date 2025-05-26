import rclpy
import threading
import time
import sys
import os
import yaml

from rclpy.logging import get_logger
from dsr_msgs2.srv import DrlPause, GetCurrentPosx

# Robot configuration
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m1013"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# Thread-safe control flags
pause_flag = threading.Event()
pause_flag.set()  # Start unpaused

stop_flag = threading.Event()

logger = get_logger('dsr_motion')

# Global service clients
pause_service_client = None
pose_service_client = None

# File to store saved poses
POSE_SAVE_PATH = "recorded_poses.yaml"


def send_pause_service_request(node):
    global pause_service_client

    if pause_service_client is None:
        pause_service_client = node.create_client(DrlPause, f'/{ROBOT_ID}/{ROBOT_MODEL}/dsr01/m1013/dsr01/drl/drl_pause')

    if not pause_service_client.wait_for_service(timeout_sec=1.0):
        logger.warn("Pause service not available.")
        return

    req = DrlPause.Request()
    future = pause_service_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if future.result():
        logger.info(f"Pause service responded: success={future.result().success}")
    else:
        logger.error("Pause service call failed or timed out.")


def save_current_pose(node):
    global pose_service_client

    if pose_service_client is None:
        pose_service_client = node.create_client(GetCurrentPosx, f'/{ROBOT_ID}/{ROBOT_MODEL}/dsr01/aux_control/get_current_posx')

    if not pose_service_client.wait_for_service(timeout_sec=2.0):
        logger.warn("Pose service not available.")
        return

    req = GetCurrentPosx.Request()
    future = pose_service_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)

    if future.result():
        pos = future.result().pos
        pose_dict = {
            "x": pos.x,
            "y": pos.y,
            "z": pos.z,
            "rx": pos.rx,
            "ry": pos.ry,
            "rz": pos.rz,
            "timestamp": time.time()
        }

        # Append to YAML file
        if os.path.exists(POSE_SAVE_PATH):
            with open(POSE_SAVE_PATH, 'r') as f:
                data = yaml.safe_load(f) or []
        else:
            data = []

        data.append(pose_dict)
        with open(POSE_SAVE_PATH, 'w') as f:
            yaml.dump(data, f)

        logger.info(f"Pose saved to {POSE_SAVE_PATH}")
    else:
        logger.error("Failed to get pose from service.")


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

        pause_flag.wait()  # Pause point

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
        cmd = input("[1=Pause & Save Pose, 2=Resume, 0=Exit] > ").strip().lower()

        if cmd == "1":
            pause_flag.clear()
            logger.info("Motion Paused.")
            send_pause_service_request(node)
            save_current_pose(node)  # Save pose on pause
        elif cmd == "2":
            pause_flag.set()
            logger.info("Motion Resumed.")
        elif cmd == "0":
            stop_flag.set()
            pause_flag.set()
            logger.info("Exiting program...")
            break
        else:
            logger.info("Invalid command. Use '1', '2', or '0'.")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('dsr_threaded_control', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

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

