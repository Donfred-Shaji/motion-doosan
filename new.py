import rclpy
import os
import sys

from rclpy.logging import get_logger

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m1013"

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

logger = get_logger('NEW')

def main(args=None):
        rclpy.init(args=args)

        node = rclpy.create_node('NEW_py', namespace=ROBOT_ID)

        DR_init.__dsr__node = node

        try:
                from DSR_ROBOT2 import print_ext_result, movej, movel, set_velx, set_accx, set_robot_mode
                from DSR_ROBOT2 import posj, posx
                from DSR_ROBOT2 import DR_BASE, ROBOT_MODE_AUTONOMOUS
                # print_result("Import DSR_ROBOT2 Success!")
        except ImportError as e:
                print(f"Error importing DSR_ROBOT2 : {e}")
                return
        
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        set_velx(30, 20)    # set global task speed : 30(mm/sec), 20(deg/sec)
        set_accx(60, 40)    # set global task speed : 60(mm/sec2), 40(deg/sec2)

        velx = [50, 50]
        accx = [100, 100]

        
        p1= posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0) #joint

        x1 = posx(600, 600, 600, 0, 175, 0)
        x2 = posx(500, 700, 500, 0, 175, 0)
        x3 = posx(400, 600, 400, 0, 175, 0)
        x4 = posx(-300, 300, 300, 0, 175, 0)
        x5 = posx(200, 300, 200, 0, 175, 0)
        

        

        while rclpy.ok():
                movej(p1, vel=100, acc=100)

                movel(x1, velx, accx, ref=DR_BASE)
                movel(x2, velx, accx, ref=DR_BASE )
                movel(x3, velx, accx, ref=DR_BASE)
                movel(x4, velx, accx, ref=DR_BASE)
                movel(x5, velx, accx, ref=DR_BASE)
                movej(p1, vel=100, acc=100)
                


        print('o')
        rclpy.shutdown()
        

if __name__ == "__main__":
        main()

