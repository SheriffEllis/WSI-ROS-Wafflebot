#!/usr/bin/env python3
# A script to attempt to hug the right wall during initial exploration phase

import rospy
from math import pow

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

MAX_FWD_VEL = 0.15
MAX_ANG_VEL = 0.26 # 0.26
MIN_WALL_DIST = 0.2
MAX_WALL_DIST = 0.8
TGT_ANG = 90
K_P = 5
K_I = 0.1

class WallHugger(): 

    def __init__(self): 
        self.node_name = "wall_hugger"
        rospy.init_node(self.node_name, anonymous=True)

        # Peripheral controller
        self.vel_controller = Tb3Move()
        self.odom = Tb3Odometry()
        self.lidar = Tb3LaserScan()

        # PID variables
        self.ang_integral = 0

        self.rate = rospy.Rate(10) 
        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.vel_controller.set_move_cmd(0,0) # stop all movement
        self.vel_controller.publish()
        self.ctrl_c = True

    def constrain(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def main_loop(self):
        while not self.ctrl_c: 
            # Get the location of the closest wall
            closest_distance = self.lidar.min_distance
            closest_angle = self.lidar.closest_object_position # degrees

            ang_error = (TGT_ANG - closest_angle)/TGT_ANG
            self.ang_integral += ang_error
            if(abs(ang_error) < 5): self.ang_integral = 0
            pid = K_P*ang_error + K_I*self.ang_integral
            pid = self.constrain(pid, -1, 1)

            print(f"Distance: {closest_distance}\tAngle: {closest_angle}\tPID: {pid}")
            
            ang_velocity = MAX_ANG_VEL*pid # turn left if wall not perpendicular to your right

            if(closest_distance >= MAX_WALL_DIST):
                fwd_velocity = MAX_FWD_VEL
            else:
                fwd_velocity = (-pid + 1)*MAX_FWD_VEL*(closest_distance >= MIN_WALL_DIST or abs(pid) < 0.2) # don't move if too close to a wall and not perpendicular

            self.vel_controller.set_move_cmd(fwd_velocity, ang_velocity)
            self.vel_controller.publish()
            self.rate.sleep()

if __name__ == '__main__': 
    publisher_instance = WallHugger() 
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass