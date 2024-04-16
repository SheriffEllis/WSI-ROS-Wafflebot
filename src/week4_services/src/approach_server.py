#!/usr/bin/env python3 

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tuos_msgs.srv import Approach, ApproachRequest

class ApproachSrv():
    def callback_function(self, service_request): 
        vel = Twist()

        service_response = ApproachRequest()
        dist_tg = service_request.approach_distance
        vel_tg = service_request.approach_velocity
    
        # Continue to approach object as long as distance is above threshold (or distance is 0 due to timeout)
        while(self.min_distance > dist_tg or self.min_distance == 0):
            # Limit rate to that of scan topic
            if(self.new_measure):
                vel.linear.x = vel_tg
                self.pub.publish(vel)
                rospy.loginfo(f"Object distance: {self.min_distance}")
                self.new_measure = False
        
        vel.linear.x = 0.0
        self.pub.publish(vel) 
        
        rospy.loginfo("Object reached")
        service_response = "Object reached"
        return service_response

    def scan_callback(self, scan_data): 
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = front_arc.min()
        self.new_measure = True

        # Optional Extra:
        arc_angles = np.arange(-20, 21)
        self.object_angle = arc_angles[np.argmin(front_arc)]
    
    def __init__(self):
        self.service_name = "approach_service" 
        rospy.init_node(f"{self.service_name}_server") 
        self.my_service = rospy.Service(self.service_name, Approach, self.callback_function)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Main loop attributes
        self.min_distance = 0
        self.object_angle = 0

        self.new_measure = False
        
        rospy.loginfo(f"the '{self.service_name}' Server is ready to be called...") 
        
    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    server_instance = ApproachSrv()
    server_instance.main_loop()