#!/usr/bin/env python3 

import rospy
from geometry_msgs.msg import Twist 
from tuos_msgs.srv import TimedMovement, TimedMovementRequest

service_name = "move_service"

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 

def callback_function(service_request): 

    vel = Twist()

    service_response = TimedMovementRequest()
    direction = service_request.movement_request
    duration = service_request.duration

    if direction == "fwd":
        vel.linear.x = 0.1
    elif direction == "back":
        vel.linear.x = -0.1
    elif direction == "left":
        vel.angular.z = 0.1
    elif direction == "right":
        vel.angular.z = -0.1
    else:
        print("Invalid direction, please use one of the following: fwd, back, left, right")
        return False
         
    print(f"The '{service_name}' Server received a {direction} request and the robot will now move for {duration} seconds...")
    StartTime = rospy.get_rostime() 
    pub.publish(vel) 

    rospy.loginfo('Published the velocity command to /cmd_vel')
    while (rospy.get_rostime().secs - StartTime.secs) < duration: 
        continue

    rospy.loginfo(f'{duration} seconds have elapsed, stopping the robot...')

    vel.linear.x = 0.0
    vel.angular.z = 0.0
    pub.publish(vel) 


    return True

rospy.init_node(f"{service_name}_server") 
my_service = rospy.Service(service_name, TimedMovement, callback_function) 
rospy.loginfo(f"the '{service_name}' Server is ready to be called...") 
rospy.spin()