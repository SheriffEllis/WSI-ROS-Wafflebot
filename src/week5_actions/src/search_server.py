#! /usr/bin/env python3
# search_server.py

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from tuos_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow

class SearchActionServer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        # create a "simple action server" with a callback function, and start it...
        self.server_name = 'search_action_server'
        rospy.init_node(self.server_name)
        self.actionserver = actionlib.SimpleActionServer(self.server_name, SearchAction, self.action_server_launcher, False)
        self.actionserver.start()

        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The 'Search Action Server' is active...")

    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        # some checks on the "goal" input parameter(s)
        success = True
        if goal.approach_distance <= 0.15:
            print("Invalid approach distance, please input a positive number larger than 0.1")
            success = False
        
        if goal.fwd_velocity <= 0.0:
            print("Invalid forward velicity, please input a positive number larger than 0")
            success = False
        elif goal.fwd_velocity > 0.26:
            print("Invalid forward velocity, max speed is 0.26")
            success = False

        if not success:
            self.result.total_distance_travelled = 0
            self.result.closest_object_angle = 0
            self.result.closest_object_distance = 0
            self.actionserver.set_aborted(self.result)
            return

        # print a message to indicate that the requested goal was valid
        print(f"Valid goal received, setting a target forward velicity of {goal.fwd_velocity:.1f} and approach distance of {goal.fwd_velocity:.1f}")

        # Get the robot's current odometry from the Tb3Odometry() class:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        # Get information about objects up ahead from the Tb3LaserScan() class:
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position

        # set the robot's forward velocity (as specified in the "goal")...
        self.vel_controller.set_move_cmd(goal.fwd_velocity)

        ## establish a conditional statement so that the  
        ## while loop continues as long as the distance to the closest object
        ## ahead of the robot is always greater than the "approach distance"
        ## (as specified in the "goal")...
        while self.closest_object > goal.approach_distance:
            # update LaserScan data:
            self.closest_object = self.tb3_lidar.min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position # measured in degrees

            # publish a velocity command to make the robot start moving 
            self.vel_controller.publish()


            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                # take appropriate action if the action is cancelled (peempted)...

                print("Approach preempted, cancelling...")

                # stop the robot
                self.vel_controller.stop()

                # publish intermediate result
                self.actionserver.set_preempted(self.result)

                success = False
                # exit the loop:
                break

            # determine how far the robot has travelled so far:
            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))

            
            
            # update all feedback message values and publish a feedback message:
            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)



            ## update all result parameters:
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_angle = self.closest_object_location
            self.result.closest_object_distance = self.closest_object



            rate.sleep()

        if success:
            rospy.loginfo("approach completed successfully.")
            # set the action server to "succeeded" and stop the robot...
            self.actionserver.set_succeeded(self.result)
            self.vel_controller.stop()
    
    def main(self):
        rospy.spin()




if __name__ == '__main__':
    node = SearchActionServer()
    node.main()