#! /usr/bin/env python3
# search_client.py

import rospy
import actionlib

from tuos_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class SearchActionClient():
    goal = SearchGoal()

    def feedback_callback(self, feedback_data: SearchFeedback):
        ## get the current distance travelled, from the feedback message
        ## and assign this to a class variable...
        self.distance = feedback_data.current_distance_travelled



    def __init__(self):
        self.distance = 0.0
        self.client_name = 'search_action_client'
        self.server_name = 'search_action_server'

        self.action_complete = False
        rospy.init_node(self.client_name)
        self.rate = rospy.Rate(1)

        ## setup a "simple action client" with a callback function
        ## and wait for the server to be available...
        self.client = actionlib.SimpleActionClient(self.server_name, SearchAction) 
        self.client.wait_for_server()



        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            ## cancel the goal request, if this node is shutdown before the action has completed...
            self.client.cancel_goal()


            rospy.logwarn("Goal Cancelled...")

        ## print the result here...
        rospy.sleep(1) # wait for the result
        result = self.client.get_result()
        print("RESULT:")
        print(f"  * Action State = {self.client.get_state()}")
        print(f"  * Total distance travelled was {result.total_distance_travelled:.1f}")
        print(f"  * Closest object is {result.closest_object_distance:.1f}m away")
        print(f"  * Closest object is {result.closest_object_angle:.1f} degrees relative to the heading")




    def main_loop(self):
        ## TODO: assign values to all goal parameters
        ## and send the goal to the action server...
        self.goal.approach_distance = 0.2
        self.goal.fwd_velocity = 0.2
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)



        while self.client.get_state() < 2:
            ## TODO: Construct an if statement and cancel the goal if the 
            ## distance travelled exceeds 2 meters...
            if self.distance > 2:
                rospy.logwarn("Max distance of 2m exceeded, Cancelling Goal...")
                self.client.cancel_goal()

                rospy.logwarn("Goal Cancelled...")

                # break out of the while loop to stop the node:
                break

            self.rate.sleep()

        self.action_complete = True

if __name__ == '__main__':
    ## Instantiate the node and call the main_loop() method from it...
    node = SearchActionClient()
    node.main_loop()