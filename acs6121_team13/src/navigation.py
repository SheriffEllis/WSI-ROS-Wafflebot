#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Global variables to keep state and control behavior
turning = False  # Indicates if the robot is in a turning state
turn_direction = 0  # Last turn direction, 0 for none, -1 for left, 1 for right
wall_following = False  # Indicates if the robot is currently following a wall

def callback(data):
    global turning, last_turn, wall_following
    segments = analyze_segments(data)
    if rospy.Time.now() - last_turn > rospy.Duration(2) or not turning:
        if wall_following:
            follow_wall(segments)
        else:
            decide_movement(segments)
    else:
        maintain_current_course(segments)

def analyze_segments(data):
    # Safe calculation of minimum distances avoiding zero distances which represent no return
    process_range = lambda ranges: min([x for x in ranges if x > 0.1], default=float('inf'))
    segments = {
        'front': process_range(data.ranges[0:18] + data.ranges[-18:]),
        'front_left': process_range(data.ranges[18:75]),
        'front_right': process_range(data.ranges[-75:-18]),
        'left': process_range(data.ranges[75:105]),
        'right': process_range(data.ranges[-105:-75]),
        'rear': process_range(data.ranges[135:225])
    }
    return segments

def decide_movement(segments):
    global turning, last_turn, turn_direction, wall_following
    critical_threshold = 0.5  # Increased threshold for immediate action
    stop_threshold = 0.8  # Increased distance at which robot should stop and consider turning
    clear_threshold = 1.2  # Increased distance considered safe for forward movement

    move.linear.x = 0.0
    move.angular.z = 0.0

    if segments['front'] < critical_threshold:
        # Critical situation, need to reverse or stop
        move.linear.x = -0.05  # Reverse slightly
        turning = True
        last_turn = rospy.Time.now()
    elif segments['front'] > clear_threshold and not turning:
        # Path is clear, proceed at a cautious speed
        move.linear.x = 0.25
        wall_following = False
    elif segments['front'] > stop_threshold:
        # Slow down but keep moving forward
        move.linear.x = 0.1
        turning = False
        wall_following = False
    else:
        # Too close to an obstacle, initiate turning
        turning = True
        last_turn = rospy.Time.now()
        initiate_turning(segments)

    pub.publish(move)

def initiate_turning(segments):
    global turn_direction
    # Decide which way to turn based on side distances
    if segments['left'] > segments['right']:
        move.angular.z = -0.5  # Turn right
        turn_direction = 1
    else:
        move.angular.z = 0.5  # Turn left
        turn_direction = -1

def follow_wall(segments):
    global turning, move
    wall_threshold = 1.0  # Adjusted ideal distance to maintain from the wall
    # Adjust to maintain a steady distance from the wall
    if segments['left'] < wall_threshold or segments['right'] < wall_threshold:
        move.linear.x = 0.2
        move.angular.z = -0.3 if segments['left'] < wall_threshold else 0.3
    else:
        move.linear.x = 0.2
        move.angular.z = 0.0
    pub.publish(move)

def maintain_current_course(segments):
    global turning, move
    # If path clears while turning, consider moving forward again
    if segments['front'] > 0.6:
        move.linear.x = 0.2
        move.angular.z = 0.0
        turning = False
    else:
        # Continue in the last turning direction with slight forward motion
        move.linear.x = 0.0
        move.angular.z = 0.5 if turn_direction == 1 else -0.5
    pub.publish(move)

def stop_robot():
    # Ensure the robot stops by publishing zero velocities
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)
    rospy.loginfo("Robot has stopped safely.")

def main():
    global move, pub, last_turn
    rospy.init_node('turtlebot3_explorer')

    move = Twist()
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    last_turn = rospy.Time.now()  # Initialize last_turn after rospy.init_node()
    sub = rospy.Subscriber("/scan", LaserScan, callback)

    rospy.on_shutdown(stop_robot)  # Set the shutdown hook to stop the robot
    rospy.spin()

if __name__ == '__main__':
    main()

