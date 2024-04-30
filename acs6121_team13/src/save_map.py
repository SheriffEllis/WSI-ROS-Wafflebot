#!/usr/bin/env python3

import roslaunch
import rospy
import time
import os
import rospkg

rospy.init_node("map_node", anonymous=True)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

rospack = rospkg.RosPack()
package_path = rospack.get_path('acs6121_team13')
maps_directory = os.path.join(package_path, 'maps')

map_path = os.path.join(maps_directory, 'explore_map')

try:
    while not rospy.is_shutdown():
        print(f"Saving map at time: {rospy.get_time()}...")
        node = roslaunch.core.Node(package="map_server",
                                   node_type="map_saver",
                                   args=f"-f {map_path}")
        process = launch.launch(node)
        time.sleep(5)
except rospy.ROSInterruptException:
    print("Map saving stopped")