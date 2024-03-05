#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from exploration_navigation.waterfill_navigation import WaterfillNavigation
import numpy as np

def map_callback(data):
    # Process the map data
    map_data = data.data  # Assuming data is in 1D array format
    map_array = np.array(map_data).reshape((data.info.height, data.info.width))

    # Perform waterfill navigation
    navigator = WaterfillNavigation(map_size=(data.info.height, data.info.width))
    navigator.update_discovered_area(map_array)
    target_position = navigator.get_navigation_target()

    rospy.loginfo("Navigation target: {}".format(target_position))

def exploration_node():
    rospy.init_node('exploration_node', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        exploration_node()
    except rospy.ROSInterruptException:
        pass
