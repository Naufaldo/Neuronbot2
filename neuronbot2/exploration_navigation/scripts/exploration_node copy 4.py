#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np

class FloodfillNavigation:
    def __init__(self, map_size=(31, 31), max_position=5):  
        self.map_size = map_size
        self.max_position = max_position
        self.map_grid = np.zeros(map_size)
        self.visited = np.zeros(map_size, dtype=bool)

    def update_discovered_area(self, discovered_area):
        # Resize the discovered_area to match the map size
        resized_area = self.resize_discovered_area(discovered_area)
        self.map_grid = np.logical_or(self.map_grid, resized_area)
        self.visited = np.logical_or(self.visited, resized_area)

    def resize_discovered_area(self, discovered_area):
        # Resize the discovered_area to match the map size
        return np.array(discovered_area, dtype=bool)[:self.map_size[0], :self.map_size[1]]

    def get_navigation_target(self):
        indices = np.transpose(np.nonzero(~self.visited))
        if len(indices) == 0:
            return None
        else:
            return self.scale_position(indices[0])

    def scale_position(self, position):
        x_scale = float(self.max_position) / self.map_size[1]
        y_scale = float(self.max_position) / self.map_size[0]
        scaled_x = position[1] * x_scale
        scaled_y = position[0] * y_scale
        return [scaled_x, scaled_y]

def map_callback(data, navigator):
    map_data = data.data  
    map_array = np.array(map_data).reshape((data.info.height, data.info.width))

    downsample_factor = 2
    downsampled_map = map_array[::downsample_factor, ::downsample_factor]

    navigator.update_discovered_area(downsampled_map)
    target_position = navigator.get_navigation_target()

    if target_position is not None:
        rospy.loginfo("Navigation target: {}".format(target_position))

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = target_position[0]
        goal_pose.pose.position.y = target_position[1]
        goal_pose.pose.orientation.w = 1.0

        goal_pub.publish(goal_pose)
    else:
        rospy.loginfo("All areas visited. No target to navigate.")

def exploration_node():
    rospy.init_node('exploration_node', anonymous=True)
    navigator = FloodfillNavigation()
    rospy.Subscriber('/map', OccupancyGrid, lambda data: map_callback(data, navigator))
    rospy.spin()

if __name__ == '__main__':
    try:
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        exploration_node()
    except rospy.ROSInterruptException:
        pass
