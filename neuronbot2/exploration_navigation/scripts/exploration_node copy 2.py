#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import numpy as np

class FloodfillNavigation:
    def __init__(self, map_size=(31, 31)):  # Adjust the map size according to the downsampled map
        self.map_size = map_size
        self.map_grid = np.zeros(map_size)
        self.visited = np.zeros(map_size, dtype=bool)

    def update_discovered_area(self, discovered_area):
        # Update the map grid and visited flags
        self.map_grid = np.logical_or(self.map_grid, discovered_area)
        self.visited = np.logical_or(self.visited, discovered_area)

    def get_navigation_target(self):
        # Find the next unvisited cell for navigation
        indices = np.transpose(np.nonzero(~self.visited))
        if len(indices) == 0:
            return None  # No unvisited cells left
        else:
            return indices[0]

def map_callback(data, navigator, move_base_client):
    # Process the map data
    map_data = data.data  # Assuming data is in 1D array format
    map_array = np.array(map_data).reshape((data.info.height, data.info.width))

    # Downsample the map data to the desired grid size for flood-fill method
    # For example, if each pixel in the original map represents 5cm x 5cm,
    # and you want each grid for flood-fill to be 1m x 1m, you need to downsample
    # the original map by a factor of 20 (5cm * 20 = 1m)
    downsample_factor = 20
    downsampled_map = map_array[::downsample_factor, ::downsample_factor]

    # Perform flood-fill navigation
    navigator.update_discovered_area(downsampled_map)
    target_position = navigator.get_navigation_target()

    if target_position is not None:
        rospy.loginfo("Navigation target: {}".format(target_position))

        # Publish goal pose for navigation
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = "map"
        goal_pose.target_pose.header.stamp = rospy.Time.now()
        goal_pose.target_pose.pose.position.x = target_position[1] * downsample_factor
        goal_pose.target_pose.pose.position.y = target_position[0] * downsample_factor
        goal_pose.target_pose.pose.orientation.w = 1.0

        move_base_client.send_goal(goal_pose)
    else:
        rospy.loginfo("All areas visited. No target to navigate.")

def exploration_node():
    rospy.init_node('exploration_node', anonymous=True)
    navigator = FloodfillNavigation()
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()
    rospy.Subscriber('/map', OccupancyGrid, lambda data: map_callback(data, navigator, move_base_client))
    rospy.spin()

if __name__ == '__main__':
    try:
        exploration_node()
    except rospy.ROSInterruptException:
        pass
