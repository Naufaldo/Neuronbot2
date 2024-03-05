#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
import numpy as np

class WaterfillNavigation:
    def __init__(self, map_size=(31, 31)):  # Adjust the map size according to the downsampled map
        self.map_size = map_size
        self.map_grid = np.zeros(map_size)

    def update_discovered_area(self, discovered_area):
        # Resize the discovered area to match the map grid size
        resized_discovered_area = self.resize_array(discovered_area, self.map_size)
        self.map_grid = np.logical_or(self.map_grid, resized_discovered_area)

    def resize_array(self, array, new_shape):
        x_ratio = float(array.shape[1]) / new_shape[1]
        y_ratio = float(array.shape[0]) / new_shape[0]
        resized_array = np.zeros(new_shape)
        for y in range(new_shape[0]):
            for x in range(new_shape[1]):
                resized_array[y, x] = array[int(y * y_ratio), int(x * x_ratio)]
        return resized_array

    def calculate_scores(self):
        score_grid = np.ones(self.map_size) * np.inf
        score_grid[np.logical_not(self.map_grid)] = 0

        for i in range(self.map_size[0]):
            for j in range(self.map_size[1]):
                if score_grid[i, j] != 0:
                    for ni in range(max(0, i - 1), min(self.map_size[0], i + 2)):
                        for nj in range(max(0, j - 1), min(self.map_size[1], j + 2)):
                            if score_grid[ni, nj] < np.inf:
                                score_grid[i, j] = min(score_grid[i, j], score_grid[ni, nj] + 1)

        return score_grid

    def get_navigation_target(self):
        score_grid = self.calculate_scores()
        max_score_index = np.unravel_index(np.argmax(score_grid), score_grid.shape)
        return max_score_index

def map_callback(data, navigator, move_base_client):
    # Process the map data
    map_data = data.data  # Assuming data is in 1D array format
    map_array = np.array(map_data).reshape((data.info.height, data.info.width))

    # Downsample the map data to the desired grid size for waterfill method
    # For example, if each pixel in the original map represents 5cm x 5cm,
    # and you want each grid for waterfill to be 1m x 1m, you need to downsample
    # the original map by a factor of 20 (5cm * 20 = 1m)
    downsample_factor = 20
    downsampled_map = map_array[::downsample_factor, ::downsample_factor]

    # Perform waterfill navigation
    navigator.update_discovered_area(downsampled_map)
    target_position = navigator.get_navigation_target()

    rospy.loginfo("Navigation target: {}".format(target_position))

    # Publish goal pose for navigation
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = "map"
    goal_pose.target_pose.header.stamp = rospy.Time.now()
    goal_pose.target_pose.pose.position.x = target_position[1] * downsample_factor
    goal_pose.target_pose.pose.position.y = target_position[0] * downsample_factor
    goal_pose.target_pose.pose.orientation.w = 1.0

    move_base_client.send_goal(goal_pose)

def exploration_node():
    rospy.init_node('exploration_node', anonymous=True)
    navigator = WaterfillNavigation()
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()
    rospy.Subscriber('/map', OccupancyGrid, lambda data: map_callback(data, navigator, move_base_client))
    rospy.spin()

if __name__ == '__main__':
    try:
        exploration_node()
    except rospy.ROSInterruptException:
        pass
