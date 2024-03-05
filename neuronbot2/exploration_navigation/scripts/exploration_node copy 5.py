#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import numpy as np

class FloodfillNavigation:
    def __init__(self):
        self.map_size = None
        self.max_position = None
        self.map_grid = None
        self.visited = None
        self.reached_goal = True

    def update_discovered_area(self, discovered_area):
        if self.map_size is None:
            rospy.logerr("Map size is not yet set. Ignoring update_discovered_area call.")
            return

        if discovered_area.shape[0] < self.map_size[0] or discovered_area.shape[1] < self.map_size[1]:
            # If the discovered area is smaller than the map size, pad it with False values
            padded_discovered_area = np.zeros(self.map_size, dtype=bool)
            padded_discovered_area[:discovered_area.shape[0], :discovered_area.shape[1]] = discovered_area
            discovered_area = padded_discovered_area
        elif discovered_area.shape[0] > self.map_size[0] or discovered_area.shape[1] > self.map_size[1]:
            # If the discovered area is larger than the map size, crop it
            rospy.logwarn("Discovered area dimensions exceed map size. Cropping to match map size.")
            discovered_area = discovered_area[:self.map_size[0], :self.map_size[1]]

        self.map_grid |= discovered_area
        self.visited |= discovered_area

    def get_navigation_target(self):
        if self.map_size is None:
            rospy.logerr("Map size is not yet set. Cannot determine navigation target.")
            return None

        # Check if there are any undiscovered areas left
        if np.any(~self.visited):
            # Get indices of all undiscovered areas
            undiscovered_indices = np.transpose(np.nonzero(~self.visited))
            
            # Select a random undiscovered position as the navigation target
            idx = np.random.randint(len(undiscovered_indices))
            undiscovered_position = undiscovered_indices[idx]

            return self.scale_position(undiscovered_position)
        else:
            rospy.loginfo("All areas visited. No target to navigate.")
            return None

    def scale_position(self, position):
        if self.max_position is None:
            rospy.logerr("Max position is not yet set. Cannot scale position.")
            return None

        x_scale = float(self.max_position) / self.map_size[1]
        y_scale = float(self.max_position) / self.map_size[0]
        scaled_x = position[1] * x_scale
        scaled_y = position[0] * y_scale
        return [scaled_x, scaled_y]

def map_callback(data, navigator):
    map_data = data.data
    map_array = np.array(map_data).reshape((data.info.height, data.info.width))

    if navigator.map_size is None:
        navigator.map_size = (map_array.shape[0], map_array.shape[1])
        navigator.max_position = 5  # Default max position, adjust as needed
        navigator.map_grid = np.zeros(navigator.map_size, dtype=bool)
        navigator.visited = np.zeros(navigator.map_size, dtype=bool)

        # Trigger exploration upon map initialization
        rospy.loginfo("Map initialized. Triggering exploration.")
        target_position = navigator.get_navigation_target()

        if target_position is not None and navigator.reached_goal:
            rospy.loginfo("Navigation target: {}".format(target_position))

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.pose.position.x = target_position[0]
            goal_pose.pose.position.y = target_position[1]
            goal_pose.pose.orientation.w = 1.0

            goal_pub.publish(goal_pose)
            navigator.reached_goal = False
        else:
            rospy.loginfo("Exploration complete. No target to navigate.")

def goal_result_callback(data, navigator):
    if data.status.status == GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached. Planning new navigation target.")
        navigator.reached_goal = True
        map_callback(last_map_data, navigator)

def exploration_node():
    rospy.init_node('exploration_node', anonymous=True)
    navigator = FloodfillNavigation()
    rospy.Subscriber('/map', OccupancyGrid, lambda data: map_callback(data, navigator))
    rospy.Subscriber('/move_base/result', GoalStatusArray, lambda data: goal_result_callback(data, navigator))
    rospy.spin()

if __name__ == '__main__':
    try:
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        exploration_node()
    except rospy.ROSInterruptException:
        pass
