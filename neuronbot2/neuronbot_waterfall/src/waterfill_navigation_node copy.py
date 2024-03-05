#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class WaterfillNavigation:
    def __init__(self):
        rospy.init_node('waterfill_navigation')
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.map_data = None
        self.laser_data = None
        self.robot_position = None

    def map_callback(self, msg):
        # Store the map data
        self.map_data = msg

    def laser_callback(self, msg):
        # Store the laser scan data
        self.laser_data = msg

    def odom_callback(self, msg):
        # Store the robot's position from odometry data
        self.robot_position = msg.pose.pose.position if msg.pose.pose.position else None

    def navigate(self):
        if self.map_data is None or self.laser_data is None or self.robot_position is None:
            rospy.loginfo("Waiting for map, laser scan, or odometry data...")
            return

        # Convert the occupancy grid to a numpy array for easier manipulation
        map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))

        # Example: Waterfill algorithm
        # Assume the target is the cell with the lowest value
        target_cell = np.unravel_index(np.argmin(map_array), map_array.shape)

        # Calculate the gradient map
        gradient_map = self.calculate_gradient_map(map_array, target_cell)

        # Determine the direction to move
        direction = self.determine_direction(gradient_map, self.robot_position)

        # Perform obstacle avoidance using laser scan data
        obstacle_avoidance_direction = self.perform_obstacle_avoidance(self.laser_data)

        # Combine directions (you may need to adjust this part based on your strategy)
        combined_direction = self.combine_directions(direction, obstacle_avoidance_direction)

        # Publish velocity command based on the combined direction
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1  # Example linear velocity
        cmd_vel.angular.z = 0.0  # Example angular velocity
        self.cmd_publisher.publish(cmd_vel)

    def calculate_gradient_map(self, map_array, target_cell):
        # Calculate gradient map using distance transform or other methods
        # Example: distance transform
        distance_map = np.linalg.norm(np.indices(map_array.shape) - np.array(target_cell)[:, None, None], axis=0)
        gradient_map = np.where(map_array == 0, distance_map, np.inf)
        return gradient_map

    def determine_direction(self, gradient_map, robot_position):
        # Determine the direction to move based on the gradient map and robot position
        # Example: Move towards the lowest gradient direction
        direction = np.unravel_index(np.argmin(gradient_map), gradient_map.shape)
        return direction

    def perform_obstacle_avoidance(self, laser_data):
        # Perform obstacle avoidance using laser scan data
        # Example: Avoid obstacles detected within a certain range
        # (You may need to adjust this based on your robot's characteristics and the environment)
        min_distance = min(laser_data.ranges)
        if min_distance < 0.5:  # If an obstacle is detected within 0.5 meters, turn left
            return np.pi / 2  # 90 degrees (left turn)
        else:
            return 0  # No avoidance needed

    def combine_directions(self, direction, obstacle_avoidance_direction):
        # Combine navigation direction with obstacle avoidance direction
        # Example: prioritize obstacle avoidance
        combined_direction = obstacle_avoidance_direction
        return combined_direction

if __name__ == '__main__':
    navigation = WaterfillNavigation()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        navigation.navigate()
        rate.sleep()
