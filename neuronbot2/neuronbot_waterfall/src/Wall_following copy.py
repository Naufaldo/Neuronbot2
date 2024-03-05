#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class WallFollowingNavigation:
    def __init__(self):
        rospy.init_node('wall_following_navigation')
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

        # Example: Wall Following algorithm
        left_obstacle, right_obstacle = self.detect_obstacles(self.laser_data)

        # Set velocities based on obstacle detection
        cmd_vel = Twist()
        if left_obstacle and not right_obstacle:
            # Obstacle on the left, turn right
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -0.5  # Turn right
        elif not left_obstacle and right_obstacle:
            # Obstacle on the right, turn left
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn left
        else:
            # No obstacle, move forward
            cmd_vel.linear.x = 0.2  # Example linear velocity
            cmd_vel.angular.z = 0.0

        # Publish velocity command
        self.cmd_publisher.publish(cmd_vel)

    def detect_obstacles(self, laser_data):
        # Detect obstacles on the left and right side based on laser scan data
        # Example: Check if any obstacle is within a predefined range on each side
        left_obstacle = any(dist < 0.5 for dist in laser_data.ranges[:len(laser_data.ranges)//2])
        right_obstacle = any(dist < 0.5 for dist in laser_data.ranges[len(laser_data.ranges)//2:])
        return left_obstacle, right_obstacle

if __name__ == '__main__':
    navigation = WallFollowingNavigation()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        navigation.navigate()
        rate.sleep()
