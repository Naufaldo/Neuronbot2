#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import time

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
        self.x_positions = []
        self.y_positions = []
        self.start_time = time.time()
        self.duration_minutes = 5  # Set the duration in minutes

    def map_callback(self, msg):
        # Store the map data
        self.map_data = msg

    def laser_callback(self, msg):
        # Store the laser scan data
        self.laser_data = msg

    def odom_callback(self, msg):
        # Store the robot's position from odometry data
        self.robot_position = msg.pose.pose.position if msg.pose.pose.position else None
        if self.robot_position:
            self.x_positions.append(self.robot_position.x)
            self.y_positions.append(self.robot_position.y)

    def navigate(self):
        if self.map_data is None or self.laser_data is None or self.robot_position is None:
            rospy.loginfo("Waiting for map, laser scan, or odometry data...")
            return

        # Check if the duration has exceeded the limit
        if time.time() - self.start_time > self.duration_minutes * 60:
            rospy.loginfo("Program reached the time limit.")
            rospy.signal_shutdown("Time limit reached.")
            return

        # Convert the occupancy grid to a numpy array for easier manipulation
        map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))

        # Example: Wall Following algorithm
        left_obstacle, right_obstacle = self.detect_obstacles(self.laser_data)
        front_obstacle = self.detect_front_obstacle(self.laser_data)

        # Set velocities based on obstacle detection
        cmd_vel = Twist()
        if front_obstacle:
            # Obstacle in front, turn in place to avoid it
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5 if left_obstacle else -0.5  # Turn left if left obstacle, else turn right
            plt.annotate('Obstacle in front, turning', xy=(self.robot_position.x, self.robot_position.y), xytext=(-20, 20),
                         textcoords='offset points', arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.5"))
        elif left_obstacle and not right_obstacle:
            # Obstacle on the left, turn right
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -0.5  # Turn right
            plt.annotate('Obstacle on the left, turning right', xy=(self.robot_position.x, self.robot_position.y), xytext=(-20, 20),
                         textcoords='offset points', arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.5"))
        elif not left_obstacle and right_obstacle:
            # Obstacle on the right, turn left
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn left
            plt.annotate('Obstacle on the right, turning left', xy=(self.robot_position.x, self.robot_position.y), xytext=(-20, 20),
                         textcoords='offset points', arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.5"))
        else:
            # No obstacle, move forward
            cmd_vel.linear.x = 0.4  # Example linear velocity
            cmd_vel.angular.z = 0.0

        # Publish velocity command
        self.cmd_publisher.publish(cmd_vel)

    def detect_front_obstacle(self, laser_data):
        # Detect obstacle in front based on laser scan data
        # Example: Check if any obstacle is within a predefined range in front of the robot
        front_obstacle = any(dist < 0.5 for dist in laser_data.ranges[225:315])  # Assuming laser scan range is 360 degrees
        return front_obstacle

    def detect_obstacles(self, laser_data):
        # Detect obstacles on the left and right side based on laser scan data
        # Example: Check if any obstacle is within a predefined range on each side
        left_obstacle = any(dist < 0.5 for dist in laser_data.ranges[:len(laser_data.ranges)//2])
        right_obstacle = any(dist < 0.5 for dist in laser_data.ranges[len(laser_data.ranges)//2:])
        return left_obstacle, right_obstacle

    def plot_trajectory(self):
        plt.figure()
        plt.plot(self.x_positions, self.y_positions)
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Robot Trajectory')
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    navigation = WallFollowingNavigation()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        navigation.navigate()
        rate.sleep()
    # After shutdown, plot the trajectory
    navigation.plot_trajectory()
