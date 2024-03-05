#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import time

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
        self.grid_resolution = 0.05  # Adjust as needed based on your map resolution
        self.grid_size = (100, 100)  # Adjust the grid size based on your map dimensions
        self.grid = np.full(self.grid_size, np.inf)  # Initialize grid with high initial values
        self.start_cell = None
        self.goal_cell = None
        self.path = None
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

        # Create the grid representation of the maze
        self.create_grid(map_array)

        # Perform waterfilling algorithm
        self.waterfill()

        # Plan path from start to goal
        self.plan_path()

        # Navigate the robot
        self.follow_path()

    def create_grid(self, map_array):
        # Convert occupancy grid to grid representation
        # Assuming each cell in the grid represents an area of size grid_resolution x grid_resolution
        # Iterate over the map and mark occupied cells as obstacles in the grid
        for i in range(self.grid_size[0]):
            for j in range(self.grid_size[1]):
                if map_array[i, j] == 100:  # Occupied cell
                    self.grid[i, j] = np.inf  # Mark as obstacle
                else:
                    self.grid[i, j] = 0  # Mark as empty cell

        # Set start and goal cells (adjust as needed based on your map)
        self.start_cell = (0, 0)  # Example: Start at top-left corner
        self.goal_cell = (self.grid_size[0] - 1, self.grid_size[1] - 1)  # Example: Goal at bottom-right corner

    def waterfill(self):
        # Perform waterfilling algorithm
        # Initialize queue with the start cell
        queue = [self.start_cell]
        visited = set([self.start_cell])

        while queue:
            cell = queue.pop(0)
            neighbors = self.get_neighbors(cell)

            for neighbor in neighbors:
                if neighbor not in visited:
                    # Increment value of neighbor cell
                    self.grid[neighbor] = self.grid[cell] + 1
                    visited.add(neighbor)
                    queue.append(neighbor)

    def get_neighbors(self, cell):
        # Get neighboring cells
        neighbors = []
        x, y = cell

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                new_x = x + dx
                new_y = y + dy

                if 0 <= new_x < self.grid_size[0] and 0 <= new_y < self.grid_size[1]:
                    neighbors.append((new_x, new_y))

        return neighbors

    def plan_path(self):
        # Plan path from start to goal using flood-filled values
        self.path = [self.start_cell]
        current_cell = self.start_cell

        while current_cell != self.goal_cell:
            neighbors = self.get_neighbors(current_cell)
            next_cell = min(neighbors, key=lambda x: self.grid[x])
            self.path.append(next_cell)
            current_cell = next_cell

    def follow_path(self):
        # Navigate the robot along the planned path
        # For simplicity, just move straight to each cell in the path without obstacle avoidance
        cmd_vel = Twist()

        if self.path:
            next_cell = self.path.pop(0)
            target_x = next_cell[0] * self.grid_resolution
            target_y = next_cell[1] * self.grid_resolution
            current_x = self.robot_position.x
            current_y = self.robot_position.y

            distance = np.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

            if distance < 0.1:  # Threshold distance to consider reaching the target cell
                # Move to the next cell in the path
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
            else:
                # Move towards the target cell
                cmd_vel.linear.x = 0.2  # Example linear velocity
                cmd_vel.angular.z = 0.0  # Example angular velocity

        # Publish velocity command
        self.cmd_publisher.publish(cmd_vel)

if __name__ == '__main__':
    navigation = WaterfillNavigation()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        navigation.navigate()
        rate.sleep()
