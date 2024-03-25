#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker

def map_callback(msg):
    global grid_resolution, origin_x, origin_y
    grid_resolution = msg.info.resolution
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    
    map_data = msg.data
    width = msg.info.width
    height = msg.info.height
    
    points = []
    for y in range(height):
        for x in range(width):
            index = x + y * width
            if map_data[index] != -1:  # Occupied cell
                # Calculate the coordinates of the cell's center
                cell_center_x = origin_x + (x + 0.5) * grid_resolution
                cell_center_y = origin_y + (y + 0.5) * grid_resolution
                points.append(Point(x=cell_center_x, y=cell_center_y, z=0.0))
    
    # Publish the points
    publish_points(points)

def publish_points(points):
    global points_publisher
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.scale.x = 0.05  # Point size
    marker.scale.y = 0.05
    marker.color.r = 1.0
    marker.color.a = 1.0
    marker.points = points
    points_publisher.publish(marker)

def main():
    global points_publisher
    rospy.init_node('grid_points_publisher')
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    points_publisher = rospy.Publisher('/grid_points', Marker, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
