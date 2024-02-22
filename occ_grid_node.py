#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np

import map_conversions as mc
import create_occ_grid as cog

def occ_grid_node():
    ##### YOUR CODE STARTS HERE #####
   # Initialize the ROS node
    rospy.init_node('occ_grid_node')
    rospy.loginfo("Node is starting")

    # Read map parameters from the parameter server
    boundary = rospy.get_param('~boundary')  # Format: [xmin, ymin, xmax, ymax]
    blocks = rospy.get_param('~blocks')      # Format: [[xmin, ymin, xmax, ymax], ...]
    resolution = rospy.get_param('~resolution')  # Cell size
    

    
    # Create an empty OccupancyGrid message
    grid = OccupancyGrid()

    # Fill in the header of the message
    grid.header.stamp = rospy.Time.now()
    grid.header.frame_id = 'map'

    # Fill in the metadata of the message
    grid.info.width = int((boundary[2] - boundary[0]) / resolution)  # Number of columns
    grid.info.height = int((boundary[3] - boundary[1]) / resolution)  # Number of rows
    grid.info.resolution = resolution

    grid.info.origin.position.x = boundary[0]
    grid.info.origin.position.y = boundary[1]
    grid.info.origin.position.z = 0.0

    grid.info.origin.orientation.x = 0.0
    grid.info.origin.orientation.y = 0.0
    grid.info.origin.orientation.z = 0.0
    grid.info.origin.orientation.w = 1.0

    # Fill in the data field of the message
    occupancy_grid = cog.create_occupancy_grid(boundary, blocks, resolution)

    grid.data = occupancy_grid.ravel().tolist()

    # Create a publisher to publish the occupancy grid message
    pub = rospy.Publisher('map', OccupancyGrid, latch=True)
    

    # Publish the message
    pub.publish(grid)

    rospy.loginfo("Occupancy grid published")

    # Prevent the node from exiting
    rospy.spin()

    ##### YOUR CODE ENDS HERE   #####

if __name__ == '__main__':
    try:
        occ_grid_node()
    except rospy.ROSInterruptException:
        pass


## og = OccupancyGrid()
## og.header.frame_id = rospy.get_param('frame_id')
# og.header.stano = rospy.Time.now()
# boundary = rospy.get_param('boundary')
# x0, y0, x1, y1 = boundary
# og.info.resolution = rospy.get_param('resolution')
# og.info.width = 
# og.info.height = 
