#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np

import map_conversions as mc
import create_occ_grid as cog

def env_to_occ_grid():
    ##### YOUR CODE STARTS HERE #####
   # Initialize the ROS node
    rospy.init_node('occ_grid_node')
    rospy.loginfo("Node is starting")

    # Read map parameters from the parameter server
    boundary = rospy.get_param('~boundary')  # Format: [xmin, ymin, xmax, ymax]
    blocks = rospy.get_param('~blocks')      # Format: [[xmin, ymin, xmax, ymax], ...]
    resolution = rospy.get_param('~resolution')  # Cell size
    
    # Create an empty OccupancyGrid message
    occ_grid_msg = OccupancyGrid()

    # Fill in the header of the message
    occ_grid_msg.header.stamp = rospy.Time.now()
    occ_grid_msg.header.frame_id = 'map'

    # Fill in the metadata of the message
    occ_grid_msg.info.width = int((boundary[2] - boundary[0]) / resolution)  # Number of columns
    occ_grid_msg.info.height = int((boundary[3] - boundary[1]) / resolution)  # Number of rows
    occ_grid_msg.info.resolution = resolution
    occ_grid_msg.info.origin.position.x = boundary[0]
    occ_grid_msg.info.origin.position.y = boundary[1]
    occ_grid_msg.info.origin.position.z = 0.0
    occ_grid_msg.info.origin.orientation.x = 0.0
    occ_grid_msg.info.origin.orientation.y = 0.0
    occ_grid_msg.info.origin.orientation.z = 0.0
    occ_grid_msg.info.origin.orientation.w = 1.0

    # Create the occupancy grid using the provided boundary, blocks, and resolution
    occupancy_grid = cog.create_occupancy_grid(boundary, blocks, resolution)

    # Fill in the data field of the message
    occ_grid_msg.data = occupancy_grid.ravel().tolist()

    # Create a publisher to publish the occupancy grid message
    pub = rospy.Publisher('map', OccupancyGrid, latch=True)

    # Publish the message
    pub.publish(occ_grid_msg)

    rospy.loginfo("Occupancy grid published")

    # Prevent the node from exiting
    rospy.spin()

    ##### YOUR CODE ENDS HERE   #####

if __name__ == '__main__':
    try:
        env_to_occ_grid()
    except rospy.ROSInterruptException:
        pass

