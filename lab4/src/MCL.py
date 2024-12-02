#!/usr/bin/env python3
from __future__ import annotations
from collections import deque

from priority_queue import PriorityQueue

import math
import rospy
import copy
import cv2
import numpy as np
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist
from tf.transformations import euler_from_quaternion


class MCL:
    def __init__(self) -> None:
        rospy.init_node(MCL)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.control_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.x_i = np.array([0.0, 
                              0.0, 
                              0.0]) # initial position: x=0, y=0, theta=0
        self.sigma = np.zeros((3,3))
        self.u_t = np.array([0.0,
                             0.0]) #initial velocities: v=0, omega=0
        self.z_t = 0.1
        self.m = OccupancyGrid()
        
        
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Convert quaternion to euler angles
        roll, pitch, theta = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        
        self.mu_t = np.array([position.x, position.y, theta])
    
    def control_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        self.u_t = np.array([v, omega])
        
    def run(self):
        self.make_prediction()
        rospy.spin()


if __name__ == '__main__':
    MCL().run()    