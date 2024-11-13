#!/usr/bin/env python3
from __future__ import annotations
from queue import PriorityQueue

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf.transformations import euler_from_quaternion

#!/usr/bin/env pythons
class Lab3_Listener:
    
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node('Lab3_Listener')
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.activate_service)

        # attributes
        Lab3_Listener.px = 0
        Lab3_Listener.py = 0
        Lab3_Listener.quart = 0

    def activate_service(self, msg: PoseStamped):
        """
        Activates the plan_path service
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The goal pose information.
        """
        plan_path = rospy.ServiceProxy('plan_path', GetPlan)
        
        start = PoseStamped()
        start.pose.position.x = self.px
        start.pose.position.y = self.py
        start.pose.position.z = 0
        start.pose.orientation = self.quart

        goal = msg

        plan_path(start, goal, 1)

    def update_odometry(self, msg: Odometry):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
    
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.quart = msg.pose.pose.orientation
        quat_orig = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w])
        self.pth = yaw

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    Lab3_Listener().run()