#!/usr/bin/env python3
from __future__ import annotations
from queue import PriorityQueue

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool

#!/usr/bin/env pythons
class Lab3_Listener:
    
    def __init__(self):
        """
        Class constructor for the initialization of the Lab3_Listener node.
        """
        rospy.init_node('Lab3_Listener')

        # Create Subscribers in order to activate the service and to update the odometry values.
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        rospy.Subscriber('/move_base_simple/centroid_goal', PoseStamped, self.activate_service)
        rospy.Subscriber('/move_base_simple/localization_goal', PoseStamped, self.activate_service_local)
        rospy.Subscriber("/localization_ready", Bool, self.readyCallback)
        self.ready = False
        self.local_goal = PoseStamped()

        # attributes
        Lab3_Listener.px = 0
        Lab3_Listener.py = 0
        Lab3_Listener.quart = 0
    
    def readyCallback(self, msg:Bool):
        self.ready = msg.data
        if self.ready:
            self.activate_service_local(self.local_goal)

    def activate_service(self, msg: PoseStamped):
        """
        Activates the plan_path serviceactivate_service)

        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The goal pose information.
        """
        plan_path = rospy.ServiceProxy('plan_path', GetPlan)

        rospy.loginfo("Service got the centroid. It's about to pass it to path planner node")
        
        start = PoseStamped()
        start.pose.position.x = self.px
        start.pose.position.y = self.py
        start.pose.position.z = 0
        start.pose.orientation = self.quart

        goal = msg
        rospy.wait_for_service('plan_path')
        plan_path(start, goal, 1)
    
    def activate_service_local(self, msg: PoseStamped):
        self.local_goal = msg
        if self.ready:
            plan_path = rospy.ServiceProxy('plan_path', GetPlan)

            print("hello! service should be active")
            
            start = PoseStamped()
            start.pose.position.x = self.px
            start.pose.position.y = self.py
            start.pose.position.z = 0
            start.pose.orientation = self.quart

            plan_path(start, self.local_goal, 1)

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