#!/usr/bin/env python3
from __future__ import annotations
from queue import PriorityQueue

import math
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
import tf

#!/usr/bin/env pythons
class Lab3_Listener:
    
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node('Lab3_Listener')
        rospy.Subscriber('/odom', Odometry, self.update_odom)
        rospy.Subscriber('/move_base_simple/centroid_goal', PoseStamped, self.activate_service)
        rospy.Subscriber('/move_base_simple/localization_goal', PoseStamped, self.activate_service_local)
        rospy.Subscriber("/localization_ready", Bool, self.readyCallback)
        self.ready = False
        self.local_goal = PoseStamped()
        
        self.listener = tf.TransformListener()

        # attributes
        Lab3_Listener.px = 0
        Lab3_Listener.py = 0
        Lab3_Listener.quart = 0

        self.pthQ = Quaternion()
        self.pthQ.x = 0
        self.pthQ.y = 0
        self.pthQ.z = 1
        self.pthQ.w = 1

    def update_odom(self, msg: Odometry) -> None:
        ps = PoseStamped()
        ps.header.frame_id = "/odom"
        ps.pose = msg.pose.pose

        self.listener.waitForTransform("/map", "/odom", rospy.Time(0), rospy.Duration(0.1))

        map_pose = self.listener.transformPose("/map", ps)

        self.px = map_pose.pose.position.x
        self.py = map_pose.pose.position.y
        self.pz = map_pose.pose.position.z

        quat_origin = map_pose.pose.orientation
        quat_list = [quat_origin.x, quat_origin.y, quat_origin.z, quat_origin.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = math.degrees(yaw)
        self.pthQ = quat_origin
    
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

        print("hello! service should be active")
        
        start = PoseStamped()
        start.pose.position.x = self.px
        start.pose.position.y = self.py
        start.pose.position.z = 0
        start.pose.orientation = self.quart

        goal = msg

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