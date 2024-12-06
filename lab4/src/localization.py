#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool


class Localization:
    
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node('localization_check')
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.localization_ready_pub = rospy.Publisher('/localization_ready', Bool, queue_size=10)

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        covariance = msg.pose.covariance
        position_variance = covariance[0] + covariance[7]
        orientation_variance = covariance[35]

        position_threshold = 0.1  # meters
        orientation_threshold = 0.1  # radians

        if position_variance < position_threshold and orientation_variance < orientation_threshold:
            self.localization_ready_pub.publish(True)
        else:
            self.localization_ready_pub.publish(False)

    def run():
        rospy.spin()

if __name__=='__main__':
    Localization.run()
