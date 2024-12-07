#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool


class local_movement:
    
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node('local_movment')
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.r)        
        self.map


    def amcl_callback(self, msg: PoseWithCovarianceStamped):

    
    @staticmethod
    def request_map() -> OccupancyGrid:
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        rospy.loginfo("Requesting the map")
        req = rospy.wait_for_message("/map", OccupancyGrid, timeout = 5)
        return req

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        self.map = self.request_map()
        rospy.spin()

        
if __name__ == '__main__':
    local_movement.run()