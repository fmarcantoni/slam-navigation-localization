#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import yaml


class Localization:
    
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node('localization_check')
        # Pass the callback function properly using self.<callback_name>
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.final_goal_callback)  # Fixed this line
        self.localization_ready_pub = rospy.Publisher('/localization_ready', Bool, queue_size=10)  # *****NEED LAB 2 TO SUBSCRIBE TO THIS*****
        self.final_point_pub = rospy.Publisher("/move_base_simple/localization_goal", PoseStamped, queue_size=10)

        self.map = self.load_yaml_map('/home/opvancampen/catkin_ws/src/RBE3002_B24_Team02/lab3/maps/simple_map.yaml')
        self.local_complete = False  # Initialize local_complete

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        covariance = msg.pose.covariance
        position_variance = covariance[0] + covariance[7]
        orientation_variance = covariance[35]

        position_threshold = 0.05  # meters
        orientation_threshold = 0.1  # radians

        if position_variance < position_threshold and orientation_variance < orientation_threshold:
            msg = Bool()
            msg.data = True
            self.local_complete = True
            self.localization_ready_pub.publish(msg)

    def final_goal_callback(self, msg: PoseStamped):
        self.final_point_pub.publish(msg)

    def load_yaml_map(self, file_path):
        with open(file_path, 'r') as file:
            map_data = yaml.safe_load(file)
        
        loaded_map = OccupancyGrid()
        loaded_map.header.frame_id = map_data['header']['frame_id']
        loaded_map.info.resolution = map_data['resolution']
        loaded_map.info.width = map_data['width']
        loaded_map.info.height = map_data['height']
        loaded_map.info.origin.position.x = map_data['origin'][0]
        loaded_map.info.origin.position.y = map_data['origin'][1]
        loaded_map.info.origin.position.z = map_data['origin'][2]
        loaded_map.info.origin.orientation.x = map_data['origin'][3]
        loaded_map.info.origin.orientation.y = map_data['origin'][4]
        loaded_map.info.origin.orientation.z = map_data['origin'][5]
        loaded_map.info.origin.orientation.w = map_data['origin'][6]
        loaded_map.data = map_data['data']
        return loaded_map

    def run(self):
        rospy.spin()  # Keeps the node running


if __name__ == '__main__':
    Localization.run()  # Call the run method on the instance
