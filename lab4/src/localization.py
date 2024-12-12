#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Vector3
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import yaml
from PIL import Image

class Localization:
    
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node('localization_check', log_level=rospy.INFO)
        rospy.loginfo("INIT CHECK")
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.final_goal_callback)
        self.localization_ready_pub = rospy.Publisher('/localization_ready', Bool, queue_size=10)  # *****NEED LAB 2 TO SUBSCRIBE TO THIS*****
        self.final_point_pub = rospy.Publisher("/move_base_simple/localization_goal", PoseStamped, queue_size=10)
        self.final_point = PoseStamped()
        self.variance_pub = rospy.Publisher("/variance", Vector3, queue_size=10)

        self.map = self.load_yaml_map('/home/palcolea/final_map.yaml', '/home/palcolea/final_map.pgm')
        rospy.sleep(1)


    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        rospy.loginfo("AMCL Callback Responding")
        covariance = msg.pose.covariance
        position_variance = covariance[0] + covariance[7]
        orientation_variance = covariance[35]

        position_threshold = 0.03  # meters
        orientation_threshold = 0.04# radians
        var_msg = Vector3()
        var_msg.x = position_variance
        var_msg.y = orientation_variance
        var_msg.z = 0
        self.variance_pub.publish(var_msg)
        rospy.loginfo(f"Position Variance: {position_variance}, Orientation Variance: {orientation_variance}")

        if position_variance < position_threshold and orientation_variance < orientation_threshold:
            rospy.loginfo("Localization is ready, publishing True.")
            msg = Bool()
            msg.data = True
            self.localization_ready_pub.publish(msg)
        else:
            rospy.loginfo("Localization is not ready, publishing False.")
            msg = Bool()
            msg.data = False
            self.localization_ready_pub.publish(msg)

    def final_goal_callback(self, msg:PoseStamped):
        rospy.loginfo("Sending Location")
        self.final_point = msg
        self.final_point_pub.publish(self.final_point)

    def load_yaml_map(self, file_path, pgm_file_path):
        with open(file_path, 'r') as file:
            map_data = yaml.safe_load(file)
        
        frame_id = "map"  # Default frame_id
        
        # making message
        loaded_map = OccupancyGrid()
        loaded_map.header.frame_id = frame_id
        loaded_map.info.resolution = map_data.get('resolution', 0.025) #0.025 = Default
        loaded_map.info.origin.position.x = map_data['origin'][0]
        loaded_map.info.origin.position.y = map_data['origin'][1]
        loaded_map.info.origin.position.z = map_data['origin'][2]
        
        loaded_map.info.origin.orientation.x = 0.0
        loaded_map.info.origin.orientation.y = 0.0
        loaded_map.info.origin.orientation.z = 0.0
        loaded_map.info.origin.orientation.w = 1.0
        
        #PGM data
        grid_data, width, height = self.load_pgm_image(pgm_file_path)
        loaded_map.info.width = width
        loaded_map.info.height = height
        loaded_map.data = grid_data

        return loaded_map
    
    def load_pgm_image(self, pgm_file_path):
        image = Image.open(pgm_file_path)
        image = image.convert('L')  # makes sure its grayscale

        # convert the image cell list
        width, height = image.size
        data = list(image.getdata())
        return data, width, height

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    localization_instance = Localization()
    localization_instance.run()