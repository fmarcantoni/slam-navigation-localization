#!/usr/bin/env python3

import rospy
import math
import angles
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2')
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        #rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)

        rospy.Subscriber("/path_planner/actual_path_viz", Path, self.go_to_destination)

        #init attributes
        self.px = 0
        self.py = 0
        self.pth = 0



    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        msg_cmd_vel = Twist()

        # linear velocity
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0

        # angular velocity
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed

        ### Publish the message
        self.cmd_vel.publish(msg_cmd_vel)
        r = rospy.Rate(10) # 10hz
        r.sleep()

        
    def drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        ### REQUIRED CREDIT
        
        # get and save current position
        self.send_speed(0.0, 0.0)
        init_x = self.px
        init_y = self.py

        current_distance = 0

        # keep running until reach target distance
        while distance - current_distance > 0.01:
            self.send_speed(linear_speed, 0.0)
            current_x = self.px
            current_y = self.py

            # calculate distance travelled
            current_distance = math.sqrt((self.px - init_x) ** 2 + (self.py - init_y) ** 2)

            rospy.sleep(0.05)

        self.send_speed(0.0, 0.0)


    def rotate(self, angle: float, aspeed: float):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT

        self.send_speed(0.0, 0.0)

        init_pth = self.pth + math.pi

        target_heading = init_pth + angle

        
        # checks if the current heading is greater that 2pi. If it is, corrects it to be less than 2pi ...
        if target_heading > 2*math.pi:
            while target_heading > 2*math.pi:
                target_heading = target_heading - 2*math.pi
        elif target_heading < 0:
            while target_heading < 0:
                target_heading = target_heading + 2*math.pi


        current_heading = 0

        while abs(target_heading - current_heading) > 0.1:
            self.send_speed(0.0, aspeed)
            current_heading = self.pth + math.pi
            rospy.sleep(0.05)

        self.send_speed(0.0,0.0)


    def go_to(self, msg: PoseStamped):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        ### REQUIRED CREDIT

        # stores the initial pose
        self.send_speed(0.0, 0.0)
        current_x = self.px
        current_y = self.py
        current_heading = self.pth

        # extract final pose of the robot
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        target_heading = yaw

        # find the first heading we are going to rotate to (then drive in straight line)

        first_heading = math.atan2((target_y - current_y),(target_x - current_x))
        
        first_angle = first_heading - current_heading

        if first_heading > current_heading:
            if first_angle < math.pi:
                rotate_speed = 1
            else:
                rotate_speed = -1
        elif first_heading < current_heading:
            if first_angle < -1*math.pi:
                rotate_speed = 1
            else: 
                rotate_speed = -1

        self.rotate(first_angle, rotate_speed)

        travel_distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

        self.drive(travel_distance, 0.1)

        final_angle = target_heading - self.pth

        if target_heading > current_heading:
            if final_angle < math.pi:
                rotate_speed = 0.5
            else:
                rotate_speed = -0.5
        elif target_heading < current_heading:
            if final_angle < -1*math.pi:
                rotate_speed = 0.5
            else: 
                rotate_speed = -0.5

        self.rotate(final_angle, rotate_speed)

    def go_to_pure(self, msg: PoseStamped):
         # stores the initial pose
        #self.send_speed(1.0, 0.0)
        current_x = self.px
        current_y = self.py
        current_heading = self.pth

        # extract final pose of the robot
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        target_heading = yaw

        kp_linear = 0.1
        kp_turn = 0.1
        print("target_x - current_x")
        print( target_x - current_x )

        print("target_y - current_y")
        print( target_y - current_y )

        print("arget_heading - current_heading")
        print(target_heading - current_heading)

        #while (abs(target_x - current_x) >= 0.02) and (abs(target_y - current_y) >= 0.02) and (abs(target_heading - current_heading) >= 0.02):
        while True:
             
            current_x = self.px
            current_y = self.py
            current_heading = self.pth

            linear_error = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            absTargetAngle = math.atan2(target_y - current_y, target_x - current_x)
            absPoseAngle = target_heading - current_heading

            # if absTargetAngle < -1*math.pi:
            #     absTargetAngle += 2 * math.pi
            # elif absTargetAngle > math.pi:
            #     absTargetAngle -= 2 * math.pi

            # if absPoseAngle < -1*math.pi:
            #     absPoseAngle += 2 * math.pi
            # elif absPoseAngle > math.pi:
            #     absPoseAngle -= 2 * math.pi


            if absTargetAngle < 0:
                absTargetAngle += 2 * math.pi
            if absPoseAngle < 0:
                absPoseAngle += 2 * math.pi

            # absTargetAngle %= (2 * math.pi)
            # absPoseAngle %= (2 * math.pi)

            

            alpha = min(1, linear_error)

            angular_error = alpha * absTargetAngle + (1-alpha) * absPoseAngle

            print("linear and turn errors: ")
            print(linear_error)
            print(" ")
            print(angular_error)

            #linearVel = min(linear_error*kp_linear, 10)
            linearVel = linear_error*kp_linear
            angularVel = angular_error*kp_turn

            print("Linear velocity: ")
            print(linearVel)

            print("angular velocity: ")
            print(angularVel)

            self.send_speed(linearVel, angularVel)

            if (abs(target_x - current_x) <= 0.02) and (abs(target_y - current_y) <= 0.02) and (abs(target_heading - current_heading) <= 0.02):
                break



        """
        //compute linear error
        linearVel = sqrt(pow(tergety - currenty, 2) + pow(tergetx - currentx, 2))

        //compute turn error
        absTargetAngle = atan2(targety - currenty, targetx - currentx)
        if absTargetAngle < 0 : absTargetAngle += 360
        turnError = find_min_angle(absTargetAngle, currentHeading)

        //compute linear and turn velocities using controller of your choice

        //send command to motors
        """

    def go_to_destination(self, msg: Path):
        print("hello")
        list_of_locations = []
        list_of_locations = msg.poses
        self.send_speed(0.0, 0.0)

        # runs through this list of location and for every location (PoseStamped), call go_to on it
        for location in list_of_locations:
            print("we are going to the next pose")
            #self.go_to(location)
            self.go_to_pure(location)

    def update_odometry(self, msg: Odometry):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]

        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw

    def smooth_drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        self.send_speed(0.0, 0.0)
        init_x = self.px
        init_y = self.py
        kp = 0.5 # some kp, adjust with testing as needed
        max_speed = 10 # some max speed, adjust with testing as needed

        current_distance = 0

        # keeps running until reach target distance
        while distance - current_distance > 0.1:
            # proportional control
            error = distance - current_distance
            motor_effort = kp * error
            if motor_effort > max_speed:
                motor_effort = max_speed
            self.send_speed(motor_effort, 0.0)
            current_x = self.px
            current_y = self.py

            # calculate current distance travelled
            current_distance = math.sqrt((self.px - init_x) ** 2 + (self.py - init_y) ** 2)

            rospy.sleep(0.05)

        # stop the robot
        self.send_speed(0.0, 0.0)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    Lab2().run()