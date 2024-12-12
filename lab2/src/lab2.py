#!/usr/bin/env python3
from __future__ import annotations
import rospy
import math
import angles

import tf
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion

from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan


class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2')
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        # create a publisher to send a boolean whenever the robot has reached the end of a path
        self.arrived_to_goal = rospy.Publisher("/arrived_at_centroid", Bool, queue_size=10)
        ## Create a publisher for the point that the robot is following in pure pursuit
        ## The topic is "/pointToFollow", the message type is pointStamped
        self.pointFollowing = rospy.Publisher("/pointToFollow", PointStamped, queue_size=10)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        #rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)
        self.are_we_moving = rospy.Publisher('/are_we_moving', Twist, queue_size=10)
        #Subscriber that gets paths that PathPlannercalculates
        rospy.Subscriber("/path_planner/actual_path_viz", Path, self.go_to_destination)


        #for avoiding wall collisions
        # self.scan_laser = rospy.Subscriber("/scan", LaserScan, self.backoff_wall)
        self.saved_map_sub = rospy.Subscriber("/map/saved", Bool, self.update_going_home)

        #init attributes
        self.px = 0
        self.py = 0
        self.pth = 0

        # pth in Quaternion
        self.pthQ = Quaternion()
        self.pthQ.x = 0
        self.pthQ.y = 0
        self.pthQ.z = 1
        self.pthQ.w = 1

        # for transofrmations
        self.listener = tf.TransformListener()

        self.lastFoundIndex = 0     #this is for finding intersections
        self.lookAhead = 0.10
        self.Kp_turn = 0.01
        self.Kp_lin = 0.2

        self.going_home = False

        #flag that dictates if we have been given a destination or not
        self.givenDestination = False

        #stores the path coordinates that gets called in run() every time that a new destination is given
        self.pathCoordinates = []

        #trying to aliviate case races
        rospy.sleep(1.0)

    def update_going_home(self, msg: Bool):
        """
        callback function for going_home flag.
        :gets a message containing a Boolean. Frontier.py sends this when it is ready to send the robot home after the whole map has been saved.
        """
        self.going_home = msg.data

    def backoff_wall(self, msg: LaserScan) -> None:
        """
        Checks if the robot is facing a wall that is too close.
        If so, backs off for a short time and stops.
        :param message with the laser scan
        """
        front_angle = 0                     # Assuming 0 radians corresponds to the front of the robot
        angle_range = math.pi / 4           # Angular range (in radians) to consider the "front". 
    
        # Calculate indices in the LaserScan ranges array for the front
        start_angle = front_angle - angle_range / 2
        end_angle = front_angle + angle_range / 2

        # Determine the corresponding indices in the LaserScan ranges
        start_index = int((start_angle - msg.angle_min) / msg.angle_increment)
        end_index = int((end_angle - msg.angle_min) / msg.angle_increment)

        # Clip indices to stay within the bounds of the ranges array
        start_index = max(0, start_index)
        end_index = min(len(msg.ranges) - 1, end_index)

        # Check if any reading in the front range is too close
        for i in range(start_index, end_index + 1):
            if msg.ranges[i] > 0 and msg.ranges[i] <= 0.2:  # Threshold distance in meters
                rospy.loginfo("Wall detected in front, backing off...")
                self.send_speed(0.0, 0.0)                   # Stop the robot
                rospy.sleep(0.5)
                self.send_speed(-0.1, 0)                    # Backward speed
                rospy.sleep(2)                              # Move backward for 2 seconds
                self.send_speed(0.0, 0.0)                   # Stop the robot

                return

    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
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

    def go_to_destination(self, msg: Path):
        """
        Callback function to subscriber. 
        This runs every time that a path is sent from path_planner.py
        :path that the robot needs to follow
        """

        print("New Destination Received")

        coordinatesInPath = []
        # print all the coordinates of the path
        print("(x, y): ")

        #iterates through all the poses in the path and puts into a list of tuples the x and y coordinates
        for pose in msg.poses:
            print(pose.pose.position.x, ", ", pose.pose.position.y)
            coordinatesInPath.append([pose.pose.position.x, pose.pose.position.y])
        # print("coordinates in path: ", coordinatesInPath)
        
        # stops the robot, it will start moving when the pure pursuit tells it to
        self.send_speed(0.0, 0.0)

        #flag variable to turn on the goal point search
        self.givenDestination = True
        self.lastFoundIndex = 0
        self.pathCoordinates.clear()

        # puts all the coordinates into the global variable to be used by a different method when it is needed.
        for i in range(0, len(coordinatesInPath)):
            self.pathCoordinates.append(coordinatesInPath[i])

    def update_odometry(self, msg: Odometry) -> None:
        """
        Updates the odometry from the robot.
        """

        ps = PoseStamped()
        ps.header.frame_id = "/odom"
        ps.pose = msg.pose.pose

        self.listener.waitForTransform("/map", "/odom", rospy.Time(0), rospy.Duration(1.0))
        
        #transforms the pose
        map_pose = self.listener.transformPose("/map", ps)

        self.px = map_pose.pose.position.x
        self.py = map_pose.pose.position.y
        self.pz = map_pose.pose.position.z

        quat_orig = map_pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = math.degrees(yaw)
        self.pthQ = quat_orig

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

    ##########################################################################################
    ################### FROM HERE ON IS THE IMPLEMENTATION OF PURE PURSUIT ###################
    ##########################################################################################
    
    def sgn(self, num: float):
        """
        Helper method needed to find interesctions of the path in the circle of lookahead distance.
        :param num         input of the function
        :returns -1 or 1
        """
        if num >= 0:
            return 1
        else:
            return -1

    def distance_points(self, point1, point2) -> float:
        """

        Helper method that calculates the euclidean distance between two points
        :param point1, and point2, tuples
        :returns a float of the euclidean distance
        """
        distance = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
        return distance

    def path_interesections(self, p1: tuple[float, float], p2: tuple[float, float]) -> tuple[float, float]:
        """
        detect the point to follow for pure pursuit, math based from Purdue's implementation
        :param p1 [float, float]        first point of the path
        :param p2 [float, float]        second point of the path
        :param lookupDistance           radius of the circle that the robot considers to follow points
        :returns            tuple of point to follow
        :                   return None if the path doesn't intersect the circle
        """
        l = self.lookAhead

        x1 = p1[0]
        x2 = p2[0]
        y1 = p1[1]
        y2 = p2[1]
        
        ###################################################################################################print()
        ###################################################################################################print()
        ###################################################################################################print("--------------------- Finding the interesections -------------")
        ###################################################################################################print("(x1, y1): ", p1)
        ###################################################################################################print("(x2, y2): ", p2)
        
        # adjust to center p1 and p2 on the robot
        x1_adjusted = x1 - self.px
        x2_adjusted = x2 - self.px
        y1_adjusted = y1 - self.py
        y2_adjusted = y2 - self.py

        ###################################################################################################print("current x, current y", self.px, self.py)
        
        # some intermediate variables
        dx = x2_adjusted - x1_adjusted
        dy = y2_adjusted - y1_adjusted
        dr = math.sqrt(dx**2 + dy**2)
        D = x1_adjusted*y2_adjusted - x2_adjusted*y1_adjusted

        # The incidence is a parameter that tells how many times the circle intercepts the line of the two points
        incidence = (l**2) * (dr**2) - D**2
        ################################################################################################### print("lookahead distance: ", l)
        ################################################################################################### print("incidence: ", incidence)

        #depending on the incidence, there will be different number of solutions
        #when there's only one solution, go to it (unless it is going backgwards)
        #have to chose the right solution if there are two
        if incidence > 0:       # there are two solutions, but we only want to return the closest
            
            # calculate the Xs and the Yx of the two points that intercect the circle
            # these equations come from Purdue's implementation
            X1 = ((D*dy) + self.sgn(dy)*dx*math.sqrt(incidence))/(dr**2)
            X2 = ((D*dy) - self.sgn(dy)*dx*math.sqrt(incidence))/(dr**2)

            Y1 = ((-D*dy) + abs(dy)*math.sqrt(incidence))/(dr**2)
            Y2 = ((-D*dy) - abs(dy)*math.sqrt(incidence))/(dr**2)

            # arrange back to remove the offset
            X1 = X1 + self.px
            X2 = X2 + self.px
            Y1 = Y1 + self.py
            Y2 = Y2 + self.py

            # calculate distance between the interesction point and p2
            distance1 = math.sqrt((x2 - X1)**2 + (y2 - Y1)**2)
            distance2 = math.sqrt((x2 - X2)**2 + (y2 - Y2)**2)
            
            # return only if it is within range, if not, check which one is closest
            # tis makes sure that the solution given are allowed. Make sure that the robot doesn't move backwards
            if not (min(x1, x2)<= X1 <= max(x1, x2)) and (min(y1, y2)<= Y1 <= max(y1, y2)):
                if not (min(x1, x2)<= X2 <= max(x1, x2)) and (min(y1, y2)<= Y2 <= max(y1, y2)):
                    return None
                else:
                    return [X2, Y2]
            elif not (min(x1, x2)<= X2 <= max(x1, x2)) and (min(y1, y2)<= Y2 <= max(y1, y2)):
                return [X1, Y1]
            else:
                if distance1 < distance2:
                    return [X1, Y1]
                else:
                    return [X2, Y2]

        elif incidence == 0:    # there is only one solution
            X1 = (D*dy)/(dr**2)
            Y1 = (-D*dy)/(dr**2)
            return [X1, Y1]
        
        else:                   # there are no solutions
            return None

    def find_min_angle(self, absTargetAngle, currentHeading) -> float:
        """
        find the angle needed to go from current heading to goal heading
        :param p1 [float, float]        first point of the path
        :param p2 [float, float]        second point of the path
        :param lookupDistance           radius of the circle that the robot considers to follow points
        :returns            tuple of point to follow
        :                   return None if the path doesn't intersect the circle
        """

        minAngle = absTargetAngle - currentHeading

        if minAngle > 180 or minAngle < -180 :
            minAngle = -1 * self.sgn(minAngle) * (360 - abs(minAngle))

        return minAngle
    
    def move_robot(self, target: tuple[float, float]):
        """
        Methods that tells the motors how much to move to aim in the right direction
                this method will call on send_speed
        :param target         tuple with the points that the robot should aim for
        """

        #print("Moving Robot to %f, %f" % (target[0], target[1]))
        #get the current position of the robot from the global variables
        #store the target x and target y from the input to the method
        rospy.loginfo("move_robot called")
        targetx = target[0]
        targety = target[1]
        currentx = self.px
        currenty = self.py

        # linear error is the euclidean distance from one current point to the target
        linearError = self.distance_points([currentx, currenty], target)

        # calculate the heading that the robot needs to take to be pointing towards the target point
        targetHeading = math.degrees(math.atan2(targety - currenty, targetx - currentx))
        if targetHeading < 0:
            targetHeading += 360
        
        # with the current heading from the odometry data, calculate the turning error (difference in angles)
        currentHeading = self.pth
        turnError = self.find_min_angle(targetHeading, currentHeading)

        # once you have angular error and linear error
        # Proportional Controller for linearSpeed and angularSpeed
        # the proportional constants are global variables that can be altered and fine tuned
        linearVel = self.Kp_lin * linearError
        turnVel = self.Kp_turn * turnError

        # tells the robot to move the wheels accordingly to what is necessary to reach the new target goal
        self.send_speed(linearVel, turnVel)

    def goal_pt_search(self, path: list[tuple[float, float]]):
        """
            this function is called constantly and udpates the last Found index if the robot passes that point in order to move forward
            :param path that is stored.
                   the path it gets from path_planner.py and put into a global function by callback function go_to_destination
        """
        
        ###################################################################################################print("Goal point searching...")

        #check if the robot has reached its final destination to stop
        
        #act like it reached the end if the path is empty
        #a_star will return an empty path to the topic that path is gotten from if a path is not possible or if it reaches the destination
        if not path:
            print("The list is empty.")
            self.givenDestination = False
            self.send_speed(0.0, 0.0)
            return

        # takes the last element of the path
        finalPosition = path[-1]
        potentiallyFollow = finalPosition

        # if the distance to the final point of the path from the position is less than 0.2. 
        # It has reached the final destination and it can stop and set everything back to 0
        if self.distance_points(finalPosition, [self.px, self.py]) < 0.2:

            velocity_msg = Twist()
            velocity_msg.linear.x = 0.0
            velocity_msg.linear.y = 0.0
            velocity_msg.linear.z = 0.0
            velocity_msg.angular.x = 0.0
            velocity_msg.angular.y = 0.0
            velocity_msg.angular.z = 0.0
            self.are_we_moving.publish(velocity_msg)

            # puublishe in a topic that the robot has reached the destination
            msg = Bool()
            msg.data = True
            self.arrived_to_goal.publish(msg)

            print("Has reached the destination!")
            print(finalPosition)
            self.send_speed(0.0, 0.0)
            self.givenDestination = False
            return 


        # loop through whatever is left of the path
        for i in range (self.lastFoundIndex, len(path)-1):
            # pointLastFoundIndex = [msg.poses[self.lastFoundIndex].pose.position.x, msg.poses[self.lastFoundIndex].pose.position.y]
            # pointLastFoundIndex1 = [msg.poses[self.lastFoundIndex + 1].pose.position.x, msg.poses[self.lastFoundIndex + 1].pose.position.y]
            
            # calculate the interesction
            # potentially follow is the point that the robot should follow next
            potentiallyFollow = self.path_interesections(path[self.lastFoundIndex], path[self.lastFoundIndex + 1])
            ###################################################################################################print("potentially follow: ", potentiallyFollow)
            
            #if there is no more point to follow, it must be reaching the end (tell it to aim for the last point of the path)
            if potentiallyFollow == None:
                potentiallyFollow = path[self.lastFoundIndex]


            # if the robot is closer to the next index than the goal, change index
            # this prevents the robot from moving backwards
            P2 = path[self.lastFoundIndex + 1]
            if self.distance_points(P2, potentiallyFollow) > self.distance_points([self.px, self.py], P2):
                self.lastFoundIndex = i + 1
            else:
                self.lastFoundIndex = i
                break

        #puts the point that needs to be followed into Point() form as opposed from a tuple with only x and y
        followPoint = PointStamped()
        followPoint.header.frame_id = "/odom"
        followPoint.point.x = potentiallyFollow[0]
        followPoint.point.y = potentiallyFollow[1]
        followPoint.point.z = 0  

        # physically moves the robot to that point
        self.move_robot(potentiallyFollow)
    
    ##########################################################################################
    ########################## This is the end of the pure pursuit. ##########################
    ##########################################################################################

    def run(self):
        while True:
            #only do goal point search if given a destination
            if self.givenDestination:
                self.goal_pt_search(self.pathCoordinates)

            # # so that it stops if it gets home
            # if self.going_home and not self.givenDestination:
            #     break
        # rospy.spin()

if __name__ == '__main__':
    Lab2().run()