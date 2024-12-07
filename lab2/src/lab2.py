#!/usr/bin/env python3
from __future__ import annotations
import rospy
import math
import angles
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
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
        ## Create a publisher for the point that the robot is following in pure pursuit
        ## The topic is "/pointToFollow", the message type is pointStamped
        self.pointFollowing = rospy.Publisher("/pointToFollow", PointStamped, queue_size=10)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        #rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)

        rospy.Subscriber("/path_planner/actual_path_viz", Path, self.go_to_destination)
        rospy.Subscriber("/move_base_simple/localization_goal", PoseStamped, self.local_move)
        rospy.Subscriber("/localization_ready", Bool, self.readyCallback)

        #init attributes
        self.px = 0
        self.py = 0
        self.pth = 0
        self.lastFoundIndex = 0     #this is for finding intersections
        self.lookAhead = 0.3
        self.Kp_turn = 0.05
        self.Kp_lin = 0.8

        self.givenDestination = False
        self.oldTime = 0.0
        self.pathCoordinates = []
        self.ready = False

    def readyCallback(self, msg:Bool):
        ready = msg.data

    def local_move(self, msg:PoseStamped):
        if ready:
            go_to_Pure(msg)
        else:
            rotate(90, 1)

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
        print("New Destination Received")
        

        coordinatesInPath = []
        # print all the coordinates of the path
        print("(x, y): ")
        for pose in msg.poses:
            print(pose.pose.position.x, ", ", pose.pose.position.y)
            coordinatesInPath.append([pose.pose.position.x, pose.pose.position.y])
        # print("coordinates in path: ", coordinatesInPath)
        
        list_of_locations = []
        list_of_locations = msg.poses
        self.send_speed(0.0, 0.0)

        # # runs through this list of location and for every location (PoseStamped), call go_to on it
        # for location in list_of_locations:
        #     print("we are going to the next pose")
        #     #self.go_to(location)
        #     self.go_to(location)
        

        #flag variable to turn on the goal point search
        self.givenDestination = True
        self.lastFoundIndex = 0
        self.pathCoordinates.clear()

        for i in range(0, len(coordinatesInPath)):
            self.pathCoordinates.append(coordinatesInPath[i])

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
        self.pth = math.degrees(yaw)


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

        incidence = (l**2) * (dr**2) - D**2
        ################################################################################################### print("lookahead distance: ", l)
        ################################################################################################### print("incidence: ", incidence)

        if incidence > 0:       # there are two solutions, but we only want to return the closest
            # calculate the Xs and the Yx of the two points that intercect the circle
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
        ###################################################################################################print("Moving Robot to %f, %f" % (target[0], target[1]))
        targetx = target[0]
        targety = target[1]
        currentx = self.px
        currenty = self.py

        # linear error is the distance from one current point to the target
        linearError = self.distance_points([currentx, currenty], target)

        targetHeading = math.degrees(math.atan2(targety - currenty, targetx - currentx))
        if targetHeading < 0:
            targetHeading += 360

        currentHeading = self.pth
        turnError = self.find_min_angle(targetHeading, currentHeading)

        # once you have angular error and linear error
        # if error is possitive, then robot should turn counterclockwise
        #   leftVel = linearVel - turnVel and rightVel = linearVel + turnVel
        # Proportional Controller for linearSpeed and angularSpeed
        linearVel = self.Kp_lin * linearError
        turnVel = self.Kp_turn * turnError

        self.send_speed(linearVel, turnVel)

    def goal_pt_search(self, path: list[tuple[float, float]]):
        """
            this function is called constantly and udpates the last Found index if the robot passes that point in order to move forward

        """
        
        ###################################################################################################print("Goal point searching...")


        #check if the robot has reached its final destination to stop
        
        #act like it reached the end if the path is empty
        if not path:
            print("The list is empty.")
            self.givenDestination = False
            self.send_speed(0.0, 0.0)
            return

        finalPosition = path[-1]
        potentiallyFollow = finalPosition
        if self.distance_points(finalPosition, [self.px, self.py]) < self.lookAhead:
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
            potentiallyFollow = self.path_interesections(path[self.lastFoundIndex], path[self.lastFoundIndex + 1])
            ###################################################################################################print("potentially follow: ", potentiallyFollow)
            if potentiallyFollow == None:
                potentiallyFollow = path[self.lastFoundIndex]


            # if the robot is closer to the next index than the goal, change index
            P2 = path[self.lastFoundIndex + 1]
            if self.distance_points(P2, potentiallyFollow) > self.distance_points([self.px, self.py], P2):
                self.lastFoundIndex = i + 1
            else:
                self.lastFoundIndex = i
                break

        followPoint = PointStamped()
        followPoint.header.frame_id = "/odom"
        followPoint.point.x = potentiallyFollow[0]
        followPoint.point.y = potentiallyFollow[1]
        followPoint.point.z = 0  
  
        # if ((rospy.get_time() - self.oldTime) > 1):
        #     self.oldTime = rospy.get_time()
        #     self.pointFollowing.publish(followPoint)

        # print(potentiallyFollow)

        # self.move_robot([-0.34999, 1.28])
        self.move_robot(potentiallyFollow)
    

    ##########################################################################################
    ########################## This is the end of the pure pursuit. ##########################
    ##########################################################################################



    def run(self):
        while True:
            #only do goal point search if given a destination
            if self.givenDestination:
                self.goal_pt_search(self.pathCoordinates)
        # rospy.spin()

if __name__ == '__main__':
    Lab2().run()