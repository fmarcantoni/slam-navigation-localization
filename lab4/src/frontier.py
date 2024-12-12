#!/usr/bin/env python3
from __future__ import annotations
from collections import deque
import math
import rospy
import copy
import cv2
import numpy as np
import subprocess
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from math import dist
import tf

import sys
import os

# Add the path to lab3/src to the Python search path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../lab3/src')))

from path_planner import PathPlanner
from queue import Queue

class Frontier:
    def __init__(self) -> None:          

        # node init       
        rospy.init_node("Frontier_Exp")

        # subscribers
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.odom = rospy.Subscriber("/odom", Odometry, self.update_odom)
        self.arrived_to_goal = rospy.Subscriber("/arrived_at_centroid", Bool, self.update_centroid)
        self.are_we_moving = rospy.Subscriber('/are_we_moving', Twist, self.update_frontiers)
        self.path_found = rospy.Subscriber("/path_found", Bool, self.update_path)

        # publishers
        self.centroid_pub = rospy.Publisher("/move_base_simple/centroid_goal", PoseStamped, queue_size=10)
        self.frontier_viz = rospy.Publisher("/frontier", GridCells, queue_size=10)
        self.empty_viz = rospy.Publisher("/empty", GridCells, queue_size=10)
        self.occupied_viz = rospy.Publisher("/occupied", GridCells, queue_size=10)
        self.unknown_viz = rospy.Publisher("/unknown", GridCells, queue_size=10)
        self.map_pub = rospy.Publisher("/map/Zeros", OccupancyGrid, queue_size=10)
        self.map_save_pub = rospy.Publisher("/map/saved", Bool, queue_size=10)
        
        # variables
        self.grid = []
        self.mapgrid = OccupancyGrid()
        self.centroids_list = []
        self.heuristic = []
        self.first_time = True # we use this to publish the centroids and frontiers for the first time 
        self.px = 0
        self.py = 0
        self.pth = 0

        self.pthQ = Quaternion()  #self.pth is a quaternion with the orientation
        self.pthQ.x = 0
        self.pthQ.y = 0
        self.pthQ.z = 1
        self.pthQ.w = 1

        self.moved_to_centroid = True
        self.is_centroid_valid = True
        self.init = True    #used to capture initial pose
        self.init_tuple = [0, 0]
        self.map_save = False #to only save the map once

        self.listener = tf.TransformListener()

        rospy.sleep(1.0)

    
    def update_path(self, msg: Bool) -> None:
        """
        Updates the path based on the validity of the centroid.
        This method is triggered when a message is received indicating whether the current centroid is valid.
        If the centroid is valid (`msg.data` is True), no action is taken.
        If the centroid is not valid (`msg.data` is False), it removes the centroid with the minimum heuristic value from the centroids list,
        updates the heuristic list, and publishes the next centroid with the minimum heuristic value.
        Args:
            msg (Bool): Message indicating whether the current centroid is valid.
        """

        print("------------------------------------------UPDATE PATH CALLED, the message (is centroid valid) is: ", msg.data)
        
        if msg.data:
            pass
            print("------------- path is found so nothing happens")
        else:
            print("------------- path is not found, pop the centroid")
            if len(self.centroids_list):
                self.centroids_list = np.delete(self.centroids_list, np.argmin(self.heuristic), axis=0) # remove the centroid with the minimum heuristic value
                self.heuristic = np.delete(self.heuristic, np.argmin(self.heuristic), axis=0) # remove the minimum heuristic value
                self.publish_centroid(self.centroids_list[np.argmin(self.heuristic)]) # publish the next centroid with the minimum heuristic value


    def update_frontiers(self, msg: Twist) -> None:
        """
        Update the list of frontiers when the robot is stationary.
        This method checks if the robot is not moving by examining the linear and angular velocities
        in the provided `Twist` message. If the robot is stationary, it processes the occupancy grid
        to detect frontiers (edges between explored and unexplored regions) and updates the list of
        frontiers. If frontiers are found, it publishes them, calculates their centroids, and selects
        a new goal centroid to navigate to. If no frontiers are detected, it logs the completion of
        exploration and saves the final map.
        Parameters
        ----------
        msg : Twist
            The current velocity command of the robot, containing linear and angular velocities.
        Returns
        -------
        None
        """

        rospy.loginfo("UPDATE_FRONTIERS GETS CALLED")
        #if we are not we update the frontiers
        if (msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and 
            msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0):

            binary_map = self.map_preprocess(self.grid)
            
            # Step 2: Gaussian smoothingource_frame o
            # laplacian = self.compute_laplacian(smoothed)
            
            # # Step 3.5: Morphological Closing
            # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, 3)
            # laplacian_closed = cv2.morphologyEx(laplacian, cv2.MORPH_CLOSE, kernel)
            
            # Step 4: Detect zero crossings
            # edges = self.detect_zero_crossings(laplacian_closed)
            edges = self.detect_zero_crossings(binary_map)

            # Step 4.1 Publish 0-crossing map
            # crossing_map = self.publish_map()
            frontiers = self.create_frontiers(edges)

            if not any(frontiers): # if we do not have any frontier
                rospy.loginfo("No frontiers detected.")
                self.save_final_map()
                #publish goal intial pose
            else:
                self.publish_frontier(frontiers)
                centroids = self.calculate_centroids(frontiers)
                self.centroids_list = centroids
                centroid = self.choose_centroid(edges, self.mapgrid, centroids, frontiers)
                print("centroid flag: ", self.moved_to_centroid)
                #if(self.moved_to_centroid and (int(centroid[0]) != 0 and int(centroid[1]) != 0)):
                if(self.moved_to_centroid and centroid is not None):
                    self.publish_centroid(centroid)
                    self.moved_to_centroid = False

    def update_odom(self, msg: Odometry) -> None:
        """
        Updates the robot's current position and orientation based on odometry data.
        This function transforms the robot's pose from the "odom" frame to the "map" frame,
        updates the robot's positional attributes, and captures the initial position if needed.
        Args:
            msg (Odometry): The odometry message containing the robot's current pose and orientation.
        Attributes Updated:
            self.px (float): The robot's x-coordinate in the map frame.
            self.py (float): The robot's y-coordinate in the map frame.
            self.pz (float): The robot's z-coordinate in the map frame.
            self.pth (float): The robot's orientation (yaw) in degrees.
            self.pthQ (Quaternion): The robot's orientation as a quaternion.
        Notes:
            - Waits for the transform between "map" and "odom" frames to be available.
            - Transforms the pose from "odom" frame to "map" frame.
            - Converts the quaternion orientation to Euler angles.
            - Captures the initial position the first time the function is called.
        """

        ps = PoseStamped()
        ps.header.frame_id = "/odom"
        ps.pose = msg.pose.pose

        self.listener.waitForTransform("/map", "/odom", rospy.Time(0), rospy.Duration(0.1))

        map_pose = self.listener.transformPose("/map", ps)

        self.px = map_pose.pose.position.x
        self.py = map_pose.pose.position.y
        self.pz = map_pose.pose.position.z

        quat_orig = map_pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = math.degrees(yaw)
        self.pthQ = quat_orig

        #capture initial poses  
        if self.init:
            self.init_tuple[0] = int(self.px)
            self.init_tuple[1] = int(self.py)
            print(" ^^^^^^^^^^^^^^^^^^^^^^^ This is when it is collecting the initial position: ", self.init_tuple[0], ", ", self.init_tuple[1])
            
            self.init = False
    
    def map_callback(self, mapdata: OccupancyGrid) -> None:        
        """
        Callback function for processing incoming map data and performing frontier detection.
        This function is triggered whenever a new `OccupancyGrid` map is received. It processes the map to detect frontiers
        (boundaries between explored and unexplored areas), calculates centroids of these frontiers, and publishes them
        for navigation. If no frontiers are detected, it saves the final map.
        Steps:
        1. Preprocess the incoming map data to create a binary map.
        2. Apply Gaussian smoothing to reduce noise.
        3. Compute the Laplacian of the smoothed map to enhance edge detection.
        4. Detect zero crossings in the Laplacian to identify frontiers.
        5. Create frontiers from the detected edges.
        6. If frontiers are found:
            - Publish the frontiers.
            - Calculate centroids of the frontiers.
            - Choose the best centroid based on certain criteria.
            - Publish the selected centroid for navigation.
        7. If no frontiers are found:
            - Save the final map.
        Parameters:
             mapdata (OccupancyGrid): The incoming occupancy grid map data.
        Returns:
             None
        """
        rospy.loginfo("map_callback function is called")
        # https://www.netlib.org/utk/lsi/pcwLSI/text/node433.html
        self.mapgrid = mapdata
        self.map_info = mapdata.info
        
        # Step 1: Preprocess map
        self.grid = np.array(mapdata.data).reshape((mapdata.info.height, mapdata.info.width))
        binary_map = self.map_preprocess(self.grid)
        
        # Step 2: Gaussian smoothing
        smoothed = self.map_smooth(binary_map)
        
        # self.visualizationMap(smoothed)

        # Step 3: Compute Laplacian
        laplacian = self.compute_laplacian(smoothed)
        
        # # Step 3.5: Morphological Closing
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, 3)
        # laplacian_closed = cv2.morphologyEx(laplacian, cv2.MORPH_CLOSE, kernel)
        
        # Step 4: Detect zero crossings
        # edges = self.detect_zero_crossings(laplacian_closed)
        edges = self.detect_zero_crossings(binary_map)

        # Step 4.1 Publish 0-crossing map
        # crossing_map = self.publish_map()
        frontiers = self.create_frontiers(edges)
        if not any(frontiers): # if we do not have any frontier
            rospy.loginfo("No frontiers detected.")
            self.save_final_map()
            #publish goal intial pose
        else:
            self.publish_frontier(frontiers)
            
            rospy.loginfo("is it the first time? ")
            rospy.loginfo(self.first_time)
            
            if self.first_time:
                rospy.loginfo("it's about to calculate the centroids...")
                centroids = self.calculate_centroids(frontiers)
                rospy.loginfo("it is about to choose the centroids")
                centroid = self.choose_centroid(edges, mapdata, centroids, frontiers)

                rospy.loginfo("centroid : ")
                rospy.loginfo(centroid)
       
        # Step 5: Choose Centroid

                if(self.moved_to_centroid and centroid is not None):
                    rospy.loginfo("centroid is going to be published")
                    self.publish_centroid(centroid)
                    self.moved_to_centroid = False
                
                self.first_time = False


    def update_centroid(self, msg: Bool) -> None:
        """
        Updates the movement status to the centroid based on the received message.
        This method sets the `moved_to_centroid` attribute to the value of `msg.data`, indicating whether the movement to the centroid has occurred.
        Args:
            msg (Bool): A message containing the movement status to the centroid.
        Returns:
            None
        """
        # https://www.netlib.org/utk/lsi/pcwLSI/text/node433.html
        self.moved_to_centroid = msg.data
    
    def map_preprocess(self, grid: np.ndarray) -> np.ndarray:
        """
        Converts an occupancy grid to a binary map for processing.
        This function transforms an input occupancy grid by mapping its values to a binary map where:
        - Unknown space (-1) is represented by **0**.
        - Occupied space (any value other than -1 or 0) is represented by **100**.
        - Free space (0) is represented by **255**.
        Args:
            grid (np.ndarray): The occupancy grid to preprocess. It contains values indicating:
                - **-1** for unknown space.
                - **0** for free space.
                - Any other value for occupied space.
        Returns:
            np.ndarray: A binary map (`np.uint8` dtype) with the same shape as the input grid, where each cell value has been replaced according to the mapping above.
        """

        bin_map = np.full(grid.shape, 100, dtype=np.uint8)  # Default to occupied space
        bin_map[grid == 0] = 255  # Free space
        bin_map[grid == -1] = 0 # unknown space
        return bin_map

    def visualizationMap(self, bin_map: np.ndarray):
        """
        Visualizes the map by identifying empty, occupied, and unknown spaces from the binary map and publishes them.
        Args:
            bin_map (np.ndarray): A binary map where each element represents a cell in the map.
                - 255 indicates empty space.
                - 0 indicates occupied space.
                - Other values indicate unknown space.
        """
        # list of tuples for empty
        emptySpace = []
        # list of tuples for full
        occupiedSpace = []
        # list of tuples of unknown
        unknownSpace = []

        for i in range(len(np.where(bin_map == 255)[0])):
            emptySpace.append([np.where(bin_map == 255)[0][i], np.where(bin_map == 255)[1][i]])

        self.publish_visualization(emptySpace, occupiedSpace, unknownSpace)

    def publish_visualization(self, emptySpace: list[tuple], occupiedSpace: list[tuple], unknownSpace: list[tuple]) -> None:
        """
        Publishes visualization messages for empty, occupied, and unknown spaces.
        This method converts the provided lists of coordinate tuples into `GridCells` messages
        and publishes them to visualize the empty, occupied, and unknown areas on the map.
        Args:
            emptySpace (list[tuple]): List of coordinate tuples representing empty cells.
            occupiedSpace (list[tuple]): List of coordinate tuples representing occupied cells.
            unknownSpace (list[tuple]): List of coordinate tuples representing unknown cells.
        Returns:
            None
        """

        empty_msg = GridCells()
        empty_msg.header.frame_id = "map"
        empty_msg.header.stamp = rospy.Time.now()
        empty_msg.cell_width = self.map_info.resolution
        empty_msg.cell_height = self.map_info.resolution

        occupied_msg = GridCells()
        occupied_msg.header.frame_id = "map"
        occupied_msg.header.stamp = rospy.Time.now()
        occupied_msg.cell_width = self.map_info.resolution
        occupied_msg.cell_height = self.map_info.resolution

        unknown_msg = GridCells()
        unknown_msg.header.frame_id = "map"
        unknown_msg.header.stamp = rospy.Time.now()
        unknown_msg.cell_width = self.map_info.resolution
        unknown_msg.cell_height = self.map_info.resolution


        # Convert tuples to world coordinates
        compiled_frontier_empty = []
        
        # for point_in_frontier in range(len(frontier)):
        for empty in emptySpace:
            world_x = self.map_info.origin.position.x + empty[0] * self.map_info.resolution
            world_y = self.map_info.origin.position.y + empty[1] * self.map_info.resolution
            world_point = Point(x = world_y, y = world_x, z = 0)
            compiled_frontier_empty.append(world_point)

        empty_msg.cells = compiled_frontier_empty
        self.empty_viz.publish(empty_msg)

    def map_smooth(self, bin_map: np.ndarray) -> np.ndarray:
        """
        Apply Gaussian smoothing to a binary map.

        Parameters:
            bin_map (np.ndarray): The input binary map to be smoothed.

        Returns:
            np.ndarray: The smoothed map.
        """
        smooth_map = cv2.GaussianBlur(bin_map, (3, 3), 0.001)  # Kernel size = 3, Sigma = 1.0
        return smooth_map
        # http://demofox.org/gauss.html

    
    def compute_laplacian(self, smooth_map: np.ndarray) -> np.ndarray:
        """
        Computes the Laplacian of a given smooth map.
        This function applies the Laplacian operator to the input smooth map using OpenCV's Laplacian function.
        The Laplacian operator is a second-order derivative operator that highlights regions of rapid intensity change.
        Args:
            smooth_map (np.ndarray): A 2D numpy array representing the smooth map.
        Returns:
            np.ndarray: A 2D numpy array representing the Laplacian of the input smooth map.
        """
        # https://docs.opencv.org/4.x/d4/d86/group__imgproc__filter.html#gad78703e4c8fe703d479c1860d76429e6
        
        laplacian = cv2.Laplacian(smooth_map, ddepth = cv2.CV_64F)
        return laplacian
    
    def detect_zero_crossings(self, binary_map: np.ndarray) -> np.ndarray:
        """
        Detect zero-crossings in a binary map.
        This function identifies the zero-crossings in a given binary map and marks them as edges.
        A zero-crossing is detected when there is a significant change in the pixel values between
        adjacent pixels.
        Parameters:
        binary_map (np.ndarray): A 2D numpy array representing the binary map.
        Returns:
        np.ndarray: A 2D numpy array of the same shape as the input, where zero-crossings are marked
                    with a value of 255 and other pixels are set to 0.
        """
        zero_crossings = np.zeros_like(binary_map, dtype=np.uint8)
        for i in range(0, zero_crossings.shape[0]):
            for j in range(0, zero_crossings.shape[1]):
                # Zero-crossing detection condition
                # if (not (laplacian[i, j] * laplacian[i + 1, j] == 0)) or (not (laplacian[i, j] * laplacian[i, j + 1] == 0)):
                if (binary_map[i, j] > 100 and binary_map[i - 1, j] < 100 or
                    binary_map[i, j] < 100 and binary_map[i - 1, j] > 100 or
                    binary_map[i, j] > 100 and binary_map[i, j - 1] < 100 or 
                    binary_map[i, j] < 100 and binary_map[i, j - 1] > 100):    

                    # if (binary_map[i, j] == 0):
                    zero_crossings[i, j] = 255  # Mark as edge

        return zero_crossings.T
    
    def create_frontiers(self, zero_crossings: np.ndarray) -> list[list[tuple]]:
        """
        Identifies and creates frontiers from the given zero crossings array.
        A frontier is a contiguous region of zero crossings that is larger than a specified size.
        Args:
            zero_crossings (np.ndarray): A 2D numpy array where zero crossings are marked with the value 255.
        Returns:
            list[list[tuple]]: A list of frontiers, where each frontier is a list of (i, j) tuples representing the coordinates of the frontier points.
        """

        frontiers = []
        # width, height = zero_crossings.shape
        # visited = np.zeros((width, height), dtype=bool)
        visited = np.zeros_like(zero_crossings, dtype=bool)
        
        frontier_points = np.argwhere(zero_crossings == 255)

        for i, j in frontier_points:
            if not visited[i, j]:
                frontier = self.bfs(i, j, zero_crossings, visited)
                # print(len(frontier))   
                if len(frontier) > 15:
                    frontiers.append(frontier)
                             
        rospy.loginfo("frontiers created:")
        rospy.loginfo(frontiers)

        return frontiers
    
    def bfs(self, start_i, start_j, zero_crossings: np.ndarray, visited: np.ndarray) -> list[tuple]:
        """
        Perform a breadth-first search (BFS) to find all connected components in a binary edge map.
        Args:
            start_i (int): The starting row index for the BFS.
            start_j (int): The starting column index for the BFS.
            zero_crossings (np.ndarray): A 2D numpy array representing the binary edge map where edges are marked with 255.
            visited (np.ndarray): A 2D numpy array of the same shape as zero_crossings to keep track of visited nodes.
        Returns:
            list[tuple]: A list of tuples where each tuple represents the coordinates (i, j) of the connected component found by BFS.
        """

        frontier = []
        search_queue = deque([(start_i, start_j)])
        visited[start_i, start_j] = True
        
        while search_queue:
            i, j = search_queue.popleft()
            frontier.append((i, j))

            # Check 8 neighbors
            neighbors = [(-1, 0), 
                         (1, 0), 
                         (0, -1), 
                         (0, 1), 
                         (-1, -1), 
                         (-1, 1), 
                         (1, -1), 
                         (1, 1)]
            for diff_i, diff_j in neighbors:
                neighbor_i, neighbor_j = i + diff_i, j + diff_j
                #If within bounds, unvisited, and 255 (a detected edge)
                if 0 <= neighbor_i < zero_crossings.shape[0] and 0 <= neighbor_j < zero_crossings.shape[1] and not visited[neighbor_i, neighbor_j] and zero_crossings[neighbor_i, neighbor_j] == 255:
                    visited[neighbor_i, neighbor_j] = True
                    search_queue.append((neighbor_i, neighbor_j))
        return frontier
    
    def calculate_centroids(self, frontier_list: list[list[tuple]]) -> list[tuple]:
        """
        Calculate the centroids of a list of frontiers.
        Args:
            frontier_list (list[list[tuple]]): A list of frontiers, where each frontier is a list of points (tuples of x, y coordinates).
        Returns:
            list[tuple]: A list of centroids, where each centroid is represented as a tuple (x, y).
        """

        centroids = []
        
        if len(frontier_list) == 0:
            rospy.loginfo("No more frontiers detected.")
            self.save_final_map()
        
        for frontier in frontier_list:
            # Calculate the centroid by averaging the x and y coordinates of the frontier points
            x_list = []
            y_list = []

            # Extract x and y coordinates separately
            for point in frontier:
                x_list.append(point[0])
                y_list.append(point[1])
            
            centroid_x = np.mean(x_list)
            centroid_y = np.mean(y_list)
            
            # centroids.append(Point(x = centroid_x, y = centroid_y, z = 0))
            centroids.append([centroid_x, centroid_y])

        rospy.loginfo("Centroids have been calculated. It is about to return them")
        return centroids

    def find_closest_walkable_cell(self, mapdata: OccupancyGrid, position: tuple[int, int]) -> tuple[int, int]:
        """
        Finds the closest free and walkable cell to the given position.

        :param mapdata: [OccupancyGrid] The map data.
        :param position: [tuple[int, int]] The starting or goal position.
        :return: [tuple[int, int]] The closest walkable cell or None if not found.
        """
        width = mapdata.info.width
        height = mapdata.info.height

        # Queue for BFS
        queue = [position]
        visited = set()
        visited.add(position)

        for current in queue:
            rospy.loginfo("STUCK :(")
            x, y = current

            # Check if the current cell is free (not an obstacle, not in C-space)
            if (
                0 <= x < width and
                0 <= y < height and
                mapdata.data[PathPlanner.grid_to_index(mapdata, current)] == 0
            ):
                return current

            # Explore neighbors (walkable or not)
            neighbors = PathPlanner.any_neighbors_of_8(mapdata, current)  # Use a method that returns all 8 neighbors
            for neighbor in neighbors:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        # Return None if no walkable cell is found
        return None

    def choose_centroid(self, zero_crossings: np.ndarray, mapdata: OccupancyGrid, centroids: list[tuple], frontiers: list[list[tuple]]) -> tuple:
        """
        Selects the most suitable centroid from a list of centroids based on their walkability, distance, and size of the frontier.
        Args:
            zero_crossings (np.ndarray): Array of zero crossings.
            mapdata (OccupancyGrid): The occupancy grid map data.
            centroids (list[tuple]): List of centroid coordinates (x, y).
            frontiers (list[list[tuple]]): List of frontiers, each frontier is a list of coordinates (x, y).
        Returns:
            tuple: The chosen centroid coordinates (x, y) or None if no walkable centroids are found.
        """

        self.grid = np.array(mapdata.data).reshape((mapdata.info.height, mapdata.info.width))
        sizes = np.array([len(f) for f in frontiers])

        centroids_array = np.array([[point[0], point[1]] for point in centroids])
        distances = np.linalg.norm(centroids_array - np.array([self.px, self.py]), axis=1)

        walkable_centroids = []
        walkable_distances = []  # Store the corresponding distances of the walkable centroids
        walkable_sizes = []  # Store the corresponding sizes of the walkable centroids

        rospy.loginfo("about to iterate through the centoirds and check if they are walkable")
        for i, centroid in enumerate(centroids_array):
            print(i)
            print(centroid)
            print(enumerate(centroids))
            centroid_x = int(centroid[0])
            centroid_y = int(centroid[1])
            centroid_truncated = (centroid_x, centroid_y)

            if PathPlanner.is_cell_walkable(mapdata, centroid_truncated):
                walkable_centroids.append(centroid_truncated)
                walkable_distances.append(distances[i])
                walkable_sizes.append(sizes[i])

            else: # if centroid is not walkable, find the closest walkable cell and return it
                new_centroid = self.find_closest_walkable_cell(self.mapgrid, centroid_truncated)
                
                if new_centroid is not None and new_centroid != (0, 0):
                    # Update centroid and calculate its distance and size
                    walkable_centroids.append(new_centroid)
                    new_distance = np.linalg.norm(np.array(new_centroid) - np.array([self.px, self.py]))
                    walkable_distances.append(new_distance)
                    
                    # Assuming the size of the frontier remains the same
                    walkable_sizes.append(sizes[i])
                else:
                    rospy.loginfo(f"Unable to find a walkable cell near centroid {centroid}")

        rospy.loginfo("it makes it out of the for loop for all the centroids")
        # Convert the filtered list back to a numpy array
        centroids = np.array(walkable_centroids)
        distances = np.array(walkable_distances)
        sizes = np.array(walkable_sizes)

        if len(centroids) == 0:
            rospy.loginfo("No walkable centroids found.")
            return None

        alpha = 0.4
        beta = 0.6
        heuristic = alpha * distances - beta * sizes
        self.heuristic = heuristic
            
        rospy.loginfo("----------------------------------------------------------------------------centroids chosen")
        return centroids[np.argmin(heuristic)]

    def publish_frontier(self, frontiers: list[list[tuple]]) -> None:
        """
        Publishes the given frontiers as GridCells messages for visualization in RViz.
        Args:
            frontiers (list[list[tuple]]): A list of frontiers, where each frontier is a list of tuples.
                                           Each tuple represents a point in the frontier with (x, y) coordinates.
        Returns:
            None
        """

        frontier_msg = GridCells()
        frontier_msg.header.frame_id = "map"
        frontier_msg.header.stamp = rospy.Time.now()
        frontier_msg.cell_width = self.map_info.resolution
        frontier_msg.cell_height = self.map_info.resolution

        # Convert tuples to world coordinates
        compiled_frontier = []
        
        # for point_in_frontier in range(len(frontier)):
    
        for frontier in frontiers:
            for point_in_frontier in frontier:
                world_x = self.map_info.origin.position.x + point_in_frontier[0] * self.map_info.resolution
                world_y = self.map_info.origin.position.y + point_in_frontier[1] * self.map_info.resolution
                world_point = Point(x = world_x, y = world_y, z = 0)
                compiled_frontier.append(world_point)

        frontier_msg.cells = compiled_frontier
        self.frontier_viz.publish(frontier_msg)
        rospy.loginfo("Frontiers have been published")

    def publish_centroid(self, centroid: tuple) -> None:
        """
        Publishes the centroid of a frontier as a PoseStamped message.
        This method converts the given centroid grid indices to world coordinates
        and publishes it as a PoseStamped message to the centroid_pub topic.
        Args:
            centroid (tuple): A tuple containing the (x, y) grid indices of the centroid.
        Returns:
            None
        """

        goal_position_msg = PoseStamped()

        # grid indices to world coordinates
        world_x = self.map_info.origin.position.x + centroid[0] * self.map_info.resolution
        world_y = self.map_info.origin.position.y + centroid[1] * self.map_info.resolution

        centroid_point = Point(x = world_x, y = world_y, z = 0)

        goal_position_msg.header.frame_id = "map"
        goal_position_msg.pose.position.x = world_x
        goal_position_msg.pose.position.y = world_y
        goal_position_msg.pose.position.z = 0
        goal_position_msg.pose.orientation = self.pthQ

        # Publish the GridCells message
        self.centroid_pub.publish(goal_position_msg)
        rospy.loginfo("------------------------ centroids published")


    def save_final_map(self):
        """
        Saves the final map to a specified location and publishes the initial position.
        This method performs the following steps:
        1. Checks if the map has already been saved.
        2. If not, saves the map to the user's home directory under the name "final_map".
        3. Logs the success or failure of the map saving process.
        4. Prints a message indicating the initial x and y coordinates.
        5. Converts the initial coordinates from world to grid coordinates.
        6. Publishes the initial position as a centroid.
        7. Sets the map_save flag to True to indicate the map has been saved.
        8. Publishes a message indicating the map has been saved.
        Raises:
            subprocess.CalledProcessError: If the map saving process fails.
        """
        
        if not self.map_save:
            map_path = os.path.expanduser("~/final_map")
            try:
                rospy.loginfo(f"Saving final map to {map_path}")
                subprocess.run(["rosrun", "map_server", "map_saver", "-f", map_path], check=True)
                rospy.loginfo("////////////////////////////////////////////////////////// Final map saved successfully. //////////////////////////////////////////////////////////")
            except subprocess.CalledProcessError as e:
                rospy.logerr(f"Failed to save map: {e}")
            
            print("@@@@@@@@@@@@@@@@@@@@@@@@@@@ we are done so we are going to the initial x and y: ", self.init_tuple[0], ", ", self.init_tuple[1])
            grid_init_point = Point()
            grid_init_point.x = self.init_tuple[0]
            grid_init_point.y = self.init_tuple[1]
            grid_init_point.z = 0
            world_initial_pose = PathPlanner.world_to_grid(self.mapgrid, grid_init_point)
            self.publish_centroid(world_initial_pose)

            self.map_save = True
            
            msg = Bool()
            msg.data = True
            self.map_save_pub.publish(msg)

    def publish_map(self, map):
        pass

    def run(self):
        rospy.spin()
   
if __name__ == '__main__':
    Frontier().run()