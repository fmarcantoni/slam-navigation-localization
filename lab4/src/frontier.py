#!/usr/bin/env python3
from __future__ import annotations
from collections import deque

# from priority_queue import PriorityQueue

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

# from lab3.src.path_planner import PathPlanner


class Frontier:
    def __init__(self) -> None:                 
        rospy.init_node("Frontier_Exp")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.odom = rospy.Subscriber("/odom", Odometry, self.update_odom)
        self.arrived_to_goal = rospy.Subscriber("/arrived_at_centroid", Bool, self.update_centroid)
        self.are_we_moving = rospy.Subscriber('/are_we_moving', Twist, self.update_frontiers)
        self.path_found = rospy.Subscriber("/path_found", Bool, self.update_path)

        self.centroid_pub = rospy.Publisher("/move_base_simple/centroid_goal", PoseStamped, queue_size=10)
        self.frontier_viz = rospy.Publisher("/frontier", GridCells, queue_size=10)
        self.empty_viz = rospy.Publisher("/empty", GridCells, queue_size=10)
        self.occupied_viz = rospy.Publisher("/occupied", GridCells, queue_size=10)
        self.unknown_viz = rospy.Publisher("/unknown", GridCells, queue_size=10)
        self.map_pub = rospy.Publisher("/map/Zeros", OccupancyGrid, queue_size=10)
        self.map_save_pub = rospy.Publisher("/map/saved", Bool, queue_size=10)
        
        # self.frontier_markers_viz = rospy.Publisher("/frontier_markers", MarkerArray, queue_size=10)
        self.grid = []
        self.mapgrid = OccupancyGrid()

        self.centroids_list = []
        self.heuristic = []
        self.first_time = True #we use this to publish the centroids and frontiers for the first time 

        self.px = 0
        self.py = 0
        self.pth = 0

        #self.pth is a quaternion with the orientation
        self.pthQ = Quaternion()
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
        print("------------------------------------------UPDATE PATH CALLED, the message (is centroid valid) is: ", msg.data)
        
        if msg.data:
            pass
            print("------------- path is found so nothing happens")
        else:
            print("------------- path is not found, pop the centroid")
            if len(self.centroids_list):
                self.centroids_list = np.delete(self.centroids_list, np.argmin(self.heuristic), axis=0)
                self.heuristic = np.delete(self.heuristic, np.argmin(self.heuristic), axis=0)
                self.publish_centroid(self.centroids_list[np.argmin(self.heuristic)])


    def update_frontiers(self, msg: Twist) -> None:
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
                # heuristic_list = self.choose_centroid(edges, self.mapgrid, centroids, frontiers)
                # self.heuristic = heuristic_list

                # print("centroid flag: ", self.moved_to_centroid)
                # if(self.moved_to_centroid):
                #     self.publish_centroid(centroids[np.argmin(heuristic_list)])
                #     self.moved_to_centroid = False

                print("centroid flag: ", self.moved_to_centroid)
                #if(self.moved_to_centroid and (int(centroid[0]) != 0 and int(centroid[1]) != 0)):
                if(self.moved_to_centroid and centroid is not None):
                    self.publish_centroid(centroid)
                    self.moved_to_centroid = False


                #update the frontiers
                #then we check if we have any more frontiers, if we do not we save the map

    def update_odom(self, msg: Odometry) -> None:
        # px = msg.pose.pose.position.x
        # py = msg.pose.pose.position.y
        # # print("x: ", self.px, " , y: ", self.py)
        
        # quat_orig = msg.pose.pose.orientation
        # quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        # (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        # pth = math.degrees(yaw)
        # pthQ = quat_orig

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
        # # Step 5: Choose Centroid

        #     print("centroid flag: ", self.moved_to_centroid)
        #     #if(self.moved_to_centroid and (int(centroid[0]) != 0 and int(centroid[1]) != 0)):
                if(self.moved_to_centroid and centroid is not None):
                    rospy.loginfo("centroid is going to be published")
                    self.publish_centroid(centroid)
                    self.moved_to_centroid = False
                
                self.first_time = False

    # def check_for_frontiers(self, mapdata: OccupancyGrid, bin_map: np.ndarray) -> Bool:
    #     if ()
    #     edges = self.detect_zero_crossings(bin_map)

    #     centroid = self.choose_centroid(edges, mapdata)

    #     print("centroid flag: ", self.moved_to_centroid)
    #     if(self.moved_to_centroid and centroid is not None):
    #         self.publish_centroid(centroid)
    #         self.moved_to_centroid = False


    #     if(centroid is None and )


    def update_centroid(self, msg: Bool) -> None:
        # https://www.netlib.org/utk/lsi/pcwLSI/text/node433.html
        self.moved_to_centroid = msg.data
    
    def map_preprocess(self, grid: np.ndarray) -> np.ndarray:
        bin_map = np.full(grid.shape, 100, dtype=np.uint8)  # Default to occupied space
        bin_map[grid == 0] = 255  # Free space
        bin_map[grid == -1] = 0 # unknown space
        return bin_map

    def visualizationMap(self, bin_map: np.ndarray):
        # list of tuples for empty
        emptySpace = []
        # list of tuples for full
        occupiedSpace = []
        # list of tuples of unknown
        unknownSpace = []

        for i in range(len(np.where(bin_map == 255)[0])):
            emptySpace.append([np.where(bin_map == 255)[0][i], np.where(bin_map == 255)[1][i]])

        # for j in range(len(np.where(bin_map == 0)[0])):
        #     occupiedSpace.append([np.where(bin_map == 0)[0][j], np.where(bin_map == 0)[1][j]])

        # for k in range(len(np.where(bin_map == 127)[0])):
        #     unknownSpace.append([np.where(bin_map == 127)[0][k], np.where(bin_map == 127)[1][k]])

        self.publish_visualization(emptySpace, occupiedSpace, unknownSpace)
        # emptySpace.append(np.where(bin_map == 0))
        # occupiedSpace.append(np.where(bin_map == 255))
        # unknownSpace.append(np.where(bin_map == 127))
        # print(emptySpace)

    def publish_visualization(self, emptySpace: list[tuple], occupiedSpace: list[tuple], unknownSpace: list[tuple]) -> None:
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
        smooth_map = cv2.GaussianBlur(bin_map, (3, 3), 0.001)  # Kernel size = 3, Sigma = 1.0
        return smooth_map
        
        
        # http://demofox.org/gauss.html
        # Sigma = 1.0, Support = 0.4
        # getGaussianKernel(int ksize, sigma, ktype = CV_32F)
        # kernel = np.array([
        #     [0.0038, 0.0150, 0.0238, 0.0150, 0.0038],
        #     [0.0150, 0.0599, 0.0949, 0.0599, 0.0150],
        #     [0.0238, 0.0949, 0.1503, 0.0949, 0.0238],
        #     [0.0150, 0.0599, 0.0949, 0.0599, 0.0150],
        #     [0.0038, 0.0150, 0.0238, 0.0150, 0.0038]], dtype = np.float32)
        
        # smooth_map = cv2.filter2D(bin_map, -1, kernel)
        # return smooth_map
    
    def compute_laplacian(self, smooth_map: np.ndarray) -> np.ndarray:
        # https://docs.opencv.org/4.x/d4/d86/group__imgproc__filter.html#gad78703e4c8fe703d479c1860d76429e6
        
        laplacian = cv2.Laplacian(smooth_map, ddepth = cv2.CV_64F)
        return laplacian
    
    def detect_zero_crossings(self, binary_map: np.ndarray) -> np.ndarray:
        zero_crossings = np.zeros_like(binary_map, dtype=np.uint8)
        # kernel = np.array([[1, -1], [-1, 1]], dtype=np.float32)
        # crossings_map = cv2.filter2D(laplacian, -1, kernel)
        # zero_crossings[np.abs(crossings_map) > 0] = 255
        # return zero_crossings
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

    # def closest_walkable_cell(self, grid: np.ndarray, start: tuple[int, int], tolerance: int) -> tuple[int, int]:
    #     """
    #     Find the closest walkable cell to the starting cell using BFS, checking all 8 neighbors, 
    #     while ensuring that the chosen cell is the furthest from the C-space but closest to the original illegal centroid.

    #     :param grid: A 2D numpy array representing the occupancy grid. 
    #                 0 represents a free cell, 1 represents an obstacle.
    #     :param start: A tuple (x, y) indicating the starting cell (the centroid).
    #     :param tolerance: The tolerance distance (half the diameter of the robot) for the offset from the centroid.
    #     :return: A tuple (x, y) of the closest walkable cell, or None if no walkable cell is found.
    #     """
    #     rows, cols = grid.shape
    #     visited = set()
    #     queue = deque([start])
        
    #     # Directions for moving in 8-connectivity (including diagonals)
    #     directions = [
    #         (-1, 0), (1, 0), (0, -1), (0, 1),  # Cardinal directions
    #         (-1, -1), (-1, 1), (1, -1), (1, 1)  # Diagonal directions
    #     ]

    #     best_cell = None
    #     best_distance = float('inf')  # To store the best distance from the centroid
    #     best_offset = -float('inf')  # To store the best offset from C-space

    #     while queue:
    #         current = queue.popleft()
    #         x, y = current

    #         # Check if the cell is within bounds and not visited
    #         if (x, y) in visited or not (0 <= x < rows and 0 <= y < cols):
    #             continue

    #         visited.add((x, y))
            
    #         # Check if the current cell is walkable
    #         if grid[x, y] == 0:
    #             # Calculate the Euclidean distance to the centroid
    #             distance_to_centroid = dist((x, y), start)
                
    #             # Check if we meet the offset criteria
    #             offset_from_c_space = distance_to_centroid - tolerance
                
    #             # We prioritize the cell with the furthest offset but closest to the original centroid
    #             if offset_from_c_space > best_offset or (offset_from_c_space == best_offset and distance_to_centroid < best_distance):
    #                 best_cell = (x, y)
    #                 best_distance = distance_to_centroid
    #                 best_offset = offset_from_c_space
            
    #         # Add neighboring cells to the queue
    #         for dx, dy in directions:
    #             neighbor = (x + dx, y + dy)
    #             if neighbor not in visited:
    #                 queue.append(neighbor)
        
    #     # Return the best cell found
    #     return best_cell

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
            # current = queue.pop(0)
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
        # frontiers = self.create_frontiers(zero_crossings)
        # self.publish_frontier(frontiers)

        # if not any(frontiers):
        #     rospy.loginfo("No frontiers detected.")
        #     self.save_final_map()
            #sedn the bot to initial position PHASE 2
        # else:

        
        # centroids = np.array([
        #     [np.mean([p[0] for p in frontier]), np.mean([p[1] for p in frontier])] for frontier in frontiers])

        self.grid = np.array(mapdata.data).reshape((mapdata.info.height, mapdata.info.width))

        # centroids = self.calculate_centroids(frontiers)
        
        sizes = np.array([len(f) for f in frontiers])
    
        # Vectorized distance calculation
        #distances = np.linalg.norm(centroids - np.array([self.px, self.py]), axis=1)

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
            # centroid_x = int(centroid[0])
            # centroid_y = int(centroid[1])
            centroid_x = int(centroid[0])
            centroid_y = int(centroid[1])
            centroid_truncated = (centroid_x, centroid_y)

            #rospy.loginfo
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


        # if not self.is_centroid_valid:
        #     print("------------- path is not found, pop the centroid")
        #     centroids = np.delete(centroids, np.argmin(heuristic), axis=0)
        #gonna return heuristic
            
        rospy.loginfo("----------------------------------------------------------------------------centroids chosen")
        return centroids[np.argmin(heuristic)]

    def publish_frontier(self, frontiers: list[list[tuple]]) -> None:
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
        goal_position_msg = PoseStamped()
        # goal_position.header = rospy.Time.now()
        # goal_position.header.stamp = rospy.Time.now()

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