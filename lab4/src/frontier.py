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
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion


class Frontier:
    def __init__(self) -> None:                 
        rospy.init_node("Frontier_Exp")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.odom = rospy.Subscriber("/odom", Odometry, self.update_odom)
        self.centroid_pub = rospy.Publisher("/move_base_simple/centroid_goal", PoseStamped, queue_size=10)
        self.frontier_viz = rospy.Publisher("/frontier", GridCells, queue_size=10)
        self.empty_viz = rospy.Publisher("/empty", GridCells, queue_size=10)
        self.occupied_viz = rospy.Publisher("/occupied", GridCells, queue_size=10)
        self.unknown_viz = rospy.Publisher("/unknown", GridCells, queue_size=10)

        self.map_pub = rospy.Publisher("/map/Zeros", OccupancyGrid, queue_size=10)
        # self.frontier_markers_viz = rospy.Publisher("/frontier_markers", MarkerArray, queue_size=10)
        self.px = 0
        self.py = 0
        self.pth = 0

        #self.pth is a quaternion with the orientation
        self.pthQ = Quaternion()
        self.pthQ.x = 0
        self.pthQ.y = 0
        self.pthQ.z = 1
        self.pthQ.w = 1
    
    def update_odom(self, msg: Odometry) -> None:
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = math.degrees(yaw)
        self.pthQ = quat_orig
    
    def map_callback(self, mapdata: OccupancyGrid) -> None:
        # https://www.netlib.org/utk/lsi/pcwLSI/text/node433.html
        
        self.map_info = mapdata.info
        
        # Step 1: Preprocess map
        grid = np.array(mapdata.data).reshape((mapdata.info.height, mapdata.info.width))
        binary_map = self.map_preprocess(grid)
        
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


        # Step 5: Choose Centroid
        self.choose_centroid(edges)
        
    
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
        for i in range(1, zero_crossings.shape[0]-1):
            for j in range(1, zero_crossings.shape[1]-1):
                # Zero-crossing detection condition
                # if (not (laplacian[i, j] * laplacian[i + 1, j] == 0)) or (not (laplacian[i, j] * laplacian[i, j + 1] == 0)):
                if (binary_map[i, j] > 100 and binary_map[i - 1, j] < 100 or
                    binary_map[i, j] < 100 and binary_map[i - 1, j] > 100 or
                    binary_map[i, j] > 100 and binary_map[i, j - 1] < 100 or 
                    binary_map[i, j] < 100 and binary_map[i, j - 1] > 100):    

                    # if (binary_map[i, j] == 0):
                    zero_crossings[j, i] = 255  # Mark as edge
        return zero_crossings
    
    def create_frontiers(self, zero_crossings: np.ndarray) -> list[list[tuple]]:
        frontiers = []
        # width, height = zero_crossings.shape
        # visited = np.zeros((width, height), dtype=bool)
        visited = np.zeros_like(zero_crossings, dtype=bool)
        
        frontier_points = np.argwhere(zero_crossings == 255)

        for i, j in frontier_points:
            if not visited[i, j]:
                frontier = self.bfs(i, j, zero_crossings, visited)
                print(len(frontier))   
                if len(frontier) > 5:
                    frontiers.append(frontier)
                             

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
            
            centroids.append(Point(x = centroid_x, y = centroid_y, z = 0))
        return centroids

    def choose_centroid(self, zero_crossings: np.ndarray):
        frontiers = self.create_frontiers(zero_crossings)
        self.publish_frontier(frontiers)
        if not frontiers:
            rospy.loginfo("No frontiers detected.")
            return
    
        centroids = np.array([
            [np.mean([p[0] for p in frontier]), np.mean([p[1] for p in frontier])] for frontier in frontiers])
        
        sizes = np.array([len(f) for f in frontiers])
    
        # Vectorized distance calculation
        distances = np.linalg.norm(centroids - np.array([self.px, self.py]), axis=1)
    
        alpha = 1
        beta = 3
        heuristic = alpha * distances - beta * sizes
        
        # Step 6: Publish Centroid
        self.publish_centroid(centroids[np.argmin(heuristic)])

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


    def publish_centroid(self, centroid: Point) -> None:
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

    def save_final_map():
        map_path = "~/final_map"
        try:
            rospy.loginfo(f"Saving final map to {map_path}")
            subprocess.run(["rosrun", "map_server", "map_saver", "-f", map_path], check=True)
            rospy.loginfo("Final map saved successfully.")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to save map: {e}")

    def publish_map(self, map):
        pass

    def run(self):
        rospy.spin()
   
if __name__ == '__main__':
    Frontier().run()