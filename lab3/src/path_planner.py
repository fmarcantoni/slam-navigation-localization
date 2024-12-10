#!/usr/bin/env python3
from __future__ import annotations

from priority_queue import PriorityQueue

import math
import rospy
import copy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class PathPlanner:


    
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        plan_path = rospy.Service("plan_path", GetPlan, self.plan_path, buff_size=65536)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.c_space = rospy.Publisher("path_planner/cspace", GridCells, queue_size=10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.expanded_cells = rospy.Publisher("path_planner/expanded_cells", GridCells, queue_size=10)
        self.frontier = rospy.Publisher("path_planner/frontier", GridCells, queue_size=10)
        self.heuristic = rospy.Publisher("path_planner/heuristic", GridCells, queue_size=10)
        self.path_viz = rospy.Publisher("path_planner/viz", GridCells, queue_size=10)
        self.actual_path_viz = rospy.Publisher("path_planner/actual_path_viz", Path, queue_size=10)


        self.are_we_moving = rospy.Publisher('/are_we_moving', Twist, queue_size=10)
        self.arrived_to_goal = rospy.Publisher("/arrived_at_centroid", Bool, queue_size=10)

        self.path_found = rospy.Publisher("/path_found", Bool, queue_size=10)


        ## Initialize the request counter
        request_counter = 0
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")

    @staticmethod
    def grid_to_index(mapdata: OccupancyGrid, p: tuple[int, int]) -> int:
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The index.
        """
        ### REQUIRED CREDIT
        
        # calculates index based on grid coords
        index = mapdata.info.width * p[1] + p[0]
        return int(index)



    @staticmethod
    def euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        ### REQUIRED CREDIT
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1]-p1[1]) ** 2)
        


    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        ### REQUIRED CREDIT
        
        # save info about map origin
        origin_x = mapdata.info.origin.position.x 
        origin_y = mapdata.info.origin.position.y

        resolution = mapdata.info.resolution

        # perform conversion

        world_x = (p[0] + 0.5) * resolution + origin_x
        world_y = (p[1] + 0.5) * resolution + origin_y

        return Point(world_x, world_y, 0)


        
    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> tuple[int, int]:
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        ### REQUIRED CREDIT
        origin_x = mapdata.info.origin.position.x
        origin_y = mapdata.info.origin.position.y

        resolution = mapdata.info.resolution

        world_x = wp.x
        world_y = wp.y

        # converts world to grid
        grid_x = (world_x - origin_x)/resolution
        grid_y = (world_y - origin_y)/resolution

        return (grid_x, grid_y)

        
    @staticmethod
    def path_to_poses(mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> list[PoseStamped]:
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        world_poses = []

        # creates a list of poses that the robot must go to get to the goal
        for coordinate in path:
            coordinate_pose = PoseStamped()
            world = PathPlanner.grid_to_world(mapdata, coordinate)

            coordinate_pose.pose.position.x = world.x
            coordinate_pose.pose.position.y = world.y
            coordinate_pose.header.frame_id = '/map'

            world_poses.append(coordinate_pose)

        return world_poses

    

    @staticmethod
    def is_cell_walkable(mapdata:OccupancyGrid, p: tuple[int, int]) -> bool:
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [bool]          True if the cell is walkable, False otherwise
        """
        ### REQUIRED CREDIT
        # get index of grid coordinate in list
        index = PathPlanner.grid_to_index(mapdata, p)
        width = mapdata.info.width
        height = mapdata.info.height

        # checks if grid is witihn boundaries of grid && free
        if not ((0 <= p[0] < width) and (0 <= p[1] < height)): 
            return False
        
        else:
            if mapdata.data[index] == 0:
                return True
            else:
                return False

    @staticmethod
    def neighbors_of_4(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        ### REQUIRED CREDIT

        can_walk = []

        # truncates for ints
        x_coordinate = int(p[0])
        y_coordinate = int(p[1])

        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate, y_coordinate + 1)): # N
            can_walk.append((x_coordinate, y_coordinate + 1))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate + 1, y_coordinate)): # E
            can_walk.append((x_coordinate + 1, y_coordinate))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate, y_coordinate - 1)): # S
            can_walk.append((x_coordinate, y_coordinate - 1))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate - 1, y_coordinate + 1)): # W
            can_walk.append((x_coordinate - 1, y_coordinate))

        return can_walk
    
    @staticmethod
    def neighbors_of_8(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT
        
        can_walk = []

        # truncates for ints
        x_coordinate = int(p[0])
        y_coordinate = int(p[1])

        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate, y_coordinate + 1)): # N
            can_walk.append((x_coordinate, y_coordinate + 1))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate + 1, y_coordinate)): # E
            can_walk.append((x_coordinate + 1, y_coordinate))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate, y_coordinate - 1)): # S
            can_walk.append((x_coordinate, y_coordinate - 1))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate - 1, y_coordinate )): # W
            can_walk.append((x_coordinate - 1, y_coordinate))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate + 1, y_coordinate + 1)): # NE
            can_walk.append((x_coordinate + 1, y_coordinate + 1))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate - 1, y_coordinate + 1)): # NW
            can_walk.append((x_coordinate - 1, y_coordinate + 1))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate + 1, y_coordinate - 1)): # SE
            can_walk.append((x_coordinate + 1, y_coordinate - 1))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate - 1, y_coordinate - 1)): # SW
            can_walk.append((x_coordinate - 1, y_coordinate - 1))

        return can_walk

    @staticmethod
    def any_neighbors_of_4(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        ### REQUIRED CREDIT

        can_walk = []

        # truncates for ints
        x_coordinate = int(p[0])
        y_coordinate = int(p[1])

    
        can_walk.append((x_coordinate, y_coordinate + 1))
    
        can_walk.append((x_coordinate + 1, y_coordinate))
    
        can_walk.append((x_coordinate, y_coordinate - 1))
    
        can_walk.append((x_coordinate - 1, y_coordinate))

        return can_walk

    @staticmethod
    def any_neighbors_of_8(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT
        
        can_walk = []

        # truncates for ints
        x_coordinate = int(p[0])
        y_coordinate = int(p[1])
    
        can_walk.append((x_coordinate, y_coordinate + 1))
    
        can_walk.append((x_coordinate + 1, y_coordinate))
    
        can_walk.append((x_coordinate, y_coordinate - 1))
    
        can_walk.append((x_coordinate - 1, y_coordinate))
    
        can_walk.append((x_coordinate + 1, y_coordinate + 1))
    
        can_walk.append((x_coordinate - 1, y_coordinate + 1))
    
        can_walk.append((x_coordinate + 1, y_coordinate - 1))
    
        can_walk.append((x_coordinate - 1, y_coordinate - 1))

        return can_walk

    @staticmethod
    def calculate_direction(start: tuple[int, int], end: tuple[int, int]) -> tuple[int, int]:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        return (dx, dy)
    

    @staticmethod
    def find_closest_walkable_cell(mapdata: OccupancyGrid, position: tuple[int, int]) -> tuple[int, int]:
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

        while queue:
            current = queue.pop(0)
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

    @staticmethod
    def find_closest_cspace_cell(mapdata: OccupancyGrid, position: tuple[int, int]) -> tuple[int, int]:
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

        while queue:
            current = queue.pop(0)
            x, y = current

            # Check if the current cell is free (not an obstacle, not in C-space)
            if (
                0 <= x < width and
                0 <= y < height and
                mapdata.data[PathPlanner.grid_to_index(mapdata, current)] == 100
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

    # @staticmethod
    # def find_nearest_cspace(mapdata: OccupancyGrid, cell: tuple[int, int]) -> tuple[int, int]:
    #     # Find the closest occupied cell in the C-space
    #     closest = None
    #     min_distance = float('inf')
    #     neighbors = PathPlanner.any_neighbors_of_8(mapdata, cell)

    #     for neighbor in neighbors:
    #         if mapdata.data[PathPlanner.grid_to_index(mapdata, neighbor)] == 100:  # C-space cell
    #             distance = PathPlanner.euclidean_distance(cell, neighbor)
    #             if distance < min_distance:
    #                 min_distance = distance
    #                 closest = neighbor
            
    #         if closest == None:
                
    #             nested_neighbors = PathPlanner.any_neighbors_of_8(mapdata, neighbor)

    #             for nested_neighbor in nested_neighbors:
    #                 if mapdata.data[PathPlanner.grid_to_index(mapdata, nested_neighbor)] == 100:  # C-space cell
    #                     distance = PathPlanner.euclidean_distance(cell, nested_neighbor)
    #                     if distance < min_distance:
    #                         min_distance = distance
    #                         closest = nested_neighbor

    #     return closest

    @staticmethod
    def penalty_for_cell_next_to_cspace(mapdata:OccupancyGrid, cell: tuple[int, int]) -> int:
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [bool]          True if the cell is not walkable, or if it's next to cspace
        """
    
        nearest_cspace = PathPlanner.find_closest_cspace_cell(mapdata, cell)
        if (nearest_cspace != None):
            distance = PathPlanner.euclidean_distance(cell, nearest_cspace)

             # Define penalty based on distance: closer to C-space, higher penalty
            if distance <= 5:  # If within 1 unit of the C-space
                return 3000  # Maximum penalty
            elif 5 < distance <= 10:  # If within 2 units of C-space
                return 1500  # Moderate penalty
            # elif distance <= 3: #if within 3 units of the C-space
            #     return 1000
            # elif distance <= 4: #if within 4 units of the C-space
            #     return 500
            else:
                return 0  # No penalty if far from C-space
            # # Define penalty based on distance: closer to C-space, higher penalty
            # if distance <= 1:  # If within 1 unit of the C-space
            #     return 3000  # Maximum penalty
            # elif distance <= 2:  # If within 2 units of C-space
            #     return 2000  # Moderate penalty
            # elif distance <= 3: #if within 3 units of the C-space
            #     return 1000
            # elif distance <= 4: #if within 4 units of the C-space
            #     return 500
            # else:
            #     return 0  # No penalty if far from C-space
        else: 
            return 0  # No penalty if far from C-space

    
    @staticmethod
    def request_map() -> OccupancyGrid:
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        req = rospy.wait_for_message("/map", OccupancyGrid, timeout = 5)
        return req



    def calc_cspace(self, mapdata: OccupancyGrid, padding: int) -> OccupancyGrid:
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
    
        # save grid info, does not change data
        grid = list(copy.deepcopy(mapdata.data))
        width = mapdata.info.width
        height = mapdata.info.height

        edited_cells = []

        # loop through cells
        for w in range(width):
            for h in range(height):    
                if mapdata.data[PathPlanner.grid_to_index(mapdata, [w,h])] == 100: # check if there is an obstacle
                    # if so, goes through nearby cells and changes them to blocked
                    for i in range (max(0, w - padding), min (width, w + padding + 1)):
                        for j in range(max(0, h - padding), min(height, h + padding + 1)):
                        # only changes unblocked cells and adds them to a list of edited cells
                            if mapdata.data[PathPlanner.grid_to_index(mapdata, [i, j])] != 100:
                                grid[PathPlanner.grid_to_index(mapdata, (i, j))] = 100
                                cell = PathPlanner.grid_to_world(mapdata, [i, j])
                                edited_cells.append(cell)

        ## Create a GridCells message and publish it
        
        grid_cells = GridCells()
        grid_cells.header.frame_id = "/map"
        grid_cells.cell_width = mapdata.info.resolution
        grid_cells.cell_height = mapdata.info.resolution
        grid_cells.cells = edited_cells
        self.c_space.publish(grid_cells)
        print("publish cspace")
        print(len(edited_cells))
        ## Return the C-space
        occupancy_grid = OccupancyGrid()
        occupancy_grid = mapdata
        occupancy_grid.data = grid
        return occupancy_grid


    
    def a_star(self, mapdata: OccupancyGrid, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        ### REQUIRED CREDIT

        # if not (PathPlanner.is_cell_walkable(mapdata, start)):

        # Validate start position
        if (
            mapdata.data[PathPlanner.grid_to_index(mapdata, start)] != 0 or
            PathPlanner.penalty_for_cell_next_to_cspace(mapdata, start) > 0
        ):
            rospy.loginfo("Start position is invalid. Searching for the closest walkable cell...")
            start = PathPlanner.find_closest_walkable_cell(mapdata, start)
            if start is None:
                rospy.loginfo("No valid start position found.")
                velocity_msg = Twist()
                velocity_msg.linear.x = 0.0
                velocity_msg.linear.y = 0.0
                velocity_msg.linear.z = 0.0
                velocity_msg.angular.x = 0.0
                velocity_msg.angular.y = 0.0
                velocity_msg.angular.z = 0.0
                self.are_we_moving.publish(velocity_msg)

                bool_msg = Bool()
                bool_msg.data = True
                self.arrived_to_goal.publish(bool_msg)
                return []

        # Validate goal position
        if (
            mapdata.data[PathPlanner.grid_to_index(mapdata, goal)] != 0 or
            PathPlanner.penalty_for_cell_next_to_cspace(mapdata, goal) > 0
        ):
            rospy.loginfo("Goal position is invalid. Searching for the closest walkable cell...")
            goal = PathPlanner.find_closest_walkable_cell(mapdata, goal)
            if goal is None:
                rospy.loginfo("No valid goal position found.")
                velocity_msg = Twist()
                velocity_msg.linear.x = 0.0
                velocity_msg.linear.y = 0.0
                velocity_msg.linear.z = 0.0
                velocity_msg.angular.x = 0.0
                velocity_msg.angular.y = 0.0
                velocity_msg.angular.z = 0.0
                self.are_we_moving.publish(velocity_msg)

                bool_msg = Bool()
                bool_msg.data = True
                self.arrived_to_goal.publish(bool_msg)
                return []

        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))

        truncated_start = (int(start[0]), int(start[1]))
        truncated_goal = (int(goal[0]), int(goal[1]))
        frontier = PriorityQueue()
        frontier.put(truncated_start, 0)
        came_from = {}  
        cost_so_far = {}    
        came_from[truncated_start] = None
        cost_so_far[truncated_start] = 0

        # if mapdata.data[truncated_start[1] * mapdata.info.width + truncated_start[0]] == 100:
        #     rospy.loginfo("Start position is an obstacle.")
        #     return []

        # runs until goal is found or frontier is fully explored
        while not frontier.empty():
            grid_cells = GridCells()
            grid_cells.header.frame_id = "/map"
            grid_cells.cell_width = mapdata.info.resolution
            grid_cells.cell_height = mapdata.info.resolution
            current = frontier.get()
            # print("---------------- current: ", current)
            # print("---------------- truncated_goal: ", truncated_goal)
            if current == truncated_goal:
                print("---------------------------------------------------reached goal")
                break

            for next in PathPlanner.neighbors_of_8(mapdata, current):
                penalty = PathPlanner.penalty_for_cell_next_to_cspace(mapdata, next)  # This will return the varying penalty

                if came_from[current] is not None:
                    previous_direction = PathPlanner.calculate_direction(came_from[current], current)
                    current_direction = PathPlanner.calculate_direction(current, next)
                    if previous_direction != current_direction:
                        penalty += 0.01 # add a small penalty when we have a change in direction

                new_cost = cost_so_far[current] + PathPlanner.euclidean_distance(current, next) + penalty


                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

            # gets new frontier and publishes it
            actual_frontier = []
            for f in frontier.get_queue():
                actual_frontier.append(f[1])
            grid_cells.cells = [PathPlanner.grid_to_world(mapdata, f) for f in actual_frontier]
            self.frontier.publish(grid_cells)

        # gets path from dictionary
        path = []
        # current = truncated_goal
        if current == truncated_goal:
            bool_msg = Bool()
            bool_msg.data = True
            self.path_found.publish(bool_msg)
            # print("current : ", current)
            # print("came from:")
            # print(came_from)
            # print("current_camefrom: ", came_from.get(current))
            while current != truncated_start:
                path.append(current)
                current = came_from.get(current)
                # print("current : ", current)
                # print("truncated start : ", truncated_start)
        else:
            rospy.loginfo("-----------------------------------------------------------could not find path to frontier")
            velocity_msg = Twist()
            velocity_msg.linear.x = 0.0
            velocity_msg.linear.y = 0.0
            velocity_msg.linear.z = 0.0
            velocity_msg.angular.x = 0.0
            velocity_msg.angular.y = 0.0
            velocity_msg.angular.z = 0.0
            self.are_we_moving.publish(velocity_msg)

            bool_msg = Bool()
            bool_msg.data = True
            self.arrived_to_goal.publish(bool_msg)

            path_found_msg = Bool()
            path_found_msg.data = False
            self.path_found.publish(bool_msg)

        
        path.reverse()

        grid_cells = GridCells()
        grid_cells.header.frame_id = "/map"
        grid_cells.cell_width = mapdata.info.resolution
        grid_cells.cell_height = mapdata.info.resolution
        grid_cells.cells = [PathPlanner.grid_to_world(mapdata, p) for p in path]
        self.path_viz.publish(grid_cells)
        rospy.loginfo("-----------------------------------------------------------------publish path")

        return path

    
    @staticmethod
    def optimize_path(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        if len(path) == 0:
            return []
        
        new_path = []
        new_path.append(path[0])
        for i in range (1, len(path) - 1): 
            previous = path[i-1]
            current = path[i]
            next = path[i+1]
            dx1 = current[0] - previous[0]
            dy1 = current[1] - previous[1]
            dx2 = next[0] - current[0]
            dy2 = next[1] - current[1]

            if dx1 - dx2 != dy1 - dy2:
                new_path.append(current)

        new_path.append(path[-1])

        return new_path

    # @staticmethod
    # def shorten_path(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    #     new_path_length = len(path)
    #     shorter_path = []

    #     for i in range(int(new_path_length/2)):
    #         shorter_path.append(path[i])

    #     return shorter_path
        

    def path_to_message(self, mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> Path:
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")
        path_message = Path()
        path_message.header.frame_id = "/map"

        path_message.poses = PathPlanner.path_to_poses(mapdata, path)
        self.actual_path_viz.publish(path_message)
        rospy.loginfo("path published")

        return path_message


        
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 5)
        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        ## Optimize waypoints
        # waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        rospy.loginfo("---------------------- plan_path")
        return self.path_to_message(mapdata, path)


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        map = self.request_map()
        #self.calc_cspace(map, 4)
        
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
