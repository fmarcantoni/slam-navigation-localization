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
    """
        PathPlanner Class calculates the c-space identifying walkable and non-walkable space, 
        along with planning the optimal path using the A* algorithm.
    """

    def __init__(self):
        """
        Class constructor: Intilaizes the "path_planner" node, the "plan_path" service and the Publishers.
        """
        ## Initialize the node with name "path_planner"
        rospy.init_node("path_planner")

        ## Create a new service called "plan_path" that accepts messages of type GetPlan and calls self.plan_path() when a message is received
        plan_path = rospy.Service("plan_path", GetPlan, self.plan_path, buff_size=65536)

        ## Create publishers for A* (expanded cells, frontier, heuristic and vizualization) of type GridCells
        self.expanded_cells = rospy.Publisher("path_planner/expanded_cells", GridCells, queue_size=10)
        self.frontier = rospy.Publisher("path_planner/frontier", GridCells, queue_size=10)
        self.heuristic = rospy.Publisher("path_planner/heuristic", GridCells, queue_size=10)
        self.path_viz = rospy.Publisher("path_planner/viz", GridCells, queue_size=10)

        ## Create a publisher for the C-space (the enlarged occupancy grid) with topic "/path_planner/cspace" of type GridCells
        self.c_space = rospy.Publisher("path_planner/cspace", GridCells, queue_size=10)

        ## Create a publisher for the distance_penalty map (which sets the path penalties based on distances from obstacles) with topic "/distances_penalty" of type OccupancyGrid
        self.penalty = rospy.Publisher("/distances_penalty", OccupancyGrid, queue_size=10)

        ## Create a publisher for the "path_planner/actual_path_viz" topic which is used to visulaize on Rviz the path calculated by the A* algorithm.
        self.actual_path_viz = rospy.Publisher("path_planner/actual_path_viz", Path, queue_size=10)

        ## Create publisher for "/are_we_moving" which is pubilish a velocity as a msg type Twist, when the robot is not moving;
        ## "/path_found" which publish True when A* is able to calculate a path, publish False otherwise.
        self.are_we_moving = rospy.Publisher("/are_we_moving", Twist, queue_size=10)
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
        
        # calculates index based on grid coordinates
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
        ## returns the euclidean distance between two tuples
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1]-p1[1]) ** 2)
        
    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        
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
        Returns the any 4-neighbors cells of (x,y) in the occupancy grid. They can be walkable or obstacles.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of the 4-neighbors.
        """

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
        Returns the any 8-neighbors cells of (x,y) in the occupancy grid. They can be walkable or obstacles.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of the 8-neighbors.
        """
        
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
        """
        Calculates the direction in x and y going from one cell to another one.
        :param p       [(int, int)]    The start cell coordinate in the grid.
        :param p       [(int, int)]    The end cell coordinate in the grid.
        :return        [[(int,int)]]   Change in direction in x and y
        """
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

    def wavefront_distance(self, mapdata: OccupancyGrid) -> np.ndarray:
        """
        Creates a distance_map to add penalties in the A* algorithm to avoid finding a path close to the walls.
        :param mapdata: [OccupancyGrid] The map data.
        :return: [np.ndarray] The distance_map as a np.ndarray.
        """
        # Get the dimensions of the grid
        width = mapdata.info.width
        height = mapdata.info.height

        # Convert OccupancyGrid to np.ndarray.
        map_array = np.array(mapdata.data).reshape((mapdata.info.height, mapdata.info.width))
        
        # Initialize the distance grid with a high value
        distance_grid = np.full((height, width), 999)

        # Find the cells that are not walkable. 
        occupied_cells = np.argwhere(map_array == 100)

        if occupied_cells.size == 0:
            return distance_grid
        
        # Create a queue for BFS
        queue = []

        for (j, i) in occupied_cells:
            distance_grid[j, i] = 0
            queue.append((j, i))

        while queue:
            j, i = queue.pop(0)
            for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                ny, nx = j + dy, i + dx
                if 0 <= ny < height and 0 <= nx < width:
                    if distance_grid[ny, nx] > distance_grid[j, i] + 1:
                        if map_array[ny, nx] > -1:
                            distance_grid[ny, nx] = distance_grid[j, i] + 1
                            queue.append([ny, nx])
        
        # add unknown cells to the map
        distance_map_with_unknown = np.where(distance_grid == 999, -1, distance_grid)
        max_distance = np.max(distance_map_with_unknown)
        
        # Scales the distances in the distance_map_with_unknown to a range from 0 to 100 and "flips" them, making areas closer to walls have higher values, while preserving unknown cells. 
        flipped_map = np.where(distance_map_with_unknown > -1, (max_distance - distance_map_with_unknown) * 100 / max_distance, distance_map_with_unknown).astype(np.float)
        flipped_map = np.where(map_array == 1, map_array, flipped_map)

        # For vizualization::
        occupancy_grid_data = flipped_map.flatten().astype(np.int8).tolist()
        # Publish the occupancy grid of the distance
        occupancy_grid = OccupancyGrid()
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        occupancy_grid.info.resolution = mapdata.info.resolution
        occupancy_grid.header.frame_id = "/map"
        occupancy_grid.data = occupancy_grid_data
        occupancy_grid.info.origin = mapdata.info.origin
        self.penalty.publish(occupancy_grid)

        return flipped_map
    
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

    def calc_cspace(self, mapdata: OccupancyGrid, padding: int) -> OccupancyGrid:
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """

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
        """
        Calculates the optimal path using the A* algorithm from a starting tuple to a goal tuple.
        Returns the list of cells that were added to optimal path.
        :param mapdata [OccupancyGrid] The map data.
        :param start [tuple[int, int]] The starting cell.
        :param goal [tuple[int, int]] The target/goal cell.
        :return    [list[tuple[int, int]]] The Optimal Path.
        """

        # Validate start position, making sure that we start from a free cell. If we are not, we call find_closest_walkable_cell that returns the closest walkable cell.
        if (
            mapdata.data[PathPlanner.grid_to_index(mapdata, start)] != 0
        ):
            rospy.loginfo("Start position is invalid. Searching for the closest walkable cell...")
            start = PathPlanner.find_closest_walkable_cell(mapdata, start)
            # If the start position is invalid, we return an empty path and publish false to the "path_found" topic.
            if not start:
                rospy.loginfo("No valid start position found.")

                path_msg = Bool()
                path_msg.data = False
                print("next step is to publish false for the path found")
                self.path_found.publish(path_msg)

                return []

        # Validate goal position, making sure that we start from a free cell. If we are not, we call find_closest_walkable_cell that returns the closest walkable cell.
        if (
            mapdata.data[PathPlanner.grid_to_index(mapdata, goal)] != 0
        ):
            rospy.loginfo("Goal position is invalid. Searching for the closest walkable cell...")
            goal = PathPlanner.find_closest_walkable_cell(mapdata, goal)
            # If the goal position is invalid, we return an empty path and publish false to the "path_found" topic.
            if goal is None:
                rospy.loginfo("No valid goal position found.")

                path_msg = Bool()
                path_msg.data = False
                print("next step is to publish false for the path found")
                self.path_found.publish(path_msg)
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

        # Calculate distance map using BFS to use to penalize path that goes to close to obstacles.
        distance_map = self.wavefront_distance(mapdata)

        # runs until goal is found or frontier is fully explored
        while not frontier.empty():
            grid_cells = GridCells()
            grid_cells.header.frame_id = "/map"
            grid_cells.cell_width = mapdata.info.resolution
            grid_cells.cell_height = mapdata.info.resolution
            current = frontier.get()

            if current == truncated_goal:
                print("---------------------------------------------------reached goal when creating path")
                break

            for next in PathPlanner.neighbors_of_8(mapdata, current):
                # Invert x,y so that we find the correct penalty in the distance_map
                temporary = (next[1], next[0])
                penalty = distance_map[temporary]

                # Code initialy used to add penalties for turning
                # if came_from[current] is not None:
                #     print("came from current is not none")
                #     previous_direction = PathPlanner.calculate_direction(came_from[current], current)
                #     current_direction = PathPlanner.calculate_direction(current, next)
                # if previous_direction != current_direction:
                #   penalty += 0.01

                new_cost = cost_so_far[current] + PathPlanner.euclidean_distance(current, next) + (penalty**2)

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
        if current == truncated_goal: # if we found a path to the goal we send a True msg to "path_found"
            bool_msg = Bool()
            bool_msg.data = True
            self.path_found.publish(bool_msg)

            while current != truncated_start:
                path.append(current)
                current = came_from.get(current)

        else:
            rospy.loginfo("-----------------------------------------------------------could not find path to frontier")

            path_found_msg = Bool()
            path_found_msg.data = False
            print("The following step is to send to path found false because there can be no path")
            self.path_found.publish(path_found_msg)
        
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
        
    def path_to_message(self, mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> Path:
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        
        rospy.loginfo("Returning a Path message")
        # Creates the Path message, publishes it to the "actual_path_viz" topic and returns the Path message
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
        rospy.loginfo("The path planner node caught the centroid")

        ## Request the map, in case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        
        ## Calculate the C-space and publish it
        rospy.loginfo("about to create cspace")
        cspacedata = self.calc_cspace(mapdata, 2)
        rospy.loginfo("cspace done")

        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        
        ## Publish the Path message
        rospy.loginfo("---------------------- plan_path")
        return self.path_to_message(mapdata, path)

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        map = self.request_map()
        #self.calc_cspace(map, 4)
        rospy.spin()


# if the function is run as main, we request the map.
if __name__ == '__main__':
    PathPlanner().run()
