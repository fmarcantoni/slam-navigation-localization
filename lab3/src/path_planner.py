#!/usr/bin/env python3

from priority_queue import PriorityQueue

import math
import rospy
import copy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped



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
        self.heuristic = rospy.Publisher("path_planner/heuristic", GridCells, queue_size=10)# future
        self.path_viz = rospy.Publisher("path_planner/viz", GridCells, queue_size=10)
        self.actual_path_viz = rospy.Publisher("path_planner/actual_path_viz", Path, queue_size=10)
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
        return index



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

        # checks if grid is witihn boundaries of grid && free
        if mapdata.data[index] == 0:
            return True
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
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate - 1, y_coordinate + 1)): # W
            can_walk.append((x_coordinate - 1, y_coordinate))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate + 1, y_coordinate + 1)): # NE
            can_walk.append((x_coordinate + 1, y_coordinate + 1))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate - 1, y_coordinate + 1)): # NW
            can_walk.append((x_coordinate - 1, y_coordinate + 1))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate + 1, y_coordinate - 1)): # SE
            can_walk.append((x_coordinate, y_coordinate - 1))
        if PathPlanner.is_cell_walkable(mapdata, (x_coordinate - 1, y_coordinate - 1)): # SW
            can_walk.append((x_coordinate - 1, y_coordinate - 1))

        return can_walk
    

    
    
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
                        for j in range(max(0, h - padding), min(height, h + padding + 1))
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
        grid_cells = edited_cells
        self.c_space.publish(grid_cells)

        ## Return the C-space
        occupancy_grid = OccupancyGrid()
        occupancy_grid = mapdata
        occupancy_grid.data = grid
        return occupancy_grid


    
    def a_star(self, mapdata: OccupancyGrid, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))

        truncated_start = (int(start[0]), int(start[1]))
        truncated_goal = (int(goal[0]), int(goal[1]))
        frontier = PriorityQueue()
        frontier.put(truncated_start, 0)
        came_from = {}  
        cost_so_far = {}    
        came_from[truncated_start] = None
        cost_so_far[truncated_start] = 0

        # runs until goal is found or frontier is fully explored
        while not frontier.empty():
            grid_cells = GridCells()
            grid_cells.header.frame_id = "/map"
            grid_cells.cell_width = mapdata.info.resolution
            grid_cells.cell_height = mapdata.info.resolution
            current = frontier.get()
            if current == truncated_goal:
                break

            for next in PathPlanner.neighbors_of_8(mapdata, current):
                new_cost = cost_so_far[current] + PathPlanner.euclidean_distance(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(next, truncated_goal)
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
        current = truncated_goal
        i = 0
        while current != truncated_start:
            path.append(current)
            current = came_from[current]
        
        path.reverse()

        grid_cells = GridCells()
        grid_cells.header.frame_id = "/map"
        grid_cells.cell_width = mapdata.info.resolution
        grid_cells.cell_height = mapdata.info.resolution
        grid_cells.cells = [PathPlanner.grid_to_world(mapdata, p) for p in path]
        self.path_viz.publish(grid_cells)

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
            current = path
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
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")
        path_message = Path()
        path_message.header.frame_id = "/map"

        path_message.poses = PathPlanner.path_to_poses(mapdata, path)
        self.actual_path_viz.publish(path_message)

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
        cspacedata = self.calc_cspace(mapdata, 1)
        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
