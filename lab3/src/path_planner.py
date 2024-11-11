#!/usr/bin/env python3

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from collections import Counter



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
        plan_path = rospy.service(self, GetPlan, self.plan_path())
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.cspace = rospy.Publisher("/path_planner/cspace", type=GridCells, queue_size=10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.expanded_cells = rospy.Publisher("path_planner/expandedcells", type=GridCells, queue_size=10)
        self.frontier = rospy.Publisher("path_planner/frontier", type=GridCells, queue_size=10)
        self.heuristic = rospy.Publisher("path_planner/heuristic")
        ## Initialize the request counter
        self.counter = Counter()
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
        
        x = p[0]
        y = p[1]
        width = mapdata.info.width
        return width * y + x
        



    @staticmethod
    def euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        
        return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

        


    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        
        grid_center = mapdata.info.origin.position
        grid_res = mapdata.info.resolution
        
        worldPoint = Point(x = grid_center.x + p[0] * grid_res, y = grid_center.y + p[1] * grid_res, z=0)
        
        return worldPoint


        
    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> tuple[int, int]:
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        
        grid_center = mapdata.info.origin.position
        grid_res = mapdata.info.resolution
        
        x = int((wp.x - grid_center.x) / grid_res)
        y = int((wp.y - grid_center.y) / grid_res)
        
        return x,y


        
    @staticmethod
    def path_to_poses(mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> list[PoseStamped]:
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        
        pose = []
        
        for coord in path:
            some_pose = PoseStamped()
            
            some_pose.pose.position = PathPlanner.grid_to_world(mapdata, coord)
            some_pose.pose.orientation = Quaternion(0,0,0,1)
            some_pose.header.stamp = rospy.Time.now()
            some_pose.header.frame_id = mapdata.info.origin
            
            pose.append(some_pose)
        return pose

    

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
        
        maxX = mapdata.info.width
        maxY = mapdata.info.height
        index = PathPlanner.grid_to_index(mapdata, p)
        if p.x < maxX and p.y < maxY:
            return mapdata.data[index] == -1 or mapdata.data[index] != 0
        return False

               

    @staticmethod
    def neighbors_of_4(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        
        possible = [(p[0], p[1] + 1),   #NORTH
                    (p[0], p[1] -1),    #SOUTH
                    (p[0] - 1, p[1]),   #EAST
                    (p[0] + 1, p[1])]   #WEST
        
        neighbors = []
        
        maxX = mapdata.info.width
        maxY = mapdata.info.height
        
        for pos in possible:
            if (0 <= pos[0] < maxX) and (0 <= pos[1] < maxY):
                if PathPlanner.is_cell_walkable(mapdata, pos):
                    neighbors.append(pos)
        
        return neighbors
            
    
    
    @staticmethod
    def neighbors_of_8(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        
        possible = [(p[0], p[1] + 1),       #NORTH
                    (p[0] - 1, p[1] + 1),   #NORTHEAST
                    (p[0] + 1, p[1] + 1),   #NORTHWEST
                    (p[0], p[1] -1),        #SOUTH
                    (p[0] - 1, p[1] - 1),   #SOUTHEAST
                    (p[0] + 1, p[1] - 1),   #SOUTHWEST
                    (p[0] - 1, p[1]),       #EAST
                    (p[0] + 1, p[1])]       #WEST
        
        neighbors = []
        
        maxX = mapdata.info.width
        maxY = mapdata.info.height
        
        for pos in possible:
            if (0 <= pos[0] < maxX) and (0 <= pos[1] < maxY):
                if PathPlanner.is_cell_walkable(mapdata, pos):
                    neighbors.append(pos)
        
        return neighbors

    
    
    @staticmethod
    def request_map() -> OccupancyGrid:
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('nav_msgs/OccupancyGrid')
        
        try:
            map = rospy.ServiceProxy('nav_msgs/OccupancyGrid', GetMap)
            return map()
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
        return None



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
        ## Create a GridCells message and publish it
        ## Return the C-space
        obstacles = []
        
        for cell in mapdata:
            if mapdata.data[PathPlanner.grid_to_index(mapdata, cell)] >= 50:
                obstacles.append[mapdata.data[cell]]
        
        for i in range(padding-1):
            for obst in obstacles:
                neighbors = PathPlanner.neighbors_of_8(obst)
                for nbCell in neighbors:
                    if mapdata.data.index[nbCell] == -1 and mapdata.data[PathPlanner.grid_to_index(mapdata, nbCell)] <= 50:
                        obstacles.append[mapdata.data[nbCell]]
        
        for occupied in obstacles:
            mapdata.data[PathPlanner.grid_to_index(mapdata, occupied)] = 100
        
        return mapdata
                        
                        
            


    
    
    ##Group Below
    
    def a_star(self, mapdata: OccupancyGrid, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))


    
    @staticmethod
    def optimize_path(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")

        

    def path_to_message(self, mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> Path:
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")


        
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
        ## Rpath_to_message(mapdata, waypoineturn a Path message
        return self.path_to_message(mapdata, waypoints)


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
