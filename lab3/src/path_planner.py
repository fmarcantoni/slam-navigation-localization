#!/usr/bin/env python3

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from typing import Tuple as Tuple
from typing import List as List



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
        self.srv = rospy.Service('plan_path', GetPlan, self.plan_path)
        
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.cSpacePub = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=10)
        
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.expandedCellsPub = rospy.Publisher('/path_planner/expanded_cells', GridCells, queue_size=10)
        self.frontierPub = rospy.Publisher('/path_planner/frontier', GridCells, queue_size=10)
        self.heuristicsPub = rospy.Publisher('/path_planner/heuristics', GridCells, queue_size=10)
        self.visualPub = rospy.Publisher('/path_planner/visual', GridCells, queue_size=10)

        ## Initialize the request counter
        request_counter = 0
        thresholdOccupied = 50

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")



    @staticmethod
    def grid_to_index(mapdata: OccupancyGrid, p: Tuple[int, int]) -> int:
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The index.
        """
        ### REQUIRED CREDIT
        width = mapdata.info.width
        height = mapdata.info.height
        # index = mapdata(width*(p[1] - 1) + (p[0] - 1)) #this is for 0, 0 on the top right
        index = mapdata(width*height - width*p[1] - (width - p[0])) #this is for 0,0 being in the bottom left, and counting coordinates like normal
        return index



    @staticmethod
    def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        ### REQUIRED CREDIT
        distance = ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2) ** 0.5
        return distance
        


    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: Tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        ### REQUIRED CREDIT
        res = OccupancyGrid.info.resolution     #meters/cell
        originPose = OccupancyGrid.info.origin
        wp = Point()
        
        wp.x = p[0]*res + originPose.position.x
        wp.y = p[1]*res + originPose.position.y
        wp.z = 0 + originPose.position.z

        return wp


        
    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> Tuple[int, int]:
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        ### REQUIRED CREDIT
        res = OccupancyGrid.info.resolution #meters/cell
        originPose = OccupancyGrid.info.origin

        wp.x = wp.x - originPose.position.x
        wp.y = wp.y - originPose.position.y

        p = [wp.x // res, wp.y // res]        #tuple. // truncating division
        return p


        
    @staticmethod
    def path_to_poses(mapdata: OccupancyGrid, path: List[Tuple[int, int]]) -> List[PoseStamped]:
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        poses = []                                                      # make a list and a PoseStamped to fill up later
        worldCoordinate = PoseStamped()

        for i in range(len(path)-1):                                    # iterate through all the cell coordinates 
            cellCoordinate = path[i]
            if (i != (len(path) - 1)):              
                nextCellCoordinate = path[i+1]
            else:                                                       # if you are at the end of the path, don't look further
                nextCellCoordinate = cellCoordinate
            
            # POSITION
            worldCoordinate.pose.position = PathPlanner.grid_to_world(mapdata, cellCoordinate)  #use grid_to_world for the transformation

            # ORIENTATION
            dx = nextCellCoordinate[0] - cellCoordinate[0]
            dy = nextCellCoordinate[1] - cellCoordinate[1]
            theta = math.atan2(dy,dx)                                   # Use trig to find angle between the world coordinates
            worldCoordinate.pose.orientation.x = 0                      # Fill out the quaternion for that rotation around the z axis
            worldCoordinate.pose.orientation.y = 0
            worldCoordinate.pose.orientation.z = math.sin(theta / 2)
            worldCoordinate.pose.orientation.w = math.cos(theta / 2)

            poses.append(worldCoordinate)
        return poses

    

    @staticmethod
    def is_cell_walkable(mapdata:OccupancyGrid, p: Tuple[int, int]) -> bool:
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [bool]          True if the cell is walkable, False otherwise
        """
        ### REQUIRED CREDIT
        wakable = True
        index = PathPlanner.grid_to_index(mapdata, p)
        cellValue = mapdata.data[index]

        width = mapdata.info.width
        height = mapdata.info.height
        notWithin = (p[0] > (width - 1) or p[1] > (height - 1))                                 # not within if coords outside width and height

        if (cellValue == -1 or cellValue > PathPlanner.thresholdOccupied or notWithin):         # if unexplored, probably occupied, or outside of boundaries
            wakable = False                                                                     # cell is not wakable

        return wakable

               

    @staticmethod
    def neighbors_of_4(mapdata: OccupancyGrid, p: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        ### REQUIRED CREDIT
        pass

    
    
    @staticmethod
    def neighbors_of_8(mapdata: OccupancyGrid, p: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT
        pass

    
    
    @staticmethod
    def request_map() -> OccupancyGrid:
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        #write into the terminal
        rospy.loginfo("Requesting the map")

        # rospy.wait_for_service('static_map')

        try:
            get_map = rospy.ServiceProxy('static_map', GetMap)
            
            # Call the servie to get the map
            map_response = get_map().map
            print(map_response)

            #return an occupancy grid of. The map data is inside the 'map' field of the response
            return map_response
        
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
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

        width = mapdata.info.width
        height = mapdata.info.height

        # run through all the rows, and change the values next to the unwakable cells to also be unwakable
        for j in range(height):                                                             # iterate throgh all the rows
            for i in range(width):                                                          # in each row, iterate through
                if not PathPlanner.is_cell_wakable(mapdata, [i, j]) and (width !=1):        # make sure that the row is not only one block long
                    
                    for x in range(padding + 1):
                        if i == 0:
                            mapdata.dat[PathPlanner.grid_to_index(mapdata, [i + x, j])] = 100
                        elif i == (width - 1):
                            mapdata.data[PathPlanner.grid_to_index(mapdata, [i - x, j])] = 100
                        else:
                            mapdata.data[PathPlanner.grid_to_index(mapdata, [i + x, j])] = 100
                            mapdata.data[PathPlanner.grid_to_index(mapdata, [i - x, j])] = 100

        for i in range(width):
            for j in range(height):
                if not PathPlanner.is_cell_wakable(mapdata, [i, j]) and (height !=1):        # make sure that the row is not only one block long
                    if j == 0:
                        mapdata.data[PathPlanner.grid_to_index(mapdata, [i, j + 1])] = 100
                    elif j == (height - 1):
                        mapdata.data[PathPlanner.grid_to_index(mapdata, [i, j - 1])] = 100
                    else:
                        mapdata.data[PathPlanner.grid_to_index(mapdata, [i, j + 1])] = 100
                        mapdata.data[PathPlanner.grid_to_index(mapdata, [i, j - 1])] = 100

        return mapdata


    
    def a_star(self, mapdata: OccupancyGrid, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))


    
    @staticmethod
    def optimize_path(path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")

        

    def path_to_message(self, mapdata: OccupancyGrid, path: List[Tuple[int, int]]) -> Path:
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
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        PathPlanner.request_map()
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
    
