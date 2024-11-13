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
        self.thresholdOccupied = 50

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
        index = (width*height) - (width*p[1]) - (width - p[0]) #this is for 0,0 being in the bottom left, and counting coordinates like normal
        # print("p[0] : %i" % p[0])
        # print("p[1] : %i" % p[1])

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
        res = mapdata.info.resolution     #meters/cell
        originPose = mapdata.info.origin
        wp = Point()
        
        wp.x = p[0]*res + originPose.position.x + res/2
        wp.y = p[1]*res + originPose.position.y + res/2 + (mapdata.info.height - 1)*res
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
        
        width = mapdata.info.width
        height = mapdata.info.height

        notWithin = ((p[0] >= width) or (p[0] < 0)) or ((p[1] >= height) or p[1] < 0)        # not within if coords outside width and height and axis (0)
        if notWithin:
            return False
        
        index = PathPlanner.grid_to_index(mapdata, p)
        cellValue = mapdata.data[index]

        if (cellValue == -1 or cellValue > 50 or notWithin):         # if unexplored, probably occupied, or outside of boundaries
            return  False                                                                     # cell is not wakable

        return True

               

    @staticmethod
    def neighbors_of_4(mapdata: OccupancyGrid, p: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        ### REQUIRED CREDIT
        #check if the cell is on the edge
        neighborsFour = []

        # cellInQuestion = [p[0] - 1, p[1]]
        # if (cellInQuestion[0] >= 0) and (PathPlanner.is_cell_walkable(mapdata, cellInQuestion)):
        #     neighborsFour.append(cellInQuestion)

        # cellInQuestion[0] += 2
        # if (cellInQuestion[0] < width) and (PathPlanner.is_cell_walkable(mapdata, cellInQuestion)):
        #     neighborsFour.append(cellInQuestion)

        # cellInQuestion[0] -= 1
        # cellInQuestion[1] -= 1
        # if (cellInQuestion[1] >= 0) and (PathPlanner.is_cell_walkable(mapdata, cellInQuestion)):
        #     neighborsFour.append(cellInQuestion)

        # cellInQuestion[1] += 2
        # if (cellInQuestion[0] < height) and (PathPlanner.is_cell_walkable(mapdata, cellInQuestion)):
        #     neighborsFour.append(cellInQuestion)

        for i in range(-1, 2, 2):
            cellInQuestion = [p[0] + i, p[1]]
            if PathPlanner.is_cell_walkable(mapdata, cellInQuestion):
                    neighborsFour.append(cellInQuestion)
            cellInQuestion = [p[0], p[1] + i]
            if PathPlanner.is_cell_walkable(mapdata, cellInQuestion):
                    neighborsFour.append(cellInQuestion)

        return neighborsFour

    
    
    @staticmethod
    def neighbors_of_8(mapdata: OccupancyGrid, p: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """


        # use neighbors helper function, the method is helpful for cspace
        return PathPlanner.neighbors_radius(mapdata, p, 1)
    
        ### REQUIRED CREDIT
        # neighborsEight = PathPlanner.neighbors_of_4(mapdata, p)
        # neighborsEight = []

        # cellInQuestion = [p[0] - 1, p[1] - 1]

        # # if bottom left corner's coordinates are 0 or more  (in bounds)
        # # and it is wakable, then it is good
        # if (cellInQuestion[0] >= 0) and (cellInQuestion[1] >= 0) and (PathPlanner.is_cell_walkable(mapdata, cellInQuestion)):
        #     neighborsEight.append(cellInQuestion)
        #     #shift the cell in question two to the right (now it is bottom right corner)
        #     cellInQuestion[0] += 2
        #     # we had already checked that the y was good, so only need to check the x boundary
        #     if (cellInQuestion[0] < width) and (PathPlanner.is_cell_walkable(mapdata, cellInQuestion)):
        #             neighborsEight.append(cellInQuestion)

        # # go up two cells and two to the left(top left)
        # cellInQuestion[0] -= 2
        # cellInQuestion[1] += 2
        # if (cellInQuestion[0] >= 0) and (cellInQuestion[1] < height) and (PathPlanner.is_cell_walkable(mapdata, cellInQuestion)):
        #     neighborsEight.append(cellInQuestion)
        #     #shift the cell in question two to the right (now it is top right corner)
        #     cellInQuestion[0] += 2
        #     # we had already checked that the y was good, so only need to check the x boundary
        #     if (cellInQuestion[0] < width) and (PathPlanner.is_cell_walkable(mapdata, cellInQuestion)):
        #             neighborsEight.append(cellInQuestion)
        
        

    @staticmethod
    def neighbors_radius(mapdata: OccupancyGrid, p: Tuple[int, int], r: int) -> List[Tuple[int, int]]:
        """
        Returns the walkable padding cells of (x,y) in the occupancy grid within radius.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :param r       [int]           The radius of padding, the number of cells to pad
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        neighbors = []
        for i in range (-r, r + 1):
            for j in range (-r, r + 1):
                cellInQuestion = [p[0] + i, p[1] + j]
                if PathPlanner.is_cell_walkable(mapdata, cellInQuestion):
                    # do not add the original cell
                    if not (cellInQuestion == [p[0], p[1]]):
                        neighbors.append(cellInQuestion)

        return neighbors

    
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
        res = mapdata.info.resolution

        padCells = []
        newMapData = []

        gridCellsMessage = GridCells()
        gridCellsMessage.header.frame_id = "/map"
        gridCellsMessage.cell_height = mapdata.info.resolution
        gridCellsMessage.cell_width = mapdata.info.resolution
        

        newmap = OccupancyGrid()
        newmap.header = mapdata.header
        newmap.info = mapdata.info

        # run through all the rows, and change the values next to the unwakable cells to also be unwakable
        for j in range(height):                                                             # iterate throgh all the rows
            for i in range(width):                                                          # in each row, iterate through
                index = PathPlanner.grid_to_index(mapdata, [i, j])
                newMapData.append(mapdata.data[index])
                if (mapdata.data[index] > 50):  #PathPlanner.thersholdoccumpancy
                    neighbors = PathPlanner.neighbors_radius(mapdata, [i, j], padding)
                    padCells.append(neighbors)
                pass

        print("Done calculating cells to pad")

        # run through all the cells that need to be changed
        for neighbors in padCells:
            for cell in neighbors:
                index = PathPlanner.grid_to_index(mapdata, cell)

                cell[1] = -cell[1]
                newCell = PathPlanner.grid_to_world(mapdata, cell)
                
                newMapData[index] = 100

                # add new padded cells to the message
                gridCellsMessage.cells.append(newCell)
        
        print("Publishing gridCellsMessage on expandedCellsPub")
        # print(gridCellsMessage)
        self.expandedCellsPub.publish(gridCellsMessage)

        newmap.data = newMapData
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
        map = self.request_map()
        self.calc_cspace(map, 1)
        # print(self.grid_to_index(map, [36,0]))

        #should be 50
        print(self.grid_to_index(map, [12, 35]))

        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
    
