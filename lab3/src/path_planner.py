
#!/usr/bin/env python3
from __future__ import annotations
import math
import rospy
import copy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from priority_queue import PriorityQueue



class PathPlanner:


	
	def __init__(self):
		"""
		Class constructor
		"""
		### REQUIRED CREDIT
		rospy.init_node("path_planner")
		## Initializeplan_pathan and calls self.plan_path() when a message is received
		plan_path = rospy.Service('plan_path', GetPlan, self.plan_path)
		## Create a publisher for the C-space (the enlarged occupancy grid)
		## The topic is "/path_planner/cspace", the message type is GridCells
		self.cspace = rospy.Publisher('path_planner/cspace', GridCells, queue_size=10)
		## Create publishers for A* (expanded cells, frontier, ...)
		## Choose a the topic names, the message type is GridCells
		self.expanded_cells = rospy.Publisher('path_planner/expanded_cells',GridCells, queue_size=10)
		self.frontier = rospy.Publisher('path_planner/frontier', GridCells, queue_size=10)
		self.heuristic = rospy.Publisher('path_planner/heuristic', GridCells, queue_size=10)
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
		x = p[0]
		y = p[1]
		index = y*mapdata.info.width + x
		return index



	@staticmethod
	def euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
		"""
		Calculates the Euclidean distance between two points.
		:param p1 [(float, float)] first point.
		:param p2 [(float, float)] second point.
		:return   [float]		  distance.
		"""
		distance = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
		return distance 
		


	@staticmethod
	def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
		"""
		Transforms a cell coordinate in the occupancy grid into a world coordinate.
		:param mapdata [OccupancyGrid] The map information.
		:param p [(int, int)] The cell coordinate.
		:return		[Point]		 The position in the world.
		"""
		origin_x = mapdata.info.origin.position.x
		origin_y = mapdata.info.origin.position.y
		map_resolution = mapdata.info.resolution
		
		# add to the coordinates 0.5 to be at the center of the robot
		world_point = Point()
		world_point.x = (p[0] + 0.5) * map_resolution + origin_x
		world_point.y = (p[1] + 0.5) * map_resolution + origin_y
		world_point.z = 0
		return world_point


		
	@staticmethod
	def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> tuple[int, int]:
		"""
		Transforms a world coordinate into a cell coordinate in the occupancy grid.
		:param mapdata [OccupancyGrid] The map information.
		:param wp	  [Point]		 The world coordinate.
		:return		[(int,int)]	 The cell position as a tuple.
		"""
		origin_x = mapdata.info.origin.position.x
		origin_y = mapdata.info.origin.position.y
		map_resolution = mapdata.info.resolution

		cell_x = int(((wp.x - origin_x)/map_resolution) - 0.5)
		cell_y = int(((wp.y - origin_y)/map_resolution) - 0.5)

		return [cell_x, cell_y]


		
	@staticmethod
	def path_to_poses(mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> list[PoseStamped]:
		"""
		Converts the given path into a list of PoseStamped.
		:param mapdata [OccupancyGrid] The map information.
		:param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
		:return		[[PoseStamped]] The path as a list of PoseStamped (world coordinates).
		"""
		poses = []
		origin_x = mapdata.info.origin.position.x
		origin_y = mapdata.info.origin.position.y
		map_resolution = mapdata.info.resolution

		for cell in path:
			world_point = PathPlanner.grid_to_world(mapdata, cell)
			
			# create PoseStamped
			pose = PoseStamped()
			pose.header.frame_id = mapdata.header.frame_id
			pose.header.stamp = rospy.Time.now()
			pose.pose.position.x = world_point.x
			pose.pose.position.y = world_point.y
			pose.pose.position.z = world_point.z
			pose.pose.orientation = Quaternion(0, 0, 0, 1)
			#pose.pose.orientation.w = 1.0
			poses.append(pose)

		return poses

	

	@staticmethod
	def is_cell_walkable(mapdata:OccupancyGrid, p: tuple[int, int]) -> bool:
		"""
		A cell is walkable if all of these conditions are true:
		1. It is within the boundaries of the grid;
		2. It is free (not unknown, not occupied by an obstacle)
		:param mapdata [OccupancyGrid] The map information.
		:param p	   [(int, int)]	The coordinate in the grid.
		:return		[bool]		  True if the cell is walkable, False otherwise
		"""
		index = PathPlanner.grid_to_index(mapdata, p)
		width = mapdata.info.width
		height = mapdata.info.height
	
		if not ((0 <= p[0] < width) and (0 <= p[1] < height)):
			return False
		else:
			if mapdata.data[index] == 100:
				return False
			else:
				return True  

			   

	@staticmethod
	def neighbors_of_4(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
		"""
		Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
		:param mapdata [OccupancyGrid] The map information.
		:param p	   [(int, int)]	The coordinate in the grid.
		:return		[[(int,int)]]   A list of walkable 4-neighbors.
		"""
		walkable_neighbours = []
		cell_x = p[0]
		cell_y = p[1]

		if PathPlaner.is_cell_walkable(mapdata, [cell_x + 1, cell_y]):
			walkable_neighbours.append([cell_x + 1, cell_y])
		if PathPlnner.is_cell_walkable(mapdata, [cell_x - 1, cell_y]):
			walkable_neighbours.append([cell_x - 1, cell_y])
		if PathPlanner.is_cell_walkable(mapdata, [cell_x, cell_y + 1]):
			walkable_neighbours.append([cell_x, cell_y + 1])
		if PathPlanner.is_cell_walkable(mapdata, [cell_x, cell_y - 1]):
			walkable_neighbours.append([cell_x, cell_y - 1])

		return walkable_neighbours

	
	
	@staticmethod
	def neighbors_of_8(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
		"""
		Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
		:param mapdata [OccupancyGrid] The map information.
		:param p	   [(int, int)]	The coordinate in the grid.
		:return		[[(int,int)]]   A list of walkable 8-neighbors.
		"""
		walkable_neighbours = []
		cell_x = p[0]
		cell_y = p[1]
		
		if PathPlanner.is_cell_walkable(mapdata, [cell_x - 1, cell_y - 1]):
			walkable_neighbours.append([cell_x - 1, cell_y - 1])
		if PathPlanner.is_cell_walkable(mapdata, [cell_x - 1, cell_y]):
			walkable_neighbours.append([cell_x - 1, cell_y])
		if PathPlanner.is_cell_walkable(mapdata, [cell_x - 1, cell_y + 1]):
			walkable_neighbours.append([cell_x - 1, cell_y + 1])
		if PathPlanner.is_cell_walkable(mapdata, [cell_x, cell_y - 1]):
			walkable_neighbours.append([cell_x, cell_y - 1])
		if PathPlanner.is_cell_walkable(mapdata, [cell_x, cell_y + 1]):
			walkable_neighbours.append([cell_x, cell_y + 1])
		if PathPlanner.is_cell_walkable(mapdata, [cell_x + 1, cell_y - 1]):
			walkable_neighbours.append([cell_x + 1, cell_y - 1])
		if PathPlanner.is_cell_walkable(mapdata, [cell_x + 1, cell_y]):
			walkable_neighbours.append([cell_x + 1, cell_y])
		if PathPlanner.is_cell_walkable(mapdata, [cell_x + 1, cell_y + 1]):
			walkable_neighbours.append([cell_x + 1, cell_y + 1])

		return walkable_neighbours

	
	
	@staticmethod
	def request_map() -> OccupancyGrid:
		"""
		Requests the map from the map server.
		:return [OccupancyGrid] The grid if the service call was successful,
								None in case of error.
		"""
		### REQUIRED CREDIT
		rospy.loginfo("Requesting the map")
		rospy.wait_for_service('static_map')
		try:
            #map_request = rospy.ServiceProxy('static_map', GetMap)
			map_request = rospy.ServiceProxy('static_map', GetMap)
			return map_request().map

		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: %s" % e)
			return None


	def calc_cspace(self, mapdata: OccupancyGrid, padding: int) -> OccupancyGrid:
		"""
		Calculates the C-Space, i.e., makes the obstacles in the map thicker.
		Publishes the list of cells that were added to the original map.
		:param mapdata [OccupancyGrid] The map data.
		:param padding [int]		   The number of cells around the obstacles.
		:return		[OccupancyGrid] The C-Space.
		"""
		### REQUIRED CREDIT
		rospy.loginfo("Calculating C-Space")

		padded_cells = []
		width = mapdata.info.width
		height = mapdata.info.height
		cspace_data = list(copy.deepcopy(mapdata.data))

		for w in range(width):
			for h in range(height):
				index = PathPlanner.grid_to_index(mapdata, [w, h])
				if mapdata.data[index] == 100:
					for x in range(max(0, w - padding), min(width, w + padding + 1)):
						for y in range(max(0, h - padding), min(height, h + padding + 1)):
							neighbor_index = PathPlanner.grid_to_index(mapdata, [x, y])
							if mapdata.data[neighbor_index] == 0:
								cspace_data[neighbor_index] = 100
								world_point = PathPlanner.grid_to_world(mapdata, [x,y])
								padded_cells.append(world_point)

		
		## Create a GridCells message and publish it
		msg = GridCells()
		msg.header = mapdata.header
		msg.cell_width = mapdata.info.resolution
		msg.cell_height = mapdata.info.resolution
		msg.cells = padded_cells

		self.cspace.publish(msg)
	
		## Return the C-space
		cspace_map = OccupancyGrid()
		cspace_map.header = mapdata.header
		cspace_map.info = mapdata.info
		cspace_map.data = mapdata.data

		return cspace_map


	
	def a_star(self, mapdata: OccupancyGrid, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
		rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))

		actual_start = (int(start[0]), int(start[1]))
		actual_goal = (int(goal[0]), int(goal[1]))
		frontier = PriorityQueue()
		frontier.put(actual_start, 0)
		cost_dict = {}
		path_dict = {}
		visited = {}
		path_dict[actual_start] = None
		cost_dict[actual_start] = 0

		while frontier:
			current = frontier.get()
			if current == actual_goal:
				break
			
			neighbors = PathPlanner.neighbors_of_8(mapdata, current)

			
			(priority, node) = heapq.heappop(frontier)
			visited.pdate(node)

			if node == goal:
				break
			else:
				unvisited_neighbors = set(mapdata[node].keys()) - visited
				for neighbor in unvisited_neighbors:

					cost_to_node = cost_dict[node] + mapdata[node][neighbor]

					if (neighbor not in cost_dict) or (cost_to_node < cost_dict[neighbor]):
						cost_dict[neighbor] = cost_to_node
						path_dict[neighbor] = path_dict[node] + [neighbor]

						heuristic = len(path_dict[node]) + 1
						estimated_cost = 1*cost_to_node + 1*heuristic
						heapq.heappush(frontier, (estimated_cost, neighbor))

	return path_dict[target]


	
	@staticmethod
	def optimize_path(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
		"""
		Optimizes the path, removing unnecessary intermediate nodes.
		:param path [[(x,y)]] The path as a list of tuples (grid coordinates)
		:return	 [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
		"""
		### EXTRA CREDIT
		rospy.loginfo("Optimizing path")

		

	def path_to_message(self, mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> Path:
		"""
		Takes a path on the grid and returns a Path message.
		:param path [[(int,int)]] The path on the grid (a list of tuples)
		:return	 [Path]		A Path message (the coordinates are expressed in the world)
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
		my_map = PathPlanner.request_map()
		self.calc_cspace(my_map, 2)
		rospy.spin()


		
if __name__ == '__main__':
	PathPlanner().run()
