#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
import math
import numpy as np
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path


class Node:
	def __init__(self, x, y, z, idx):

		# Index of the node in the graph
		self.idx = idx

		# Position of node
		self.x = x
		self.y = y
		self.z = z

		# Neighbouring edges
		self.neighbours = [] # List of nodes
		self.neighbour_costs = [] # the ith neighbour in self.neighbours has an edge cost defined as the ith element in self.neighbour_costs

		# Search variables
		self.cost = 9999999 # A large number
		self.parent_node = None # Invalid parent

	def distance_to(self, other_node):
		return math.sqrt((self.x-other_node.x)**2 + (self.y-other_node.y)**2 + (self.z-other_node.z)**2)

	def is_connected(self, map, other_node):
		p1 = [self.x, self.y , self.z]
		p2 = [other_node.x, other_node.y, other_node.z]
		return True

class Graph:
	def __init__(self, map):

		self.map_ = map

		self.nodes_ = []

		self.grid_step_size_ = 4 # Grid spacing
		##self.prm_num_nodes_ = rospy.get_param("~prm_num_nodes") # Number of PRM nodes

		# Publishers
		self.path_pub_ = rospy.Publisher('/path_planner/plan', Path, queue_size=1)

		self.create_grid()

	def create_grid(self):

		# Create nodes
		idx = 0

		width=(self.map_.max_x_+1)/self.grid_step_size_
		height=(self.map_.max_y_+1)/self.grid_step_size_
		depth=(self.map_.max_z_+1)/self.grid_step_size_	
		print(width, height,depth)
		self.nodes_=[]
		## DONE
		for x in range(self.map_.min_x_, self.map_.max_x_,self.grid_step_size_):
			mat_nodes=[]
			for y in range(self.map_.min_y_, self.map_.max_y_,self.grid_step_size_):
				line_nodes=[]
				for z in range(self.map_.min_z_, self.map_.max_z_,self.grid_step_size_):
					
					if rospy.is_shutdown():
						return

					# Check if it is occupied
					occupied = self.map_.is_occupied(x,y,z)

					# Create the node
					if not occupied:
						self.nodes_.append(Node(x,y,z,idx))
						
					else:
						self.nodes_.append(None)
					idx = idx + 1

		
		# Create edges
		count = 0
		# distance_threshold = math.sqrt(2*(self.grid_step_size_*1.01)**2) # Chosen so that diagonals are connected, but not 2 steps away
		distance_threshold = self.grid_step_size_*1.01 # only 4 connected

		rospy.logerr(len(self.nodes_))
							
		arr=np.array(self.nodes_).flatten()

		rospy.logerr(np.shape(arr))
		self.nodes_=list(arr)
		neighbours=[]
		for i in range(-1,2):
			for j in range(-1,2):
				for k in range(-1,2):
					if abs(i)+abs(j)+abs(k)==1:
						neighbours.append([i,j,k])
		print(neighbours)
		for node_i in self.nodes_:
			#print("test",node_i.x,node_i.y,node_i.z)
			# Debug print status
			
			
			if rospy.is_shutdown():
				return
			if node_i is not None:
				for n in neighbours:
					j=count+(n[0]*depth*height)+(n[1]*depth)+n[2]
					
					if j>=0 and j<len(self.nodes_):
						node_j=self.nodes_[j]

						# Don't create edges to itself
						if node_i != node_j and node_j is not None:
							#print(node_j.x,node_j.y,node_j.z)
							# Check if the nodes are close to each other
							distance = node_i.distance_to(node_j)
							#print(j)
							#print(node_j.x,node_j.y,node_j.z)
							if distance < distance_threshold:
								
								# Check edge is collision free
								if node_i.is_connected(self.map_.map, node_j):
									
									
									# Create the edge
									node_i.neighbours.append(node_j)
									node_i.neighbour_costs.append(distance)
				#print(count, "of", len(self.nodes_), len(node_i.neighbours), node_i.x, node_i.y, node_i.z)
			count += 1
			#rospy.logerr(count)
				
		


	def get_closest_node(self, xyz):
		# input: xy is a point in the form of an array, such that x=xy[0] and y=xy[1]. 
		# output: return the index of the node in self.nodes_ that has the lowest Euclidean distance to the point xy. 

		best_dist = 999999999 # A large number to begin with
		best_index = None # Index of best node found so far

		for i in range(len(self.nodes_)):
			if self.nodes_[i] is not None:
				newdist=(self.nodes_[i].x-xyz[0])**2+(self.nodes_[i].y-xyz[1])**2+(self.nodes_[i].z-xyz[2])**2
				
				if best_dist>newdist:
					best_dist=newdist
					best_index=i
				# you can remove this line after you have filled in the above code
			#print(self.nodes_[i].x,self.nodes_[i].y,self.nodes_[i].z)
		return best_index




class Map:
	def __init__(self):

		## Extract the image from a file
		# put the subscribed grid


		self.min_x_ = 0
		self.min_y_ = 0
		self.min_z_ = 0
		self.max_x_ = 1
		self.max_y_ = 1
		self.max_z_ = 1

		self.sub_map = rospy.Subscriber('occupancy_grid', OccupancyGrid, self.callback_occupancy_grid)

		self.map=None
		self.shape	=None
		self.OG_map=None
		#rospy.logwarn(self.shape	)
		self.goal = None
				

	def callback_occupancy_grid(self,msg):
		self.OG_shape	=[msg.info.width,msg.info.height,len(msg.data)/(msg.info.width*msg.info.height)]
		self.OG_map=np.reshape(np.array(msg.data),self.OG_shape	)
		self.origin_x=msg.info.origin.position.x
		self.origin_y=msg.info.origin.position.y
		self.origin_z=msg.info.origin.position.z
		self.resolution=msg.info.resolution
		

	def update_map(self):
		self.map=self.OG_map
		self.shape	=self.OG_shape	
		self.max_x_ = self.shape[0]
		self.max_y_ = self.shape[1]
		self.max_z_ = self.shape[2]

	def world_to_grid(self, x, y, z):

		return [round((x+0.5)/self.resolution+self.origin_x), round((y+0.5)/self.resolution+self.origin_y), round((z+0.2)/self.resolution+self.origin_z)]

	def grid_to_world(self, x, y, z):
		
		return [(x-self.origin_x)*self.resolution -0.5,(y-self.origin_y)*self.resolution-0.5, (z-self.origin_z)*self.resolution -0.2]

	def is_occupied(self, x, y, z):

		
		# Out o,f bounds
		if x < 0 or x >= self.shape	[0] or y < 0 or y >= self.shape	[1] or z < 0 or z >= self.shape	[2]:
			return True

		if self.map[x,y,z] !=0:
			return True
		else:
			return False

def is_occluded(map, p1, p2, threshold=235):
	# Draws a line from p1 to p2
	# Stops at the first pixel that is a "hit", i.e. above the threshold
	# Returns True if a hit is found, False otherwise

	# Extract the vector
	x1 = float(p1[0])
	y1 = float(p1[1])
	z1 = float(p1[2])
	x2 = float(p2[0])
	y2 = float(p2[1])
	z2 = float(p2[2])

	step = 1.0

	dx = x2 - x1
	dy = y2 - y1
	dz = z2 - z1
	l = math.sqrt(dx**2. + dy**2. + dz**2.)
	if l == 0:
		return False
	dx = dx / l
	dy = dy / l
	dz = dz / l

	max_steps = int(l / step)

	for i in range(max_steps):

		# Get the next pixel
		x = int(round(x1 + dx*i))
		y = int(round(y1 + dy*i))
		z = int(round(y1 + dy*i))
		
		shape_map=np.shape(map)
		# Check if it's outside the image
		if x < 0 or x >= shape_map[0] or y < 0 or y >= shape_map[1] or z < 0 or z >= shape_map[2]:
			return False

		# Check for "hit"
		if map[x, y, z] != 1:
			return True

	# No hits found
	return False

class GraphSearch:
	def __init__(self, graph, start_xyz, goal_xyz):
		self.graph_ = graph

		self.heuristic_weight_ = 1

		self.start_idx_ = self.graph_.get_closest_node(start_xyz)
		self.goal_idx_ = self.graph_.get_closest_node(goal_xyz)

		goal=graph.nodes_[self.goal_idx_]
		start=graph.nodes_[self.start_idx_]
		map=self.graph_.map_
		print("goal")
		print(goal.x,goal.y,goal.z)
		print(goal_xyz)
		print(map.grid_to_world(goal.x,goal.y,goal.z))

		print("start")
		print(start.x,start.y,start.z)
		print(start_xyz)
		print(map.grid_to_world(start.x,start.y,start.z))
		self.search(self.start_idx_, self.goal_idx_)
		rospy.loginfo("end search")

		self.path_ = self.generate_path(self.goal_idx_)



	def search(self, start_idx, goal_idx):
		# Find a path from nodes_[start_idx] to nodes_[goal_idx]
		print("id",start_idx, goal_idx)

		
		# Reset all parents and costs
		for n in self.graph_.nodes_:
			if n is not None:
				n.cost = 9999999 # a large number
				n.parent_node = None # invalid to begin with

		# Setup sets. These should contain indices (i.e. numbers) into the self.graph_.nodes_ array
		unvisited_set = []
		visited_set = []

		# Add start node to visited set
		unvisited_set.append(start_idx)
		self.graph_.nodes_[start_idx].cost = 0

		# Loop until solution found or graph is disconnected
		while len(unvisited_set) > 0:
			
			# Select a node
			# hint: self.get_minimum_cost_node(unvisited_set) will help you find the node with the minimum cost

			unvi_index=self.get_minimum_cost_node(unvisited_set)
			node_idx=unvisited_set[unvi_index]
			# Move the selected node from the unvisited_set to the visited_set

			visited_set.append(unvisited_set.pop(unvi_index))
			
			# Termination criteria
			# Finish early (i.e. "return") if the goal is found
			print(node_idx)
			if self.goal_idx_==node_idx:
				rospy.loginfo("Goal found!")
				return
			# For each neighbour of the node
			rospy.logerr("test")
			#print(self.graph_.nodes_[node_idx].x,self.graph_.nodes_[node_idx].y,self.graph_.nodes_[node_idx].z)
			for neighbour_idx in range(len(self.graph_.nodes_[node_idx].neighbours)):
				#rospy.logwarn("test")
				# For convenience, extract the neighbour and the edge cost from the arrays
				neighbour = self.graph_.nodes_[node_idx].neighbours[neighbour_idx]  
				neighbour_cost = self.graph_.nodes_[node_idx].neighbour_costs[neighbour_idx]

				# Check if neighbours is already in visited
				if neighbour.idx in visited_set:
					
					# Do nothing
					pass
				
				else:

					# Compute the cost of this neighbour node
					# hint: cost = cost-of-previous-node + cost-of-edge + self.heuristic_weight_ * A*-heuristic-score
					# hint: implement it without the heuristic-score first. once this is working, add the heuristic score.
					# hint: neighbour.distance_to() function is likely to be helpful for the heuristic-score
					cost=  self.graph_.nodes_[node_idx].cost+ neighbour_cost+(self.heuristic_weight_*neighbour.distance_to(self.graph_.nodes_[goal_idx]))
				
					# Check if neighbours is already in unvisited set
					if neighbour.idx in unvisited_set:

						# If the cost is lower than the previous cost for this node
						# Then update it to the new cost
						# Also, update the parent pointer to point to the new parent 

						if cost < neighbour.cost :
							neighbour.parent_node = self.graph_.nodes_[node_idx]
							neighbour.cost = cost

					else:

						# Add it to the unvisited set
						unvisited_set.append(neighbour.idx)

						# Initialise the cost and the parent pointer
						# hint: this will be similar to your answer above

						neighbour.parent_node = self.graph_.nodes_[node_idx]
						neighbour.cost = cost
			
		rospy.logerr("no path Found")
			# Visualise the current search status in RVIZ
			#rospy.sleep(0.1) # Pause for easier visualisation
		
				   

	def get_minimum_cost_node(self, unvisited_set):
		# Find the node that has the minimum cost

		# There's more efficient ways of doing this...
		min_cost = 99999999
		min_idx = None
		for idx in range(len(unvisited_set)):
			cost = self.graph_.nodes_[unvisited_set[idx]].cost
			if cost < min_cost:
				min_cost = cost
				min_idx = idx
		return min_idx

	def generate_path(self, goal_idx):
		# Generate the path by following the parents from the goal back to the start

		path = []

		current = self.graph_.nodes_[goal_idx]
		if current.cost==0:
			return [current]
		while current.parent_node.cost != 0:
			rospy.logerr("loop")
			path.append(current)
			current=current.parent_node
		path.append(current.parent_node)
		final_path=[]
		for i in range(0,len(path)):
			final_path.append(path[len(path)-i-1])
		return final_path


def gazebo_state(position):
	gazebo_pub_msgs=ModelState()
	gazebo_pub_msgs.pose.position.x=position[0]
	gazebo_pub_msgs.pose.position.y=position[1]
	gazebo_pub_msgs.pose.position.z=position[2]
	gazebo_pub_msgs.pose.orientation.x=0
	gazebo_pub_msgs.pose.orientation.y=0
	gazebo_pub_msgs.pose.orientation.z=0
	gazebo_pub_msgs.twist.linear.x=0
	gazebo_pub_msgs.twist.linear.y=0
	gazebo_pub_msgs.twist.linear.z=0
	gazebo_pub_msgs.twist.angular.x=0
	gazebo_pub_msgs.twist.angular.y=0
	gazebo_pub_msgs.twist.angular.z=0
	gazebo_pub_msgs.model_name='example'
	gazebo_pub_msgs.reference_frame='world'
	return gazebo_pub_msgs


def set_tf_world(position):
	br = tf.TransformBroadcaster()
	print(position)
	br.sendTransform((position[0], position[1], position[2]),
					 tf.transformations.quaternion_from_euler(0, 0, 0),
					 rospy.Time.now(),
					 "base_footprint",
					 "world")
	rospy.loginfo("tf update")

def publish_pos(pub,point):
	
	set_tf_world(point)
	gazebo_pub_msgs=gazebo_state(point)
	pub.publish(gazebo_pub_msgs)

def main():
	gazebo_pos_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

	rospy.init_node('set_pose_model', anonymous=True)

	rate = rospy.Rate(10) # 10hz

	start_point=[0,0,0]
	publish_pos(gazebo_pos_pub,start_point)
	rospy.sleep(3)

	start_point=[0,0,1]
	publish_pos(gazebo_pos_pub,start_point)
	rate.sleep()

	start_point=[0,0,2]
	publish_pos(gazebo_pos_pub,start_point)
	rate.sleep()


	rospy.logwarn("start")
	map=Map()
	
	while(map.OG_map is None):
		publish_pos(gazebo_pos_pub,start_point)
		rate.sleep()
				
	rospy.logerr(np.shape(map.OG_map))
	goal=[4,2,4]
	start_point=[0,0,2]
	goal_grid=map.world_to_grid(goal[0],goal[1],goal[2])
	start_grid=map.world_to_grid(start_point[0],start_point[1],start_point[2])
	
	
	print(map.origin_x,map.origin_y,map.origin_z)
	

	map.update_map()
	print(map.world_to_grid(goal[0],goal[1],goal[2]))
	print(map.grid_to_world(goal_grid[0],goal_grid[1],goal_grid[2]))
	graph = Graph(map)
	
	

	graph_search = GraphSearch(graph, start_grid, goal_grid)
	for node in graph_search.path_:
		coord=map.grid_to_world(node.x,node.y,node.z)
		print(node.x, node.y,node.z)
		rospy.logwarn(coord)
		publish_pos(gazebo_pos_pub,coord)
		#rate.sleep()
		rospy.sleep(1)
	
	#publish_pos(gazebo_pos_pub,goal)
	rate.sleep()
	while not rospy.is_shutdown():
		
		publish_pos(gazebo_pos_pub,goal)
		rate.sleep()
			

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

