import rospy
from Nodes import Node
import numpy as np

class Graph:
	def __init__(self, map):

		self.map_ = map

		self.nodes_ = []

		self.grid_step_size_ = 2 # Grid spacing

		self.create_grid()

	def create_grid(self):

		# Create nodes
		idx = 0

		width=round(float(self.map_.max_x_)/self.grid_step_size_)
		height=round(float(self.map_.max_y_)/self.grid_step_size_)
		depth=round(float(self.map_.max_z_)/self.grid_step_size_)
		
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

							
		arr=np.array(self.nodes_).flatten()

		self.nodes_=list(arr)
		neighbours=[]
		for i in range(-1,2):
			for j in range(-1,2):
				for k in range(-1,2):
					if abs(i)+abs(j)+abs(k)==1:
						neighbours.append([i,j,k])

		for node_i in self.nodes_:
			# Debug print status
			
			
			if rospy.is_shutdown():
				return
			if node_i is not None:
				for n in neighbours:
					j=int(count+(n[0]*depth*height)+(n[1]*depth)+n[2])
					
					if j>=0 and j<len(self.nodes_):
						if n[2]!=-1 or count%depth!=0:
							node_j=self.nodes_[j]

							# Don't create edges to itself
							if node_j is not None:
								# Check if the nodes are close to each other
								distance = node_i.distance_to(node_j)
								# Check edge is collision free
								if distance<=distance_threshold:
									if node_i.is_connected(self.map_, node_j):
										#print(node_j.x,node_j.y,node_j.z)
										# Create the edge
										node_i.neighbours.append(node_j)
										node_i.neighbour_costs.append(distance)
									else:
										pass
										#print("what")
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
				#print(self.nodes_[i].x,self.nodes_[i].y,self.nodes_[i].z,newdist,i)
				if best_dist>newdist:
					best_dist=newdist
					best_index=i
				# you can remove this line after you have filled in the above code
		
		print(best_dist)
		return best_index
