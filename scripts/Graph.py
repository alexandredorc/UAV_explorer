import rospy
from Nodes import Node
import numpy as np

class Graph:
	def __init__(self, map):

		self.map_ = map

		self.nodes_ = []

		self.grid_step_size_ = 1 # Grid spacing

		self.create_grid()

		self.graph_size=[100,100,40]
		self.graph_ori=[50,50,20]

	def create_grid(self):

		# Create nodes
		idx = 0

		
		self.nodes_=[]
		## DONE
		for x in range(0,self.graph_size[0]):
			
			for y in range(0,self.graph_size[1]):
				
				for z in range(0,self.graph_size[2]):
					world_x= (x/self.map_.resolution)+self.graph_ori[0]
					world_y= (y/self.map_.resolution)+self.graph_ori[1]
					world_z= (z/self.map_.resolution)+self.graph_ori[2]
					self.nodes_.append(Node(world_x,world_y,world_z,idx,False))
					
					idx = idx + 1

		# Create edges
		count = 0
		# distance_threshold = math.sqrt(2*(self.grid_step_size_*1.01)**2) # Chosen so that diagonals are connected, but not 2 steps away
		distance_threshold = self.map_.resolution*1.01 # only 4 connected

		
		neighbours=[]
		for i in range(-1,2):
			for j in range(-1,2):
				for k in range(-1,2):
					if abs(i)+abs(j)+abs(k)==1:
						neighbours.append([i,j,k])

		for node_i in self.nodes_:
			for n in neighbours:
				j=int(count+(n[0]*self.graph_size[2]*self.graph_size[1])+(n[1]*self.graph_size[2])+n[2])
				
				if j>=0 and j<len(self.nodes_):
					if n[2]!=-1 or count%self.graph_size[2]!=0:
						node_j=self.nodes_[j]

						# Check if the nodes are close to each other
						distance = node_i.distance_to(node_j)
						print(distance)
						# Check edge is collision free
						if distance<=distance_threshold:
							if node_i.is_connected(self.map_, node_j):
								#print(node_j.x,node_j.y,node_j.z)
								# Create the edge
								node_i.neighbours.append(node_j)
								node_i.neighbour_costs.append(distance)
							else:
								pass
		count += 1
			#rospy.logerr(count)
				
		
	def update_graph(self):
		self.map_.update_map()
		width=round(float(self.map_.max_x_)/self.grid_step_size_)
		height=round(float(self.map_.max_y_)/self.grid_step_size_)
		depth=round(float(self.map_.max_z_)/self.grid_step_size_)
		
		ori_x=self.graph_size[0]-self.map_.origin_x
		ori_y=self.graph_size[1]-self.map_.origin_y
		ori_z=self.graph_size[2]-self.map_.origin_z

		for x in range(width):
			for y in range(height):
				for z in range(depth):
					j=int(((x+ori_x)*self.graph_size[2]*self.graph_size[1])+((y+ori_y)*self.graph_size[2])+(z+ori_z))
					self.nodes_[j].occupied=self.map_.is_occupied(x,y,z)


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
