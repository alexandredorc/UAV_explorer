import rospy



class GraphSearch:
	def __init__(self, graph, start_xyz, goal_xyz):
		self.graph_ = graph

		self.heuristic_weight_ = 1

		self.start_idx_ = self.graph_.get_closest_node(start_xyz)
		self.goal_idx_ = self.graph_.get_closest_node(goal_xyz)

		goal=graph.nodes_[self.goal_idx_]
		start=graph.nodes_[self.start_idx_]

		map=self.graph_.map_
		
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
			#print(node_idx)
			if self.goal_idx_==node_idx:
				rospy.loginfo("Goal found!")
				return
			# For each neighbour of the node
			
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
			path.append(current)
			current=current.parent_node
		path.append(current.parent_node)
		final_path=[]
		for i in range(0,len(path)):
			final_path.append(path[len(path)-i-1])
		return final_path
