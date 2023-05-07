import math

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
