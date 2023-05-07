import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

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

		if self.map[x,y,z] ==1:
			return True
		else:
			return False
