import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

class Map:
	def __init__(self):

		self.min_x_ = -50
		self.min_y_ = -50
		self.min_z_ = -20
		self.max_x_ = 50
		self.max_y_ = 50
		self.max_z_ = 20

		self.sub_map = rospy.Subscriber('occupancy_grid', OccupancyGrid, self.callback_occupancy_grid)

		self.map=None
		self.shape	=None
		self.OG_map=None

		self.resolution=None
		#rospy.logwarn(self.shape	)
		self.goal = None
				

	def callback_occupancy_grid(self,msg):
		self.OG_shape=[msg.info.width,msg.info.height,len(msg.data)/(msg.info.width*msg.info.height)]
		self.OG_map=np.reshape(np.array(msg.data),self.OG_shape	)
		self.OG_origin_x=msg.info.origin.position.x
		self.OG_origin_y=msg.info.origin.position.y
		self.OG_origin_z=msg.info.origin.position.z
		self.OG_resolution=msg.info.resolution
		

	def update_map(self):
		self.map=self.OG_map
		self.shape	=self.OG_shape	
		self.origin_x=self.OG_origin_x
		self.origin_y=self.OG_origin_y
		self.origin_z=self.OG_origin_z
		self.resolution=self.OG_resolution

	def world_to_grid(self, x, y, z):
		
		return [round((x)/self.resolution+self.origin_x), round((y)/self.resolution+self.origin_y), round((z)/self.resolution+self.origin_z)]

	def grid_to_world(self, x, y, z):
		
		return [(x-self.origin_x)*self.resolution ,(y-self.origin_y)*self.resolution, (z-self.origin_z)*self.resolution ]

	def is_occupied(self, x, y, z):

		# Out o,f bounds
		if x < 0 or x >= self.shape	[0] or y < 0 or y >= self.shape	[1] or z < 0 or z >= self.shape	[2]:
			return True

		if self.map[x,y,z] ==1:
			return True
		else:
			return False
		
	def is_occupied_now(self, xyz):
		x=xyz[0]
		y=xyz[1]
		z=xyz[2]
		print("xyz",x,y,z)
		xd=round((x)/self.resolution+self.OG_origin_x)
		yd=round((y)/self.resolution+self.OG_origin_y)
		zd=round((z)/self.resolution+self.OG_origin_z)
		
		# Out o,f bounds
		if xd < 0 or xd >= self.OG_shape[0] or yd < 0 or yd >= self.OG_shape[1] or zd < 0 or zd >= self.OG_shape[2]:
			return True

		if self.OG_map[int(xd),int(yd),int(zd)] ==1:
			return True
		else:
			return False
