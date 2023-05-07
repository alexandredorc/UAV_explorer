#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray

class Cluster:
	def __init__(self,grid_front,width,height,depth):
		self.width=width
		self.height=height
		self.depth=depth
		self.element_frontier=[]
		self.grid_front=grid_front
	
	def get_frontier(self,pos,count):
		count+=1
		self.grid_front[pos[0],pos[1],pos[2]]=False
		self.element_frontier.append([pos[0],pos[1],pos[2]])
		if count==900:
			return
		
		for i in range(-1,2):
			for j in range(-1,2):
				for k in range(-1,2):
					if pos[0]+i>=0 and pos[1]+j>=0 and pos[2]+k>=0 and pos[0]+i<self.width and pos[1]+j<self.height and pos[2]+k<self.depth:
						if self.grid_front[pos[0]+i,pos[1]+j,pos[2]+k]==True:
							
							self.get_frontier([pos[0]+i,pos[1]+j,pos[2]+k],count)


def generate_cluster(width,height,depth,grid_front):
	clusters=[]
	for x in range(width):
		for y in range(height):
			for z in range(depth):
				
				if grid_front[x,y,z]:
					clu=Cluster(grid_front,width,height,depth)
					clu.get_frontier([x,y,z],0)
					grid_front=clu.grid_front
					if (len(clu.element_frontier)>=4):

						clusters.append(clu)
						front_arr=np.array(clu.element_frontier)
						clu.center=np.average(front_arr,axis=0).astype(np.float32)
					
						clu.frt_min=np.amin(front_arr,axis=0).astype(np.float32)
						clu.frt_max=np.amax(front_arr,axis=0).astype(np.float32)
	return clusters

def format_front_msg(clusters):
	frontiers_info=[]
	for clust in clusters:
		frontiers_info+=list(clust.center)+list(clust.frt_min)+list(clust.frt_max)+[len(clust.element_frontier)]

	print(len(clusters))
	#rospy.loginfo(frontiers_info)
	msg=Float32MultiArray()
	msg.data=frontiers_info
	return msg
				
def octomap_grid_callback(msg):
	arr=np.array(msg.data)
	width=msg.info.width
	height=msg.info.height
	depth=len(msg.data)/(width*height)
	new_occ=np.reshape(arr,(width,height,depth))
	grid_front=np.full((width,height,depth), False)
	count=0

	for x in range(width):
		for y in range(height):
			for z in range(depth):
				
				if new_occ[x,y,z]==-1:
					
					res=False
					for i in range(-1,2):
						for j in range(-1,2):
							for k in range(-1,2):
								
								if x+i>=0 and y+j>=0 and z+k>=0 and x+i<width and y+j<height and z+k<depth:
									
									if new_occ[x+i,y+j,z+k]==0:
	
										res=True
										break
					if res:
						count+=1
						grid_front[x,y,z]=True
					
	rospy.loginfo(count)
	clusters=generate_cluster(width,height,depth,grid_front)
	msg=format_front_msg(clusters)
	pub.publish(msg)



if __name__ == '__main__':
	# Initialize ROS node
	rospy.init_node('get_occupancy_grid')

	# Subscribe to the octomap binary topic
	sub = rospy.Subscriber('occupancy_grid', OccupancyGrid, octomap_grid_callback)

	# Publish the occupancy grid to a topic
	pub = rospy.Publisher('/frontier_info', Float32MultiArray, queue_size=1)

	loop_rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rospy.spin()
		loop_rate.sleep()

