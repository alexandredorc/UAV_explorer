#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from Map import Map
from Graph import Graph
from GraphSearch import GraphSearch

class PathPlanning:
	def __init__(self,current,goal):
	
		self.current=current
		self.goal=goal

		self.map=Map()

		while self.map.OG_map is None :
			rospy.sleep(0.1)

		self.map.update_map()

		self.goal=self.map.world_to_grid(self.goal[0],self.goal[1],self.goal[2])
		self.current=self.map.world_to_grid(self.current[0],self.current[1],self.current[2])
	
		
	def create_path(self):
		rate = rospy.Rate(5)
		self.map.update_map()
		graph = Graph(self.map)
		print(self.map.origin_x,self.map.origin_y,self.map.origin_z)
		

		graph_search = GraphSearch(graph, self.current, self.goal)

		for node in graph_search.path_:
			print(node.x,node.y,node.z)
			coord=self.map.grid_to_world(node.x,node.y,node.z)
			
			rospy.logwarn(coord)
			publish_pos(publish_pos,coord)

			rate.sleep()
		
		rospy.loginfo("finish path")

	def get_goal(self,msg):
		self.current=self.goal
		self.goal=[msg.x,msg.y,msg.z]
		self.create_path()

		
def publish_pos(pub,pos):
	msg_point=Point()
	msg_point.x=pos[0]
	msg_point.y=pos[1]
	msg_point.z=pos[2]
	pos_pub.publish(msg_point)
		

if __name__ == '__main__':
	try:
		pos_pub = rospy.Publisher('/gazebo_coordinate', Point, queue_size=10)

		rospy.init_node('path_planner', anonymous=True)

		path_plan=PathPlanning([0,0,0],[0,0,8])

		rate=rospy.Rate(10)

		while path_plan.map.OG_map is None:
			rate.sleep()

		path_plan.create_path()

		rospy.Subscriber("/goal", Point, path_plan.get_goal)

		loop_rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			rospy.spin()
			loop_rate.sleep()
	except rospy.ROSInterruptException:
		pass

