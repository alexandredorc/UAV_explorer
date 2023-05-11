#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from Map import Map
from Graph import Graph
from GraphSearch import GraphSearch

class PathPlanning:
	def __init__(self,current,goal):
		self.state=False
		self.goal_coord=goal
		self.current_coord=current

		self.map=Map()

		while self.map.OG_map is None :
			rospy.sleep(0.1)

		self.map.update_map()

		self.goal=self.map.world_to_grid(goal[0],goal[1],goal[2])
		self.current=self.map.world_to_grid(current[0],current[1],current[2])
	
		
	def create_path(self):
		rate = rospy.Rate(10)
		self.map.update_map()
		graph = Graph(self.map)

		self.current=self.map.world_to_grid(self.current_coord[0],self.current_coord[1],self.current_coord[2])

		graph_search = GraphSearch(graph, self.current, self.goal)

		for node in graph_search.path_:
			print(node.x,node.y,node.z,node.idx)

			coord=self.map.grid_to_world(node.x,node.y,node.z)
			if not self.map.is_occupied_now(coord):
				rospy.logwarn(coord)

				publish_pos(publish_pos,coord)

				rate.sleep()
			else:
				print("colision")
				break
		self.goal_coord=coord
		self.state=False
		rospy.loginfo("finish path")

	def get_goal(self,msg):
		if not self.state:
			self.state = True
			self.current_coord=self.goal_coord
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

		path_plan=PathPlanning([0,0,1],[0,0,4])

		rate=rospy.Rate(10)

		while path_plan.map.OG_map is None:
			rate.sleep()

		path_plan.create_path()

		rospy.Subscriber("/goal", Point, path_plan.get_goal)


		while not rospy.is_shutdown():
			rospy.spin()
			rate.sleep()
	except rospy.ROSInterruptException:
		pass

