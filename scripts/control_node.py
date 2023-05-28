#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class Control:
	def __init__(self):
		self.genes=None

		self.goal_pub = rospy.Publisher('/goal', Point, queue_size=10)

	

	def select_frontier(self):
		#COMPLETE GOAL
		best_score=0
		best_index=-1
		for i in range(self.nb_front):
    		
			score=np.dot(self.genes,np.transpose(self.frontier_info[i]))
			if best_score<score:
				best_score=score
				best_index=i
		self.idx_goal=best_index

	def get_frontier_info(self,msg):
		if self.genes is not None:
			size=len(msg.data)
			self.frontier_info=np.reshape(np.array(msg.data),(size/10,10))
			self.nb_front=size/10
			self.select_frontier()
			selected_front=self.frontier_info[self.idx_goal]
			print("select",selected_front)
			self.goal=[selected_front[0],selected_front[1],selected_front[2]]
			self.publish_goal()

	def get_genes(self,msg):
		self.genes=np.array(msg.data)
		
	def publish_goal(self):
		rospy.logwarn(self.goal)
		msg_point=Point()
		msg_point.x=round(self.goal[0])
		msg_point.y=round(self.goal[1])
		msg_point.z=round(self.goal[2])
		print(self.goal)
		self.goal_pub.publish(msg_point)
	

if __name__ == '__main__':
	try:
		rospy.init_node('control_node', anonymous=True)

		control=Control()

		sub_front = rospy.Subscriber('frontier_info', Float32MultiArray, control.get_frontier_info)

		sub_gen = rospy.Subscriber('current_genes', Float32MultiArray, control.get_genes)

		loop_rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			rospy.spin()
			loop_rate.sleep()
	except rospy.ROSInterruptException:
		pass

