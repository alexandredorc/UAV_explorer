#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
import numpy as np
from gazebo_msgs.msg import ModelState

from Map import Map
from Graph import Graph
from GraphSearch import GraphSearch

def gazebo_state(position):
	gazebo_pub_msgs=ModelState()
	gazebo_pub_msgs.pose.position.x=position[0]
	gazebo_pub_msgs.pose.position.y=position[1]
	gazebo_pub_msgs.pose.position.z=position[2]
	gazebo_pub_msgs.pose.orientation.x=0
	gazebo_pub_msgs.pose.orientation.y=0
	gazebo_pub_msgs.pose.orientation.z=0
	gazebo_pub_msgs.twist.linear.x=0
	gazebo_pub_msgs.twist.linear.y=0
	gazebo_pub_msgs.twist.linear.z=0
	gazebo_pub_msgs.twist.angular.x=0
	gazebo_pub_msgs.twist.angular.y=0
	gazebo_pub_msgs.twist.angular.z=0
	gazebo_pub_msgs.model_name='example'
	gazebo_pub_msgs.reference_frame='world'
	return gazebo_pub_msgs


def set_tf_world(position):
	br = tf.TransformBroadcaster()
	print(position)
	br.sendTransform((position[0], position[1], position[2]),
					 tf.transformations.quaternion_from_euler(0, 0, 0),
					 rospy.Time.now(),
					 "base_footprint",
					 "world")
	rospy.loginfo("tf update")

def publish_pos(pub,point):
	
	set_tf_world(point)
	gazebo_pub_msgs=gazebo_state(point)
	pub.publish(gazebo_pub_msgs)

def main():
	gazebo_pos_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

	rospy.init_node('set_pose_model', anonymous=True)

	rate = rospy.Rate(10) # 10hz

	start_point=[0,0,0]
	publish_pos(gazebo_pos_pub,start_point)
	rospy.sleep(3)

	start_point=[0,0,1]
	publish_pos(gazebo_pos_pub,start_point)
	rate.sleep()

	start_point=[0,0,2]
	publish_pos(gazebo_pos_pub,start_point)
	rate.sleep()


	rospy.logwarn("start")
	map=Map()
	
	while(map.OG_map is None):
		publish_pos(gazebo_pos_pub,start_point)
		rate.sleep()
				
	rospy.logerr(np.shape(map.OG_map))
	goal=[-7.25, -16.0, 2.8]
	start_point=[0,0,2]
	goal_grid=map.world_to_grid(goal[0],goal[1],goal[2])
	start_grid=map.world_to_grid(start_point[0],start_point[1],start_point[2])
	
	print(map.grid_to_world(16,18,15))
	

	map.update_map()
	graph = Graph(map)
	
	

	graph_search = GraphSearch(graph, start_grid, goal_grid)
	for node in graph_search.path_:
		coord=map.grid_to_world(node.x,node.y,node.z)
		print(node.x, node.y,node.z)
		rospy.logwarn(coord)
		publish_pos(gazebo_pos_pub,coord)
		#rate.sleep()
		rospy.sleep(1)
	
	#publish_pos(gazebo_pos_pub,goal)
	rate.sleep()
	while not rospy.is_shutdown():
		
		publish_pos(gazebo_pos_pub,goal)
		rate.sleep()
			

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

