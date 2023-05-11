#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point


class Position:
	def __init__(self,initial):
		self.pos=initial

	def update_pose(self,msg):
		self.pos=[msg.x,msg.y,msg.z]

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

	rospy.logwarn("start")

	goal=Position([0,0,1])
				
	rospy.Subscriber("/gazebo_coordinate", Point, goal.update_pose)

	while not rospy.is_shutdown():
		
		publish_pos(gazebo_pos_pub,goal.pos)
		rate.sleep()
			

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

