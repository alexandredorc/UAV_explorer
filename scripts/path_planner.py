#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from Map import Map
from Graph import Graph
from GraphSearch import GraphSearch
from Genetics import Training
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray

class PathPlanning:
	def __init__(self,current,goal):
		self.state=False

		self.steps=0
		map_=Map()
		self.graph = Graph(map_)

		self.run=True
		while self.map.OG_map is None :
			rospy.sleep(0.1)
		

		self.graph.map_.update_map()

		rospy.Subscriber("/goal", Point, self.get_goal)
		
	def create_path(self):
		rate = rospy.Rate(10)
		self.graph.update_graph()

		graph_search = GraphSearch(self.graph, self.current, self.goal)

		for node in graph_search.path_:
			self.steps+=1
			if self.steps>=200:
				self.run=False
				break
			print(node.x,node.y,node.z,node.idx)

			coord=[node.x,node.y,node.z]
			if not self.graph.map_.is_occupied_now(coord):
				rospy.logwarn(coord)

				publish_pos(coord)

				rate.sleep()
			else:
				print("colision")
				break
		
		self.state=False
		rospy.loginfo("finish path")

	def get_goal(self,msg):
		rospy.loginfo("get goal")
		if not self.state:
			self.state = True
			self.current=self.goal
			self.graph.map_.grid_to_world(msg.x,msg.y,msg.z)
			self.create_path()

def publish_pos(pos):
	msg_point=Point()
	msg_point.x=pos[0]
	msg_point.y=pos[1]
	msg_point.z=pos[2]
	pos_pub.publish(msg_point)

def doTraining(train):
	train.current_gen.initial_gen()
	train.start_session_to_json()
	while(True):
		print(train.current_gen_num)
		train.current_gen=get_fitness_gen(train.current_gen)
		train.write_data_to_json()
		if train.stop_condition():
			break
		train.nextGeneration()	

def get_fitness_gen(gen):
	start=[0,0,0.5]
	gen.global_fitness=0
	for i in range(gen.nb_indi):
		publish_genes(gen.individuals[i].genes)
		gen.individuals[i].fitness=get_fitness(start)
		gen.global_fitness+= gen.individuals[i].fitness
	gen.global_fitness/=gen.nb_indi
	return gen

def get_fitness(start):
    
	publish_pos(start)
	rospy.sleep(0.5)
	clear_map()
	path_plan=PathPlanning(start,[start[0],start[0],start[0]+3])

	
	while path_plan.graph.map_.OG_map is None:
		rate.sleep()
	publish_genes
	path_plan.create_path()
	while(path_plan.run):
		pass
	fitness=path_plan.steps
	return fitness
    	


def publish_genes(genes):
	msg_genes=Float32MultiArray()
	msg_genes.data=genes
	genes_pub.publish(msg_genes)

def clear_map():
	rospy.wait_for_service('/octomap_server/reset')
	try:
		clear_service = rospy.ServiceProxy('/octomap_server/reset', Empty)
		response = clear_service()
		print(response)
	except rospy.ServiceException as e:
		rospy.logerr("Service call failed: " + str(e))

	print('clear map')


    

if __name__ == '__main__':
	try:

		pos_pub = rospy.Publisher('/gazebo_coordinate', Point, queue_size=10)
		genes_pub = rospy.Publisher('/current_genes', Float32MultiArray, queue_size=10)
	
		rospy.init_node('training', anonymous=True)
		rospy.loginfo("training initalization")
		train=Training()
		rospy.loginfo("training start")
		doTraining(train)
		
		rate=rospy.Rate(10)



		while not rospy.is_shutdown():
			rospy.spin()
			rate.sleep()
	except rospy.ROSInterruptException:
		pass

