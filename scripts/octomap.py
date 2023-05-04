#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

class Frontier:
    def __init__(self,pos,width,height,depth):
        self.pos=pos
        self.width=width
        self.height=height
        self.depth=depth
        self.neighbors=self.get_neighbors()

    def get_neighbors(self):
        neighbors=[]
        for i in range(-1,2):
            for j in range(-1,2):
                for k in range(-1,2):
                    if self.pos[0]+i>=0 and self.pos[1]+j>=0 and self.pos[2]+k>=0 and self.pos[0]+i<self.width and self.pos[1]+j<self.height and self.pos[2]+k<self.depth:
                        neighbors.append([self.pos[0]+i,self.pos[1]+j,self.pos[2]+k])
        return neighbors

class Cluster:
    def __init__(self,all_frontiers,all_frontiers_pos):
        self.element_frontier=[]
        self.all_frontiers=all_frontiers
        self.all_frontiers_pos=all_frontiers_pos
    
    def get_frontiers_cluster(self,frontier_element):
        self.element_frontier.append(frontier_element.pos) # append the current element to the cluster
        if len(self.all_frontiers_pos)!=0 and frontier_element.pos in self.all_frontiers_pos:
            idx=self.all_frontiers_pos.index(frontier_element.pos) #get the index of the current element
            
            # delete the current element from the lists
            self.all_frontiers_pos.pop(idx)
            self.all_frontiers.pop(idx)

            for front in frontier_element.neighbors: #loop for every neighbor
                if front in self.all_frontiers_pos: #check for every neighbors if it is a frontier
                    idx=self.all_frontiers_pos.index(front)
                    self.get_frontiers_cluster(self.all_frontiers[idx]) #recursive call of the function
                    #rospy.logerr("end of rec")
            

def generate_cluster(frontier,frontier_pos):
    cluster_lst=[]
    while len(frontier)!=0:
        cluster=Cluster(frontier,frontier_pos)
        
        cluster.get_frontiers_cluster(frontier[0])
        cluster_lst.append(cluster)

        #update the two lists
        frontier=cluster.all_frontiers
        frontier_pos=cluster.all_frontiers_pos
        rospy.logerr(len(cluster.element_frontier))

def octomap_grid_callback(msg):
    arr=np.array(msg.data)
    width=msg.info.width
    height=msg.info.height
    depth=len(msg.data)/(width*height)
    new_occ=np.reshape(arr,(width,height,depth))
    frontier_pos=[]
    frontier=[]
    for z in range(depth):
        for y in range(height):
            for x in range(depth):
                
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
                        frontier_pos.append([x,y,z])
                        frontier.append(Frontier([x,y,z],width,height,depth))
                                            
    rospy.loginfo(len(frontier))
    generate_cluster(frontier,frontier_pos)





if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('get_occupancy_grid')

    # Subscribe to the octomap binary topic
    sub = rospy.Subscriber('occupancy_grid', OccupancyGrid, octomap_grid_callback)

    # Publish the occupancy grid to a topic
    pub = rospy.Publisher('frontier', OccupancyGrid, queue_size=1)

    loop_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.spin()
        loop_rate.sleep()

