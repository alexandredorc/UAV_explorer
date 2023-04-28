#!/usr/bin/env python

import rospy
import math
import cv2 as cv # OpenCV2
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import copy
import random


class Node:
    def __init__(self, x, y, idx):

        # Index of the node in the graph
        self.idx = idx

        # Position of node
        self.x = x
        self.y = y
        self.z = z

        # Neighbouring edges
        self.neighbours = [] # List of nodes
        self.neighbour_costs = [] # the ith neighbour in self.neighbours has an edge cost defined as the ith element in self.neighbour_costs

        # Search variables
        self.cost = 9999999 # A large number
        self.parent_node = None # Invalid parent

    def distance_to(self, other_node):
        return math.sqrt((self.x-other_node.x)**2 + (self.y-other_node.y)**2 + (self.z-other_node.z)**2)

    def is_connected(self, img, other_node):
        p1 = [self.x, self.y , self.z]
        p2 = [other_node.x, other_node.y, other_node.z]
        return not is_occluded(img, p1, p2)

class Graph:
    def __init__(self, map):

        self.map_ = map

        self.nodes_ = []

        self.grid_step_size_ = rospy.get_param("~grid_step_size") # Grid spacing
        ##self.prm_num_nodes_ = rospy.get_param("~prm_num_nodes") # Number of PRM nodes

        # Publishers
        self.path_pub_ = rospy.Publisher('/path_planner/plan', Path, queue_size=1)
        self.path_smooth_pub_ = rospy.Publisher('/path_planner/plan_smooth', Path, queue_size=1)

        # Visualisation Marker (you can ignore this)
        self.marker_nodes_ = Marker()
        self.marker_nodes_.header.frame_id = "map"
        self.marker_nodes_.ns = "nodes"
        self.marker_nodes_.id = 0
        self.marker_nodes_.type = Marker.POINTS
        self.marker_nodes_.action = Marker.ADD
        self.marker_nodes_.pose.position.x = 0.0
        self.marker_nodes_.pose.position.y = 0.0
        self.marker_nodes_.pose.position.z = 0.0
        self.marker_nodes_.pose.orientation.x = 0.0
        self.marker_nodes_.pose.orientation.y = 0.0
        self.marker_nodes_.pose.orientation.z = 0.0
        self.marker_nodes_.pose.orientation.w = 1.0
        self.marker_nodes_.scale.x = .03
        self.marker_nodes_.scale.y = .03
        self.marker_nodes_.scale.z = .03
        self.marker_nodes_.color.a = 1.0
        self.marker_nodes_.color.r = 1.0
        self.marker_nodes_.color.g = 0.2
        self.marker_nodes_.color.b = 0.2

        self.marker_start_ = Marker()
        self.marker_start_.header.frame_id = "map"
        self.marker_start_.ns = "start"
        self.marker_start_.id = 0
        self.marker_start_.type = Marker.POINTS
        self.marker_start_.action = Marker.ADD
        self.marker_start_.pose.position.x = 0.0
        self.marker_start_.pose.position.y = 0.0
        self.marker_start_.pose.position.z = 0.0
        self.marker_start_.pose.orientation.x = 0.0
        self.marker_start_.pose.orientation.y = 0.0
        self.marker_start_.pose.orientation.z = 0.0
        self.marker_start_.pose.orientation.w = 1.0
        self.marker_start_.scale.x = .08
        self.marker_start_.scale.y = .08
        self.marker_start_.scale.z = .08
        self.marker_start_.color.a = 1.0
        self.marker_start_.color.r = 1.0
        self.marker_start_.color.g = 1.0
        self.marker_start_.color.b = 0.2

        self.marker_visited_ = Marker()
        self.marker_visited_.header.frame_id = "map"
        self.marker_visited_.ns = "visited"
        self.marker_visited_.id = 0
        self.marker_visited_.type = Marker.POINTS
        self.marker_visited_.action = Marker.ADD
        self.marker_visited_.pose.position.x = 0.0
        self.marker_visited_.pose.position.y = 0.0
        self.marker_visited_.pose.position.z = 0.0
        self.marker_visited_.pose.orientation.x = 0.0
        self.marker_visited_.pose.orientation.y = 0.0
        self.marker_visited_.pose.orientation.z = 0.0
        self.marker_visited_.pose.orientation.w = 1.0
        self.marker_visited_.scale.x = .05
        self.marker_visited_.scale.y = .05
        self.marker_visited_.scale.z = .05
        self.marker_visited_.color.a = 1.0
        self.marker_visited_.color.r = 0.2
        self.marker_visited_.color.g = 0.2
        self.marker_visited_.color.b = 1.0

        self.marker_unvisited_ = Marker()
        self.marker_unvisited_.header.frame_id = "map"
        self.marker_unvisited_.ns = "unvisited"
        self.marker_unvisited_.id = 0
        self.marker_unvisited_.type = Marker.POINTS
        self.marker_unvisited_.action = Marker.ADD
        self.marker_unvisited_.pose.position.x = 0.0
        self.marker_unvisited_.pose.position.y = 0.0
        self.marker_unvisited_.pose.position.z = 0.0
        self.marker_unvisited_.pose.orientation.x = 0.0
        self.marker_unvisited_.pose.orientation.y = 0.0
        self.marker_unvisited_.pose.orientation.z = 0.0
        self.marker_unvisited_.pose.orientation.w = 1.0
        self.marker_unvisited_.scale.x = .06
        self.marker_unvisited_.scale.y = .06
        self.marker_unvisited_.scale.z = .06
        self.marker_unvisited_.color.a = 1.0
        self.marker_unvisited_.color.r = 0.3
        self.marker_unvisited_.color.g = 1.0
        self.marker_unvisited_.color.b = 0.3
        
        self.marker_edges_ = Marker()
        self.marker_edges_.header.frame_id = "map"
        self.marker_edges_.ns = "edges"
        self.marker_edges_.id = 0
        self.marker_edges_.type = Marker.LINE_LIST
        self.marker_edges_.action = Marker.ADD
        self.marker_edges_.pose.position.x = 0.0
        self.marker_edges_.pose.position.y = 0.0
        self.marker_edges_.pose.position.z = 0.0
        self.marker_edges_.pose.orientation.x = 0.0
        self.marker_edges_.pose.orientation.y = 0.0
        self.marker_edges_.pose.orientation.z = 0.0
        self.marker_edges_.pose.orientation.w = 1.0
        self.marker_edges_.scale.x = 0.008
        self.marker_edges_.scale.y = 0.008
        self.marker_edges_.scale.z = 0.008
        self.marker_edges_.color.a = 1.0
        self.marker_edges_.color.r = 1.0
        self.marker_edges_.color.g = 1.0
        self.marker_edges_.color.b = 0.4
        
        self.marker_pub_ = rospy.Publisher('marker', Marker, queue_size=1)
        
        # Select between grid or PRM

        ##use_prm = rospy.get_param("~use_prm")
        ##if use_prm:
            ##self.create_PRM()
        ##else:
        self.create_grid()
        
        #self.visualise_graph()

    def create_grid(self):

        # Create nodes
        idx = 0
        ## DONE
        for x in range(self.map_.min_x_, self.map_.max_x_-1, self.grid_step_size_):
            for y in range(self.map_.min_y_, self.map_.max_y_-1, self.grid_step_size_):
                for z in range(self.map_.min_z_, self.map_.max_z_-1, self.grid_step_size_):

                    if rospy.is_shutdown():
                        return

                    # Check if it is occupied
                    occupied = self.map_.is_occupied(x,y,z)

                    # Create the node
                    if not occupied:
                        self.nodes_.append(Node(x,y,z,idx))
                        idx = idx + 1

        # Create edges
        count = 0
        # distance_threshold = math.sqrt(2*(self.grid_step_size_*1.01)**2) # Chosen so that diagonals are connected, but not 2 steps away
        distance_threshold = self.grid_step_size_*1.01 # only 4 connected
        for node_i in self.nodes_:

            # Debug print status
            count = count + 1
            print(count, "of", len(self.nodes_))
            if rospy.is_shutdown():
                return

            for node_j in self.nodes_:

                # Don't create edges to itself
                if node_i != node_j:

                    # Check if the nodes are close to each other
                    distance = node_i.distance_to(node_j)
                    if distance < distance_threshold:

                        # Check edge is collision free
                        if node_i.is_connected(self.map_.image_, node_j):

                            # Create the edge
                            node_i.neighbours.append(node_j)
                            node_i.neighbour_costs.append(distance)


    def get_closest_node(self, xyz):
        # input: xy is a point in the form of an array, such that x=xy[0] and y=xy[1]. 
        # output: return the index of the node in self.nodes_ that has the lowest Euclidean distance to the point xy. 

        best_dist = 999999999 # A large number to begin with
        best_index = None # Index of best node found so far

        for i in range(len(self.nodes_)):
            newdist=(self.nodes_[i].x-xyz[0])**2+(self.nodes_[i].y-xyz[1])**2+(self.nodes_[i].z-xyz[2])**2
            if best_dist>newdist:
                best_dist=newdist
                best_index=i     

             # you can remove this line after you have filled in the above code

        return best_index

    def visualise_graph(self):
        # Create and publish visualisation markers for the graph

        self.marker_nodes_.points = []
        for node_i in self.nodes_:
            p = self.map_.pixel_to_world(node_i.x, node_i.y, node_i.z)
            point = Point(p[0], p[1], p[2])
            self.marker_nodes_.points.append(point)
        self.marker_pub_.publish(self.marker_nodes_)

        self.marker_edges_.points = []
        for node_i in self.nodes_:
            for node_j in node_i.neighbours:
                p = self.map_.pixel_to_world(node_i.x, node_i.y, node_i.z)
                point = Point(p[0], p[1], p[2])
                self.marker_edges_.points.append(point)
                p = self.map_.pixel_to_world(node_j.x, node_j.y, node_i.z)
                point = Point(p[0], p[1], p[2])
                self.marker_edges_.points.append(point)
        self.marker_pub_.publish(self.marker_edges_)

    def visualise_search(self, visited_set, unvisited_set, start_idx, goal_idx):
        # Visualise the nodes with these node indexes
        self.marker_visited_.points = []
        for i in visited_set:
            node_i = self.nodes_[i]
            p = self.map_.pixel_to_world(node_i.x, node_i.y, node_i.z)
            point = Point(p[0], p[1], p[2])
            self.marker_visited_.points.append(point)
        self.marker_pub_.publish(self.marker_visited_)

        self.marker_unvisited_.points = []
        for i in unvisited_set:
            node_i = self.nodes_[i]
            p = self.map_.pixel_to_world(node_i.x, node_i.y, node_i.z)
            point = Point(p[0], p[1], p[2])
            self.marker_unvisited_.points.append(point)
        self.marker_pub_.publish(self.marker_unvisited_)

        self.marker_start_.points = []
        node_i = self.nodes_[start_idx]
        p = self.map_.pixel_to_world(node_i.x, node_i.y, node_i.z)
        point = Point(p[0], p[1], p[2])
        self.marker_start_.points.append(point)

        node_i = self.nodes_[goal_idx]
        p = self.map_.pixel_to_world(node_i.x, node_i.y, node_i.z)
        point = Point(p[0], p[1], p[2])
        self.marker_start_.points.append(point)
        self.marker_pub_.publish(self.marker_start_)

    def visualise_path(self, path):
        msg = Path()
        msg.header.frame_id = 'world'
        for node in path:
            p = self.map_.pixel_to_world(node.x, node.y, node.z)
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = 'world'
            msg.poses.append(pose)
        self.path_pub_.publish(msg)

    def visualise_path_smooth(self, path):
        msg = Path()
        msg.header.frame_id = 'world'
        for node in path:
            p = self.map_.pixel_to_world(node.x, node.y, node.z)
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = 'world'
            msg.poses.append(pose)
        self.path_smooth_pub_.publish(msg)



class Map:
    def __init__(self):

        ## Extract the image from a file
        # put the subscribed grid

        shape = self.image_.shape
        self.min_x_ = 0
        self.min_y_ = 0
        self.min_z_ = 0
        self.max_x_ = shape[0]
        self.max_y_ = shape[1]
        self.max_z_ = shape[2]

        # Rviz subscriber
        ## change the goal 
        #self.rviz_goal_sub_ = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback, queue_size=1)
        #self.rviz_goal = None

    def pixel_to_world(self, x, y, z):
        resolution = 0.01
        return [x*resolution, y*resolution, z*resolution]

    def world_to_pixel(self, x, y, z):
        resolution = 0.01
        return [x/resolution, y/resolution, z/resolution]

    def is_occupied(self, x, y, z):

        shape = self.image_.shape

        # Out of bounds
        if x < 0 or x >= shape[0] or y < 0 or y >= shape[1] or z < 0 or z >= shape[2]:
            return True

        if self.image_[x,y,z] ==1:
            return False
        else:
            return True

def is_occluded(img, p1, p2, threshold=235):
    # Draws a line from p1 to p2
    # Stops at the first pixel that is a "hit", i.e. above the threshold
    # Returns True if a hit is found, False otherwise

    # Extract the vector
    x1 = float(p1[0])
    y1 = float(p1[1])
    z1 = float(p1[2])
    x2 = float(p2[0])
    y2 = float(p2[1])
    z2 = float(p2[2])

    step = 1.0

    dx = x2 - x1
    dy = y2 - y1
    dy = z2 - z1
    l = math.sqrt(dx**2. + dy**2. + dz**2.)
    if l == 0:
        return False
    dx = dx / l
    dy = dy / l
    dz = dz / l

    max_steps = int(l / step)

    for i in range(max_steps):

        # Get the next pixel
        x = int(round(x1 + dx*i))
        y = int(round(y1 + dy*i))
        z = int(round(y1 + dy*i))

        # Check if it's outside the image
        if x < 0 or x >= img.shape[0] or y < 0 or y >= img.shape[1] or z < 0 or z >= img.shape[2]:
            return False

        # Check for "hit"
        if img[x, y, z] <= threshold:
            return True

    # No hits found
    return False


class GraphSearch:
    def __init__(self, graph, start_xyz, goal_xyz):
        self.graph_ = graph

        self.heuristic_weight_ = rospy.get_param("~heuristic_weight")

        self.start_idx_ = self.graph_.get_closest_node(start_xyz)
        self.goal_idx_ = self.graph_.get_closest_node(goal_xyz)

        self.search(self.start_idx_, self.goal_idx_)

        self.path_ = self.generate_path(self.goal_idx_)
        self.graph_.visualise_path(self.path_)



    def search(self, start_idx, goal_idx):
        # Find a path from nodes_[start_idx] to nodes_[goal_idx]
        
        # Reset all parents and costs
        for n in self.graph_.nodes_:
            n.cost = 9999999 # a large number
            n.parent_node = None # invalid to begin with

        # Setup sets. These should contain indices (i.e. numbers) into the self.graph_.nodes_ array
        unvisited_set = []
        visited_set = []

        # Add start node to visited set
        unvisited_set.append(start_idx)
        self.graph_.nodes_[start_idx].cost = 0

        # Loop until solution found or graph is disconnected
        while len(unvisited_set) > 0:
            
            # Select a node
            # hint: self.get_minimum_cost_node(unvisited_set) will help you find the node with the minimum cost

            unvi_index=self.get_minimum_cost_node(unvisited_set)
            node_idx=unvisited_set[unvi_index]
            # Move the selected node from the unvisited_set to the visited_set

            visited_set.append(unvisited_set.pop(unvi_index))
            
            # Termination criteria
            # Finish early (i.e. "return") if the goal is found

            if self.goal_idx_==node_idx:
                rospy.loginfo("Goal found!")
                return
            # For each neighbour of the node
            for neighbour_idx in range(len(self.graph_.nodes_[node_idx].neighbours)):

                # For convenience, extract the neighbour and the edge cost from the arrays
                neighbour = self.graph_.nodes_[node_idx].neighbours[neighbour_idx]  
                neighbour_cost = self.graph_.nodes_[node_idx].neighbour_costs[neighbour_idx]

                # Check if neighbours is already in visited
                if neighbour.idx in visited_set:
                    
                    # Do nothing
                    pass
                
                else:

                    # Compute the cost of this neighbour node
                    # hint: cost = cost-of-previous-node + cost-of-edge + self.heuristic_weight_ * A*-heuristic-score
                    # hint: implement it without the heuristic-score first. once this is working, add the heuristic score.
                    # hint: neighbour.distance_to() function is likely to be helpful for the heuristic-score
                    cost=  self.graph_.nodes_[node_idx].cost+ neighbour_cost+(self.heuristic_weight_*neighbour.distance_to(self.graph_.nodes_[goal_idx]))
                
                    # Check if neighbours is already in unvisited set
                    if neighbour.idx in unvisited_set:

                        # If the cost is lower than the previous cost for this node
                        # Then update it to the new cost
                        # Also, update the parent pointer to point to the new parent 

                        if cost < neighbour.cost :
                            neighbour.parent_node = self.graph_.nodes_[node_idx]
                            neighbour.cost = cost

                    else:

                        # Add it to the unvisited set
                        unvisited_set.append(neighbour.idx)

                        # Initialise the cost and the parent pointer
                        # hint: this will be similar to your answer above

                        neighbour.parent_node = self.graph_.nodes_[node_idx]
                        neighbour.cost = cost
            
            # Visualise the current search status in RVIZ
            self.visualise_search(visited_set, unvisited_set, start_idx, goal_idx)
            #rospy.sleep(0.1) # Pause for easier visualisation
        
                   

    def get_minimum_cost_node(self, unvisited_set):
        # Find the node that has the minimum cost

        # There's more efficient ways of doing this...
        min_cost = 99999999
        min_idx = None
        for idx in range(len(unvisited_set)):
            cost = self.graph_.nodes_[unvisited_set[idx]].cost
            if cost < min_cost:
                min_cost = cost
                min_idx = idx
        return min_idx

    def generate_path(self, goal_idx):
        # Generate the path by following the parents from the goal back to the start

        path = []

        current = self.graph_.nodes_[goal_idx]
        while current.parent_node.cost != 0:
            path.append(current)
            current=current.parent_node
        path.append(current.parent_node)
        final_path=[]
        for i in range(0,len(path)):
            final_path.append(path[len(path)-i-1])
        return final_path


    def visualise_search(self, visited_set, unvisited_set, start_idx, goal_idx):
        self.graph_.visualise_search(visited_set, unvisited_set, start_idx, goal_idx)


class PathSmoother():
    def __init__(self, graph, path):
        self.graph_ = graph
        self.path_ = self.smooth_path(path)
        self.graph_.visualise_path_smooth(self.path_)

    def smooth_path(self, path_nodes):

        # Convert the path into a geometry_msgs.Point[]
        path = []

        for node in path_nodes:
            p = Point()
            p.x = node.x
            p.y = node.y
            p.z = node.z
            path.append(p)

        # Initialise the smooth path
        path_smooth = copy.deepcopy(path)

        alpha = rospy.get_param("~alpha")
        beta = rospy.get_param("~beta")

        # Loop until the smoothing converges
        # In each iteration, update every waypoint except the first and last waypoint
        e=1
        blocked=True
        while e>0.001 :
            e=0
            for i in range(1,len(path)-1):
                path[i].x=path_smooth[i].x
                path[i].y=path_smooth[i].y
                path_smooth[i].x+=alpha*path_nodes[i].x+beta*path[i-1].x+beta*path[i+1].x-(alpha+2*beta)*path[i].x
                path_smooth[i].y+=alpha*path_nodes[i].y+beta*path[i-1].y+beta*path[i+1].y-(alpha+2*beta)*path[i].y
                
                blocked = is_occluded(self.graph_.map_.image_, [path[i-1].x, path[i-1].y], [path[i].x, path[i].y])
                blocked += is_occluded(self.graph_.map_.image_, [path[i].x, path[i].y], [path[i+1].x, path[i+1].y])
                if blocked:
                    path_smooth[i].x=path[i].x
                    path_smooth[i].y=path[i].y

                
                e+=(path[i].x-path_smooth[i].x)**2+(path[i].y-path_smooth[i].y)**2
        return path




if __name__ == '__main__':
    # Create the ROS node
    rospy.init_node('path_planner')

    # Create a map from image
    map = Map()

    # Create a graph from the map
    graph = Graph(map)

    startx = rospy.get_param("~startx")
    starty = rospy.get_param("~starty")
    startz = rospy.get_param("~startz")
    goalx = rospy.get_param("~goalx")
    goaly = rospy.get_param("~goaly")
    goalz = rospy.get_param("~goalz")

    # Do the graph search
    graph_search = GraphSearch(graph, [startx, starty, startz], [goalx, goaly, goalz])

    # Smooth the path
    PathSmoother(graph, graph_search.path_)

    print("Plan finished! Click a new goal in rviz 2D Nav Goal.")

    # Re-plan indefinitely when rviz goals received
    while not rospy.is_shutdown():

        if map.rviz_goal is None:
            # Do nothing, waiting for goal
            rospy.sleep(0.01)
        else:

            # Extract the next goal
            startx = goalx
            starty = goaly
            startz = goalz

            goalx,goaly,goalz = map.rviz_goal
            map.rviz_goal = None # Clear it so a new goal can be set

            # Do the graph search
            graph_search = GraphSearch(graph, [startx, starty, startz], [goalx, goaly, goalz])

            # Smooth the path
            PathSmoother(graph, graph_search.path_)

            print("Plan finished! Click a new goal in rviz 2D Nav Goal.")


    # Loop forever while processing callbacks
    rospy.spin()
