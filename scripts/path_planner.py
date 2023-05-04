#!/usr/bin/env python

import rospy
import math
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import copy






if __name__ == '__main__':
    # Create the ROS node
    rospy.init_node('path_planner')

    # Create a map from image
    map = Map()

    # Create a graph from the map
    graph = Graph(map)

    startx = rospy.get_param("~startx")
    starty = rospy.get_param("~starty")
    startz = 
    goalx = 1
    goaly = 1
    goalz = 5
    
    # Do the graph search
    graph_search = GraphSearch(graph, [startx, starty, startz], [goalx, goaly, goalz])

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

            print("Plan finished! Click a new goal in rviz 2D Nav Goal.")


    # Loop forever while processing callbacks
    rospy.spin()
