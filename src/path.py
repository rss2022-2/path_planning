#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
from Queue import PriorityQueue

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)


    def map_cb(self, msg):
        self.orientation = msg.info.origin.orientation
        self.pos = msg.info.origin.position
        self.res = msg.info.resolution
        self.grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
      
    def odom_cb(self, msg):
        self.startx = msg.pose.position.x
        self.starty = msg.pose.position.y
        self.startz = msg.pose.position.z


    def goal_cb(self, msg):
        self.goalx = msg.pose.position.x
        self.goaly = msg.pose.position.y
        self.goalz = msg.pose.position.z
       
    def pix2coord(self,msg):
        self.resolution = msg.info.resolution
        
        r,p,y = tf.transformations.euler_from_quaternion([msg.info.origin.orientation.x,msg.info.origin.orientation.y,msg.info.origin.orientation.z,msg.info.origin.orientation.w])
        u, v = msg.info.origin.position.x, msg.info.origin.position.y
        
        rotation = np.array([[np.cos(y), -np.sin(y), u], [np.sin(y),  np.cos(y), v],[0,0,1]]) 
        x, y = np.

    def a_star(self, start_point, end_point):
        ## CODE FOR PATH PLANNING ##
        # referencing the doc given for A*
        frontier = PriorityQueue()
        frontier.put(start_point, 0)
        came_from = dict()
        cost_so_far = dict()
        came_from[start_point] = None
        cost_so_far[start_point] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break
   
            for next in graph.neighbors(current):
                new_cost = cost_so_far[current] + graph.cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

        return came_from, cost_so_far        
    
    def plane_path(self, start_point, end_point, map):
        #use a star to  get the ggeneral path
        path, cost = self.a_star(start_point, end_point)
        
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()

