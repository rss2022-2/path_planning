#!/usr/bin/env python

from curses.textpad import rectangle
from operator import index
import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
from numpy import inf
import math

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
    
        self.map_msg = None

        self.box_size = 1 
        self.occupied_threshold = 3 
        self.padding = 10

        self.map_ready = False
        self.start_ready = False
        self.goal_ready = False

        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_data = None
        self.dmap_width = None # width of discretized map
        self.dmap_height= None # height of discretized map

        self.start_point = None
        self.goal_point = None


    def map_cb(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_orientation = msg.info.origin.orientation #Quaternion
        self.map_position = msg.info.origin.position #Point
        self.map_msg = msg

    def discretize_map(self, height, width, data):
        data[data == -1] = 100

        map_2d = data.reshape((height, width))
        map_2d_copy = map_2d.copy()
        for row in range(self.padding, height - self.padding):
            for col in range(self.padding, width - self.padding):
                if (736 < row < 780 and 831 < col < 862):
                    rospy.loginfo("don't pad the pole in stata") 
                elif map_2d_copy[row, col] > self.occupied_threshold:
                    map_2d[row+self.padding, col+self.padding] = 100
                    map_2d[row+self.padding, col-self.padding] = 100
                    map_2d[row-self.padding, col+self.padding] = 100
                    map_2d[row-self.padding, col-self.padding] = 100

        discretized_map_2d = np.zeros((height//self.box_size, width//self.box_size))
        if (self.box_size == 1):
            discretized_map_2d = map_2d
        else:
            for row in range(0, height-self.box_size+1, self.box_size):
                for col in range(0, width-self.box_size+1, self.box_size):
                    avg = np.average(map_2d[row:row + self.box_size, col:col + self.box_size])
                    discretized_map_2d[row//self.box_size, col//self.box_size] = avg
        
        return discretized_map_2d

    def odom_cb(self, msg):
        start_x = msg.pose.pose.position.x
        start_y = msg.pose.pose.position.y
        self.start_point = np.array([start_x, start_y])
        self.start_ready = True

    def goal_cb(self, msg):
        if not self.map_ready:
            self.map_data = self.discretize_map(self.map_height, self.map_width, np.array(self.map_msg.data))
            self.dmap_height, self.dmap_width = self.map_data.shape

            rospy.loginfo(np.unique(self.map_msg.data))
            self.map_ready = True
            
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.goal_point = np.array([goal_x, goal_y])

        self.goal_ready = True
        self.plan_path(self.start_point, self.goal_point, self.map_data)

    def plan_path(self, start_point, goal_point, map):
        if self.map_ready and self.start_ready and self.goal_ready:
            discretized_start = self.xy_to_discretized(start_point)
            discretized_goal = self.xy_to_discretized(goal_point)

            uv_path = self.A_star(discretized_start, discretized_goal)

            xy_path = []
            for coord in uv_path:
                xy_coord = self.discretized_to_xy(coord)
                xy_path.append(xy_coord)

                point = Point()
                point.x = xy_coord[0]
                point.y = xy_coord[1]
                self.trajectory.addPoint(point)

            self.traj_pub.publish(self.trajectory.toPoseArray())
            self.trajectory.publish_viz()

    def xy_to_discretized(self, coord):
        rot_mat = tf.transformations.quaternion_matrix([self.map_orientation.x, self.map_orientation.y, self.map_orientation.z, self.map_orientation.w])
        rot_mat = np.array([[rot_mat[0,0], rot_mat[0,1]], [rot_mat[1,0], rot_mat[1,1]]])
        pixel = (np.dot(rot_mat, coord) + np.array([self.map_position.x, self.map_position.y]))/self.map_resolution
        return (int(pixel[1]//self.box_size), int(pixel[0]//self.box_size))

    def discretized_to_xy(self, coord):
        quat_inverse = tf.transformations.quaternion_inverse([self.map_orientation.x, self.map_orientation.y, self.map_orientation.z, self.map_orientation.w])
        rot_mat = tf.transformations.quaternion_matrix(quat_inverse)
        rot_mat = np.array([[rot_mat[0,0], rot_mat[0,1]], [rot_mat[1,0], rot_mat[1,1]]])

        xy_untranslated = np.array([coord[1], coord[0]]) * self.box_size * self.map_resolution
        xy_translated = xy_untranslated - np.array([self.map_position.x, self.map_position.y])
        
        return tuple(np.dot(rot_mat, xy_translated))

    def A_star(self, start, goal):
        heuristic_func = lambda start, goal: math.sqrt((start[0] - goal[0])**2 + (start[1] - goal[1])**2)
        cost_func = lambda node: 1 if self.map_data[node[0],node[1]] < self.occupied_threshold else np.inf
        
        open = {start} 
        segments = {} 
        cost = {start: 0} 
        score = {start: heuristic_func(start, goal)} 

        while open:
            curr = min(open, key=score.get)
            if curr == goal:
                return self.get_path(segments, curr)

            open.remove(curr)
            for node in self.get_neighbors(curr):
                node = tuple(node)
                temp_cost = cost.get(curr, np.inf) + cost_func(node)
                if temp_cost < cost.get(node, np.inf):
                    segments[node] = curr
                    cost[node] = temp_cost
                    score[node] = temp_cost + heuristic_func(node, goal)
                    if node not in open:
                        open.add(node)
    
    def get_neighbors(self, node):
        neighbors = np.array([
            (node[0]+1, node[1]+1),
            (node[0]+1, node[1]),
            (node[0]+1, node[1]-1),
            (node[0], node[1]+1),
            (node[0], node[1]-1),
            (node[0]-1, node[1]+1),
            (node[0]-1, node[1]),
            (node[0]-1, node[1]-1)])
        
        neighbors = neighbors[neighbors[:,0] >= 0]
        neighbors = neighbors[neighbors[:,0] < self.map_height]
        neighbors = neighbors[neighbors[:,1] >= 0]
        neighbors = neighbors[neighbors[:,1] < self.map_width]

        return neighbors

    def get_path(self, segments, node):
        path = [node]
        while node in segments.keys():
            node = segments[node]
            path.append(node)
        return path.reverse()

if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
