#!/usr/bin/env python

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
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        
        self.box_size = 10 
        self.occupied_threshold = 3 
        self.padding = 5 

        self.map_ready = False
        self.start_ready = False
        self.goal_ready = False

        self.map_resolution = None
        self.map_data = None
        self.map_dimensions = None
        self.dmap_width = None 
        self.dmap_height= None 

        self.start_point = None
        self.goal_point = None

    def map_cb(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_orientation = msg.info.origin.orientation 
        self.map_position = msg.info.origin.position
        self.map_dimensions = (msg.info.height, msg.info.width)

        map_2d = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        map_2d_copy = map_2d.copy()

        for row in range(self.padding, msg.info.height - self.padding):
            for col in range(self.padding, msg.info.width - self.padding):
                if map_2d_copy[row, col] > self.occupied_threshold:
                    for i in range(-self.padding, self.padding + 1):
                        for j in range(-self.padding, self.padding + 1):
                            map_2d[row+i, col+j] = 100

        map_2d[map_2d == -1] = 100

        discretized_map_2d = np.zeros((msg.info.height//self.box_size, msg.info.width//self.box_size))
        for row in range(0, msg.info.height-self.box_size+1, self.box_size):
            for col in range(0, msg.info.width-self.box_size+1, self.box_size):
                avg = np.average(map_2d[row:row + self.box_size, col:col + self.box_size])
                discretized_map_2d[row//self.box_size, col//self.box_size] = avg
        
        self.map_data = discretized_map_2d
        self.dmap_height, self.dmap_width = self.map_data.shape
        self.map_ready = True
        self.plan_path(self.start_point, self.goal_point, self.map_data)

    def odom_cb(self, msg):
        self.start_point = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.start_ready = True
        self.plan_path(self.start_point, self.goal_point, self.map_data)

    def goal_cb(self, msg):
        self.goal_point = np.array([msg.pose.position.x, msg.pose.position.y])
        self.goal_ready = True
        self.plan_path(self.start_point, self.goal_point, self.map_data)

    def plan_path(self, start_point, goal_point, map):
        if self.map_ready and self.start_ready and self.goal_ready:
            self.goal_ready = False

            discretized_start = self.xy_to_discretized(start_point)
            discretized_goal = self.xy_to_discretized(goal_point)

            uv_path = self.a_star(discretized_start, discretized_goal)

            if uv_path is not None:
                xy_path = []
                self.trajectory.clear()
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
        
        pixel = (np.dot(rot_mat, coord) + np.array([self.map_position.x, self.map_position.y])) / self.map_resolution
        discretized = (int(pixel[1]//self.box_size), int(pixel[0]//self.box_size))

        return discretized

    def discretized_to_xy(self, coord):
        quat_inverse = tf.transformations.quaternion_inverse([self.map_orientation.x, self.map_orientation.y, self.map_orientation.z, self.map_orientation.w])
        rot_mat = tf.transformations.quaternion_matrix(quat_inverse)
        rot_mat = np.array([[rot_mat[0,0], rot_mat[0,1]], [rot_mat[1,0], rot_mat[1,1]]])

        xy_untranslated = np.array([coord[1], coord[0]]) * self.box_size * self.map_resolution
        xy_translated = xy_untranslated - np.array([self.map_position.x, self.map_position.y])
        xy = np.dot(rot_mat, xy_translated)

        return tuple(xy)

    def heur(self, node, end_point):
        dist = np.sqrt((node[0] - end_point[0])**2 + (node[1] - end_point[1])**2)
        return dist

    def cost(self, curr, node):
        if self.map_data[node[0],node[1]] < self.occupied_threshold:
            return heur(curr, node)
        else
            return np.inf

    def a_star(self, start, goal):
        open = {start} 
        segments = {} 
        cost = {start: 0} 
        score = {start: heur(start, goal)} 

        while open:
            curr = min(open, key=score.get)
            if curr == goal:
                return self.get_path(segments, curr)
            open.remove(curr)
            for node in self.get_neighbors(curr):
                node = tuple(node)
                temp_cost = cost.get(curr, np.inf) + cost(curr, node)
                if temp_cost < cost.get(node, np.inf):
                    segments[node] = curr
                    cost[node] = temp_cost
                    score[node] = temp_cost + heur(node, goal)
                    if node not in open:
                        open.add(node)
    
    def get_neighbors(self, node):
        neighbors = []
        for i in [-1,0,1]:
            for j in [-1,0,1]:
                neighbor = (node[0] + i, node[1] + j)
                if self.map_data[neighbor[1]][neighbor[0]] == 0 and neighbor[0] >= 0 and neighbor[1] >= 0 and neighbor[0] < self.map_dimensions[1] and neighbor[1] < self.map_dimensions[0]:
                    neighbors.append(neighbor)
        return neighbors

    def get_path(self, segments, node):
        path = [node]
        while node in segments.keys():
            node = segments[node]
            path.append(node)
        path.reverse()
        return path

if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
