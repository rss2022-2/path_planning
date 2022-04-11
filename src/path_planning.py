#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

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
        #copying sensor model map callback for now
        #so map is array
        self.map = np.array(msg.data, np.double)/100.
        self.map = np.clip(self.map, 0, 1)
        self.map_resolution = msg.info.resolution

        # Convert the origin to a tuple
        origin_p = msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((origin_o.x,origin_o.y,origin_o.z,origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        #maybe initialize the laser scan like in sensor model?

    def odom_cb(self, msg):
        self.startx = msg.pose.pose.position.x
        self.starty = msg.pose.pose.position.y
        self.startz = msg.pose.pose.position.z


    def goal_cb(self, msg):
        self.goalx = msg.pose.position.x
        self.goaly = msg.pose.position.y
        self.goalz = msg.pose.position.z

    def plan_path(self, start_point, end_point, map):
        ## CODE FOR PATH PLANNING ##
        

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()

