#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic         = rospy.get_param("~odom_topic")
        self.lookahead          = rospy.get_param("~lookahead")
        self.lookahead_increase = rospy.get_param("~lookahead_increase")
        self.speed              = rospy.get_param("~speed")
        self.wheelbase_length   = rospy.get_param("~wheelbase_length")
        self.P_gain             = 2.0
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

    def odom_callback(self, odom):
        if (len(self.trajectory.points) == 0):
            rospy.loginfo("Cannot see trajectory yet")
            return

        # get car's position
        point_car = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])
        theta = euler_from_quaternion([
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        ])[2]

        # calculate closes points for each segments
        points = np.array(self.trajectory.get_points())
        dis_to_seg = []
        for i in range(len(points) - 1):
            # rospy.logerr("in here")
            dis_to_seg.append(self.__distance_squared_to_seg(point_car, points[i], points[i+1]))
            
        dis_to_seg = np.array(dis_to_seg)
        assert(len(dis_to_seg) > 0)
        # calculate the lookahead point
        lookahead = self.find_lookahead_point(dis_to_seg, point_car)

        # calculate the steering angle
        ## find distance between car and lookahead
        lookahead_vec = lookahead - point_car
        distance = np.linalg.norm(lookahead_vec)

        ## find alpha
        car_unit_vec = np.array([np.cos(theta), np.sin(theta)])
        lookahead_unit_vec = lookahead_vec / distance
        dot_product = np.dot(car_unit_vec, lookahead_unit_vec)
        # rospy.logerr(car_unit_vec)
        # rospy.logerr(lookahead_unit_vec)
        # rospy.logerr(dot_product)
        alpha = np.arccos(dot_product)

        # steering angle
        # steer_ang = np.arctan(2*self.wheelbase_length*np.sin(alpha)
        #                         / (self.P_gain * self.speed))
        steer_ang = np.arctan(2*self.wheelbase_length*np.sin(alpha)
                        / (distance))


        # publish drive commands
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0
        drive_msg.drive.steering_angle = steer_ang
        drive_msg.drive.acceleration = 0
        drive_msg.drive.steering_angle_velocity = 0
        drive_msg.drive.jerk = 0
        drive_msg.header.stamp = rospy.Time.now()
        self.drive_pub.publish(drive_msg)

    def find_lookahead_point(self, dis_to_seg, point_car):
        '''
        Find the lookhead point. 
        point_car: center of the circle
        dis_to_seg: np array of closest distance to each segment
        Idea from: https://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm/86428#86428
        '''
        points = np.array(self.trajectory.get_points())
        Q = point_car           # center of circle
        r = self.lookahead      # radius of circle
        
        seg_index = np.argmin(dis_to_seg) # closest segment
        
        # increase radius until the circle intersect the closest segment
        while (r < dis_to_seg[seg_index]):
            r += self.lookahead_increase
        
        # find the segment intersect with circle 
        # (solve edge case: when circle intersect 2 or more segments,
        #                   get the farthest segment)
        while dis_to_seg[seg_index] < r:
            seg_index += 1
            if seg_index >= len(dis_to_seg) - 1:
                break
        seg_index -= 1

        # get the intersect point
        P1 = points[seg_index]              # Start of line segment
        V  = points[seg_index + 1] - P1     # Vector along line segment

        a  = np.dot(V, V)
        b  = 2*np.dot(V, P1 - Q)
        disc = -1
        while disc < 0:
            c  = np.dot(P1, P1) + np.dot(Q, Q) - 2*np.dot(P1, Q) - r**2
            disc = b**2 - 4*a*c
            if disc < 0: # segment is too far, increase radius
                r += self.lookahead_increase
        
        sqrt_disc = np.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)

        # rospy.logerr(P1)
        # rospy.logerr(dis_to_seg[seg_index])

        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            assert False, "t1 is not correct, should never happen"
            return None
        
        return P1 + t1*V if t1 > t2 else P1 + t2*V


    @staticmethod
    def __distance_squared_to_seg(point, seg_1, seg_2):
        '''
        Find the closest distance squared from a point to a line segment
        Idea from: https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment/1501725#1501725
        '''
        l2 = np.linalg.norm(seg_1-seg_2)**2
        if (l2 == 0.0): return np.linalg.norm(l2 - seg_1)
        t = max(0, min(1, np.dot(point - seg_1, seg_2 - seg_1) / l2))
        projection = seg_1 + t*(seg_2 - seg_1)
        return np.linalg.norm(point - projection)


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
