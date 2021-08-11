#!/usr/bin/env python
# import __future__
import rospy
import math
# from astar_nav.srv import *
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from itertools import chain
import numpy as np
import csv
pi = math.pi

class pose():
    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __eq__(self, other): 
            if not isinstance(other, pose):
                # don't attempt to compare against unrelated types
                return NotImplemented

            return self.x == other.x and self.y == other.y

class pathTracker():
    def __init__(self):
        # self.pose_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.poseCallback)
        # self.pose_sub = rospy.Subscriber("robot_pose", Twist, self.poseCallback)
        self.pose_sub = rospy.Subscriber("/odom", Odometry, self.poseCallback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # self.goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goalCallback)
        self.tf_listener = tf.TransformListener()
        # pos feedback from localization
        self.curPos = pose(0,0,0)
        self.goalPos = pose(0,0,0)
        self.path = Path()
        self.globalPath = []
        self.xy_tolerance = 0
        self.sub_xy_tolerance = 0.05
        self.theta_tolerance = 0.1
        self.linear_velocity = 1
        self.max_vel = 1
        self.min_vel = 0.6
        self.angular_velocity = 0
        # pure pursuit
        self.d_lookahead = 0.8
        self.d_lookahead_const = 0.5
        self.d_lookahead_K = 0.01
        self.d_lookahead_max = 1
        self.init_goal = pose(0,0,0)
        self.localgoal = pose(0,0,0)
        self.startTime = 0
        self.d_t = 0.1


    def start(self):
        while 1:
            if self.tf_listener.canTransform('base_footprint', 'map', rospy.Time(0)):
                break
        self.globalPath = self.getPath()
        print("global path received !")
        self.goalPos = self.globalPath[-1]
        # find the closest point on global path, set as first local goal
        min = 1000000000000000
        for i in self.globalPath:
            d = self.distance(self.curPos, i)
            if d < min:
                min = d
                self.init_goal = i
        
        self.show_pose(self.goalPos)
        self.localgoal = self.init_goal
        self.controller()
    
    def theta_convert(self, input):
        # convert rad domain to [-pi, pi]
        if input >=0:
            input = math.fmod(input, 2*pi)
            if input > pi:
                input -= 2*pi
                output = input
            else:
                output = input
        else:
            input *= -1
            input = math.fmod(input, 2*pi)
            if input > pi:
                input -= 2*pi
                output = input*-1
            else:
                output = input*-1
        return output

    def find_localgoal(self, cur_center, R, globalPath):
        # find the localgoal in interval [a, b], then interpolate
        k = 1
        lastk = 0
        a = pose(0,0,0)
        b = pose(1000,1000,0)
        count = 0
        for i in globalPath:
            if count == 1:
                lastk = 0 
            lastk = k
            d = self.distance(i, cur_center)
            if d >= R:
                k = 1
            else:
                k = 0

            deltak = k - lastk
            if deltak == 1:
                b = i
                break
            count +=1
        
        if b not in globalPath:
            min = 1000000000000000
            for i in globalPath:
                d = self.distance(cur_center, i)
                if d < min:
                    min = d
                    b = i

        a = globalPath[globalPath.index(b)-1]
        dis = [self.distance(a, cur_center), self.distance(b, cur_center)]
        # print ("dis"), dis
        x = [a.x, b.x]
        y = [a.y, b.y]
        localgoal_x = np.interp(R, dis, x)
        localgoal_y = np.interp(R, dis, y)    
        localgoal = pose(localgoal_x, localgoal_y, a.theta)
        # self.show_pose(a)
        # self.show_pose(b)
        return localgoal

    def show_pose(self,pos):
        print (pos.x, pos.y, pos.theta)

    def path_client(self, curPos, goalPos):
        cur_pos = Pose2D()
        goal_pos = Pose2D()
        cur_pos.x = curPos.x
        cur_pos.y = curPos.y
        cur_pos.theta = curPos.theta
        goal_pos.x = goalPos.x
        goal_pos.y = goalPos.y
        goal_pos.theta = goalPos.theta
        rospy.wait_for_service('path')
        try:
            path = rospy.ServiceProxy('path', pathTracking)
            respond_path = path(cur_pos, goal_pos)
            return respond_path
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def getPath(self):
        # nd = np.genfromtxt('/home/nvidia/catkin_ws/src/path_folder/output_point2_3.csv', delimiter=',', skip_header=True)
        # l = nd.tolist()
        # l = [0.93, 1.5, -1.5707963]
        p = pose(0.93, 1.5, -1.5707963)
        L = [p]
        # for i in l:
        #     p = pose(i[0], i[1], i[2])
        #     # p =self.map2real(p)
        #     L.append(p)
        return L

    def map2real(self, mapcoord):
        # convert map coordinate to world coordinate
        # mapcoord = [x, y]
        mapx = mapcoord.x
        mapy = mapcoord.y
        #origin on map 
        orgx = 15.555
        orgy = 14.765
        res = self.distance(pose(0,0,0), pose(-0.005, -0.3899,0))/self.distance(pose(orgx, orgy,0), pose(15.95,14.765,0))
        # realx = (mapx - orgx)/res
        # realy = (mapx - orgx)/res
        realx = mapy - orgy
        realy = -(mapx - orgx)
        return pose(realx, realy, self.theta_convert(mapcoord.theta - pi/2))
 
    def poseCallback(self, data):
        self.curPos.x = data.pose.pose.position.x
        self.curPos.y = data.pose.pose.position.y
        quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                      data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.curPos.theta = self.theta_convert(yaw)

    def distance(self, curPos, goalPos):
        dis = math.sqrt(pow((curPos.x - goalPos.x),2) + pow((curPos.y - goalPos.y),2))
        return dis

    def xy_goalReached(self,curPos, goalPos, tol):
        if self.distance(curPos, goalPos) < tol:
            return True
        else:
            return False

    def theta_goalReached(self,curPos, goalPos):
        if abs(self.theta_convert(goalPos.theta - curPos.theta)) < self.theta_tolerance:
            return True
        else:
            return False

    def controller(self):
        curGoal = self.localgoal
        print("current subgoal :"), [curGoal.x, curGoal.y]
        self.startTime = rospy.Time.now().to_sec()
        lap = 1
        while not self.xy_goalReached(self.curPos, self.goalPos, self.xy_tolerance) and not rospy.is_shutdown():
            precurGoal = curGoal
            prev_vel = self.linear_velocity

            # find localgoal
            # self.d_lookahead = self.linear_velocity * self.d_lookahead_K + self.d_lookahead_const
            # if self.d_lookahead > self.d_lookahead_max
            #     self.d_lookahead = self.d_lookahead_max

            #######################
            # Tune parameter here #
            #######################

            # if self.distance(self.curPos, pose(15.555,14.765,0)) < 1.5 and lap==1:
            #     # when near start point, large curvature
            #     self.linear_velocity = 0.5
            #     self.d_lookahead = 0.35
            # else:
            #     # v = 1, d_lookahead = 0.8  // 0.8 - 0.65 //
            #     self.linear_velocity =0.5 #0.5
            #     self.d_lookahead = 0.6# 0.6
            #     lap +=1
            
            self.linear_velocity = 0.2 #0.5
            self.d_lookahead = 0.6# 0.6
            
            print("---------------")
            print ("curpos :"), (self.curPos.x, self.curPos.y, self.curPos.theta)
            curGoal = self.find_localgoal(self.curPos, self.d_lookahead, self.globalPath)

            # transform local goal to base frame
            curGoal_base_y = -math.sin(self.curPos.theta)*(curGoal.x-self.curPos.x) + math.cos(self.curPos.theta)*(curGoal.y-self.curPos.y)

            # curGoal_tf = self.map2real(curGoal) # for coordinate transform due to line 251 tf, now we are at gazebo
            
            # ps = PoseStamped()
            # ps.header.seq = 0
            # ps.header.stamp = rospy.Time.now()
            # ps.header.frame_id = 'map'
            # ps.pose.position.x = curGoal_tf.x
            # ps.pose.position.y = curGoal_tf.y
            # ps.pose.position.z = 0

            # self.tf_listener.waitForTransform('base_footprint', ps.header.frame_id, ps.header.stamp, rospy.Duration(3.0))
            # curGoal_base = self.tf_listener.transformPose('base_footprint', ps)
            # curGoal_base = pose(curGoal_base.pose.position.x, curGoal_base.pose.position.y, 0)
            
            L = self.distance(self.curPos, curGoal)
            # R = pow(L, 2)/2/curGoal_base.y
            R = pow(L, 2)/2/curGoal_base_y

            self.angular_velocity = self.linear_velocity/R
            self.velOutput(self.linear_velocity, self.angular_velocity)
            # print("curGoal_base :"), [curGoal_base.x, curGoal_base.y]
            print("current subgoal :"), [curGoal.x, curGoal.y]
            print("v :"), self.linear_velocity
            print("w :"), self.angular_velocity
        self.velOutput(0, 0)

    def velOutput(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = w
        # print("current v, w ="), v, w
        self.vel_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit', anonymous = True)
    rate = rospy.Rate(10)
    path_tracker = pathTracker()
    path_tracker.start()
    rospy.spin()
