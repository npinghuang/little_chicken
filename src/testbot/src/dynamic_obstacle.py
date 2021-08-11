#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, math, tf
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32, QuaternionStamped, Quaternion, TwistWithCovariance
from tf.transformations import quaternion_from_euler

#ROS教學改編的動態障礙物程式，只是應急用，沒有寫的很好
def publish_obstacle_msg():
  pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  rospy.init_node("test_obstacle_msg")

  x_0 = 0.3
  y_0 = 1.5
  vel_x = 0.1
  vel_y = 0.0
  # range_y = 6.0
  p_radius = 0.135

  last_x = x_0

  obstacle_msg = ObstacleArrayMsg() 
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "map" # CHANGE HERE: odom/map
  
  # Add point obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[0].id = 99

  obstacle_msg.obstacles[0].radius = p_radius
  obstacle_msg.obstacles[0].orientation.w = 1.0

  obstacle_msg.obstacles[0].velocities.twist.linear.x = vel_x
  obstacle_msg.obstacles[0].velocities.twist.linear.y = vel_y
  obstacle_msg.obstacles[0].velocities.twist.linear.z = 0
  obstacle_msg.obstacles[0].velocities.twist.angular.x = 0
  obstacle_msg.obstacles[0].velocities.twist.angular.y = 0
  obstacle_msg.obstacles[0].velocities.twist.angular.z = 0

  r = rospy.Rate(30) # 10hz
  t = 0.0
  while not rospy.is_shutdown():

    if (t>=7):
      t = 0.0
      x_0 = obstacle_msg.obstacles[0].polygon.points[0].x + p_radius
      # vel_x = -vel_x
      vel_x = 0

    # Vary y component of the point obstacle
    x_now = x_0 + (vel_x*t)
    obstacle_msg.obstacles[0].polygon.points = [Point32(), Point32(), Point32(), Point32()]
    obstacle_msg.obstacles[0].polygon.points[0].x = x_now - p_radius
    obstacle_msg.obstacles[0].polygon.points[0].y = y_0 - p_radius
    obstacle_msg.obstacles[0].polygon.points[1].x = x_now - p_radius
    obstacle_msg.obstacles[0].polygon.points[1].y = y_0 + p_radius
    obstacle_msg.obstacles[0].polygon.points[2].x = x_now + p_radius
    obstacle_msg.obstacles[0].polygon.points[2].y = y_0 + p_radius
    obstacle_msg.obstacles[0].polygon.points[3].x = x_now + p_radius
    obstacle_msg.obstacles[0].polygon.points[3].y = y_0 - p_radius

    obstacle_msg.obstacles[0].velocities.twist.linear.x = vel_x

    t = t + 0.033
    rospy.loginfo(obstacle_msg.obstacles[0].polygon.points[0].x)
    
    pub.publish(obstacle_msg)
    
    r.sleep()


if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass