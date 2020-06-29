#!/usr/bin/env python

import rospy
from math import cos, sin, atan, pi
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import pdb

pub = rospy.Publisher('pid_error', Float64, queue_size=10)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = 0.1
MAX_DISTANCE = 30.0
MIN_ANGLE = -45.0
MAX_ANGLE = 225.0
a = 0.0
b = 0.0
al = 0.0
bl = 0.0
ar = 0.0
br = 0.0

# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle):
  # TODO: implement
  ranges = np.asarray(data.ranges)
  angle_index = (angle + 45) * 4  # hokuyo ust-10lx fen bian lv 0.25du
  output_range = ranges[angle_index]
  return output_range

# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data, desired_distance):
  # TODO: implement
  global a, b
  L = 0.015 #old: 0.025
  desired_distance = desired_distance
  a = getRange(data, 135)
  b = getRange(data, 180)
  theta = 45 * pi / 180
  alpha = atan((a * cos(theta) - b) / (a * sin(theta)))
  current_dist = b * cos(alpha)
  next_dist = current_dist + L * sin(alpha)
  error_t = -(current_dist - desired_distance + L * sin(alpha))

  # pass the error_t term into some function and output the next_angle and velocity
  return error_t


# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):
  # TODO: implement
  global a, b
  L = 0.025
  desired_distance = desired_distance
  a = getRange(data, 45)
  b = getRange(data, 0)
  theta = 45 * pi / 180
  alpha = atan((a * cos(theta) - b) / (a * sin(theta)))
  current_dist = b * cos(alpha)
  next_dist = current_dist + L * sin(alpha)
  error_t = -(current_dist - desired_distance + L * sin(alpha))

  # pass the error_t term into some function and output the next_angle and velocity
  return error_t


# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):
  # TODO: implement
  global al, bl, ar, br
  L = 0.025
  al = getRange(data, 135)
  bl = getRange(data, 180)
  ar = getRange(data, 0)
  br = getRange(data, 45)
  theta = 45 * pi / 180
  alpha_l = atan((al * cos(theta) - bl) / (al * sin(theta)))
  alpha_r = atan((ar * cos(theta) - br) / (ar * sin(theta)))
  left_dist = bl * cos(alpha_l)
  right_dist = br * cos(alpha_r)
  desired_distance = (left_dist + right_dist) / 2.0
  error_t = -(right_dist - desired_distance + L * sin(alpha_r))

  # pass the error_t term into some function and output the next_angle and velocity
  return error_t


# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):
  error = followCenter(data) # TODO: replace with followLeft, followRight, or followCenter

  msg = Float64()
  msg.data = error
  pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_error_node', anonymous = True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()
