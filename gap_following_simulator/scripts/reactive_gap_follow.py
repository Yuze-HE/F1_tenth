#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, lidar_callback) #TODO
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1) #TODO

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        for i in range(180, 901):
            ranges[i] = np.mean(ranges[i-5:i+6])
        proc_ranges = ranges[180:901] + 0.01
        proc_ranges[proc_ranges > 3] = 3 #或者np.minmum(ranges, 3)

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        free_space_ranges = np.array(free_space_ranges)
        x = [free_space_ranges[1:], 0]
        y = x - free_space_ranges
        y = [0, y]
        free_space_ranges_indexs = []
        for i in range(0, length(y)+1):
            if y[i] != 1:
                free_space_ranges_indexs.append(i)
        free_space_ranges_indexs = np.array(free_space_ranges_indexs)
        j = argmax(free_space_ranges_indexs[j+1] - free_space_ranges_indexs[j])
        start_i = free_space_ranges[free_space_ranges_indexs[j]]
        end_i = free_space_ranges[free_space_ranges_indexs[j+1] - 1]

        return start_i, end_i

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        best_point_i = argmax(ranges[start_i:end_i])
        return best_point_i

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        clo_point_index = argmin(proc_ranges)
        clo_point_dis = proc_ranges[clo_point_index]

        #Eliminate all points inside 'bubble' (set them to zero)
        # set bubble radian range = 45du: 90 = 45/2 * 4
        # set radius of bubble = 0.5m
        if clo_point_index >= 90 and clo_point_index + 90 <= 720:
            for i in range(clo_point_index - 90, clo_point_index + 90):
                if abs(proc_ranges[i] - clo_point_dis) <= 0.5:
                    proc_ranges[i] = 0
        elif clo_point_index < 90:
            for i in range(0, clo_point_index + 90):
                if abs(proc_ranges[i] - clo_point_dis) <= 0.5:
                    proc_ranges[i] = 0
        elif clo_point_index + 90 > 720:
            for i in range(clo_point_index - 90, 720):
                if abs(proc_ranges[i] - clo_point_dis) <= 0.5:
                    proc_ranges[i] = 0
        proc_ranges[clo_point_index] = 0

        #Find max length gap
        free_space_ranges = []
        for i in range(0, 721):
            if proc_ranges[i] != 0:
                free_space_ranges.append(i)
        max_gap_start, max_gap_end = self.find_max_gap(free_space_ranges)

        #Find the best point in the gap
        best_point_i = self.find_best_point(max_gap_start, max_gap_end, proc_ranges)

        #steering angle to the best point Positive is to the left

        steering_angle = (best_point_i/4.0 - 90) * math.pi / 180.0

        #Publish Drive message
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = 'laser'
        ack_msg.drive.steering_angle = steering_angle
        ack_msg.drive.speed = 1.0
        self.drive_pub.publish(ack_msg)
def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
