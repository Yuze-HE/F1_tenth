#!/usr/bin/env python
import rospy
import math
# TODO: import ROS msg types and libraries
import numpy as ny
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # TODO: create ROS subscribers and publishers.
        pub_brake = rospy.Publisher('/brake', AckermannDriveStamped, queue_size=10)
        pub_brake_bool = rospy.Publisher('/brake_bool', Bool, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, scan_callback)
        rospy.Subscriber("/odom", Odometry, odom_callback)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ranges = np.asarray(scan_msg.ranges)
        TTC = 10000
        brake_sign = False
        for angle in (0, 180):  # 因为(-45,0) 和（180，225）都使得TTC为无穷大
            r = ranges[(angle+45)*4]
            r_dot = self.speed * cos((90 - angle) * pi / 180)
            if r_dot > 0:
                TTC_pre = r / r_dot
                if TTC_pre < 0.5:
                    brake_sign = True
                    break

        # TODO: publish brake message and publish controller bool
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        if brake_sign:
            drive_msg.drive.speed = 0.0
        pub_brake.publish(drive_msg)
        pub_brake_bool.publish(brake_sign)


def main():
    rospy.init_node('safety_node', anonymous=True)
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()