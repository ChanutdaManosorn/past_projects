#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SmartWallFollower:
    def __init__(self):
        rospy.init_node('smart_wall_explorer')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.state = 'forward'
        self.rate = rospy.Rate(10)

        # movement config
        self.forward_speed = 0.15
        self.turn_speed = 0.6
        self.min_front_dist = 0.5
        self.min_side_dist = 0.3
        self.wall_following = True

        # sensor sectors
        self.front = float('inf')
        self.left = float('inf')
        self.right = float('inf')

    def scan_callback(self, scan):
        ranges = scan.ranges
        n = len(ranges)

        # Slice sectors
        self.front = min(min(ranges[n//3:2*n//3]), 3.0)
        self.left = min(min(ranges[3*n//4:]), 3.0)
        self.right = min(min(ranges[:n//4]), 3.0)

        # Decision making
        if self.front < self.min_front_dist:
            self.state = 'avoid'
        elif self.wall_following:
            if self.right > 0.6:
                self.state = 'turn_right'
            elif self.right < self.min_side_dist:
                self.state = 'turn_left'
            else:
                self.state = 'forward'

    def move(self, linear=0.0, angular=0.0, duration=1.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            self.rate.sleep()
        self.cmd_pub.publish(Twist())  # stop

    def run(self):
        rospy.loginfo("Smart wall-following SLAM explorer started.")
        while not rospy.is_shutdown():
            if self.state == 'forward':
                self.move(linear=self.forward_speed, duration=0.5)
            elif self.state == 'turn_left':
                self.move(angular=self.turn_speed, duration=0.3)
            elif self.state == 'turn_right':
                self.move(angular=-self.turn_speed, duration=0.3)
            elif self.state == 'avoid':
                rospy.loginfo("Obstacle ahead! Backing up and turning.")
                self.move(linear=-0.1, duration=2.0)
                self.move(angular=random.choice([-1, 1]) * self.turn_speed, duration=1.0)
                self.state = 'forward'
            self.rate.sleep()

if __name__ == '__main__':
    try:
        SmartWallFollower().run()
    except rospy.ROSInterruptException:
        pass
