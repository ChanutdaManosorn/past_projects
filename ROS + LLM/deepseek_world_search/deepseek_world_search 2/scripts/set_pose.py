#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import math

if __name__ == '__main__':
    rospy.init_node('set_initial_pose')
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(1.0)  # wait for AMCL

    x = rospy.get_param('~initial_pose_x', 0.0)
    y = rospy.get_param('~initial_pose_y', 0.0)
    yaw = rospy.get_param('~initial_pose_yaw', 0.0)

    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.pose.position.x = x
    pose.pose.pose.position.y = y
    pose.pose.pose.orientation.x = quat[0]
    pose.pose.pose.orientation.y = quat[1]
    pose.pose.pose.orientation.z = quat[2]
    pose.pose.pose.orientation.w = quat[3]
    pose.pose.covariance[0] = 0.25
    pose.pose.covariance[7] = 0.25
    pose.pose.covariance[35] = math.radians(10)

    pub.publish(pose)
    rospy.loginfo("Initial pose published.")
