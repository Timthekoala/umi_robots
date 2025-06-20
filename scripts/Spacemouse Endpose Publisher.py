#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Quaternion
from geometry_msgs.msg import Twist
import time
import sys
import os
from scipy.spatial.transform import Rotation
import numpy as np







class TwistToPose:
    def __init__(self):
        rospy.init_node('twist_to_pose_converter')

        self.pose_pub = rospy.Publisher('/endpose_command', Pose, queue_size=10)
        self.twist_sub = rospy.Subscriber('/spacenav/twist', Twist, self.twist_callback)

        self.current_pose = Pose()

        rospy.loginfo("Twist to Pose converter initialized.")
    
    def twist_callback(self, twist_msg):

        # Convert position
        x_pos = twist_msg.linear.x
        y_pos = twist_msg.linear.y
        z_pos = twist_msg.linear.z
        x_ang = twist_msg.angular.x
        y_ang = twist_msg.angular.y
        z_ang = twist_msg.angular.z

        self.pose_msg = Pose()
        self.pose_msg.position.x = x_pos
        self.pose_msg.position.y = y_pos
        self.pose_msg.position.z = z_pos
        self.pose_msg.orientation.x = x_ang
        self.pose_msg.orientation.y = y_ang
        self.pose_msg.orientation.z = z_ang
        self.pose_msg.orientation.w = 1.0

        # Publish the new pose
        self.pose_pub.publish(self.pose_msg)


if __name__ == '__main__':
    try:
        node = TwistToPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

