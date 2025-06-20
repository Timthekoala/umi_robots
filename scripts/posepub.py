#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
import time
import sys
import os
import numpy as np
from avp_stream import VisionProStreamer

class AvpPub:
    def __init__(self, Vision):
        rospy.init_node('PosePub')
        self.pose_pub = rospy.Publisher('/endpose_command', Pose, queue_size=10)
        self.current_pose = Pose()
        self.rate = rospy.Rate(10)
        self.vision = Vision
        rospy.loginfo('pub is initialised')
        r = Vision.latest
        self.initx = (r['right_wrist'][0][1][3])
        self.inity = (r['right_wrist'][0][0][3])
        self.initz = (r['right_wrist'][0][2][3])
        self.instartx = 0.1
        self.inendx = 0.5
        self.instarty = -0.2
        self.inendy = 0.4
        self.instartz = 0.7
        self.inendz = 1.35
    def publish_pose(self):
        while not rospy.is_shutdown():
            flag = False
            try:
                r = self.vision.latest
    
                self.current_pose.position.x = ((r['right_wrist'][0][1][3]) - self.instartx)/(self.inendx-self.instartx)*2 -1
                self.current_pose.position.y = ((r['right_wrist'][0][0][3]) - self.instarty)/(self.inendy-self.instarty)*2 -1
                self.current_pose.position.z = ((r['right_wrist'][0][2][3]) - self.instartz)/(self.inendz-self.instartz)*2 -1
                self.current_pose.orientation.x = 0
                self.current_pose.orientation.y = 0
                self.current_pose.orientation.z = 0
                self.current_pose.orientation.w = 1
                rospy.loginfo("Currx: %s", (r['right_wrist'][0][1][3]))
                rospy.loginfo("Curry: %s", (r['right_wrist'][0][0][3]))
                rospy.loginfo("Currz: %s", (r['right_wrist'][0][2][3]))
                flag = True
            except (AttributeError, IndexError, TypeError) as e:
                rospy.logerr(f"Error parsing VisionProStreamer data: {e}")

            if flag:
                self.pose_pub.publish(self.current_pose)
                rospy.loginfo("Published pose: %s", self.current_pose)
            else:
                rospy.logwarn("Failed hehe u suck")

if __name__ == '__main__':
    try:
        avp_ip = "192.168.10.115"
        Vision = VisionProStreamer(ip=avp_ip, record=True)
        
        node = AvpPub(Vision)
        node.publish_pose()
        
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Shutting down node")
