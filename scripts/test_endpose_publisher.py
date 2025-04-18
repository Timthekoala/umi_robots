#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Quaternion
import time
from scipy.spatial.transform import Rotation
import numpy as np

def test_endpose_publisher():
    """
    Simple test script that publishes a fixed end pose command to the Z1 arm controller.
    
    Position: (0.1, 0.2, 0.3) meters
    Orientation: (0, 0, 0) Euler angles (roll, pitch, yaw) in XYZ convention
    
    This tests the endpose_cmd_callback functionality of the Z1 arm controller.
    """
    # Initialize ROS node
    rospy.init_node('test_endpose_publisher', anonymous=True)
    
    # Create publisher for end pose commands
    pose_pub = rospy.Publisher('/endpose_command', Pose, queue_size=10)
    
    # Give time for the publisher to connect
    rospy.sleep(1.0)
    
    # Position values (x, y, z) in meters
    x, y, z = 0.5,-0.2,0.5
    
    # Orientation values (roll, pitch, yaw) in radians
    roll, pitch, yaw = 0, 0, 0
    
    # Convert Euler angles to quaternion
    rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
    quat = rot.as_quat()  # Returns [x, y, z, w]
    
    # Create the pose message
    pose_msg = Pose()
    
    # Set position
    pose_msg.position.x = x
    pose_msg.position.y = y
    pose_msg.position.z = z
    
    # Set orientation as quaternion
    pose_msg.orientation.x = quat[0]
    pose_msg.orientation.y = quat[1]
    pose_msg.orientation.z = quat[2]
    pose_msg.orientation.w = quat[3]
    
    # Log the message
    rospy.loginfo("Publishing end pose command:")
    rospy.loginfo(f"Position: ({x}, {y}, {z})")
    rospy.loginfo(f"Orientation (roll, pitch, yaw): ({roll}, {pitch}, {yaw})")
    
    # Publish the message a few times to make sure it's received
    rate = rospy.Rate(1)  # 1 Hz
    for _ in range(5):  # Publish for 5 seconds
        pose_pub.publish(pose_msg)
        rate.sleep()
    
    rospy.loginfo("End pose command sending completed!")

if __name__ == "__main__":
    try:
        test_endpose_publisher()
    except rospy.ROSInterruptException:
        pass
