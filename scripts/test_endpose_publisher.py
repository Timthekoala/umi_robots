#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Quaternion
import time
from scipy.spatial.transform import Rotation
import numpy as np

def test_endpose_publisher():
    """
    Test script that publishes end pose commands to the Z1 arm controller.
    
    1. Publishes initial position for 2 seconds
    2. Gradually moves Y position from -0.5 to 0.5 at 0.01 units/sec
    3. Publishing rate: 50 Hz
    
    This tests the endpose_cmd_callback functionality of the Z1 arm controller.
    """
    # Initialize ROS node
    rospy.init_node('test_endpose_publisher', anonymous=True)
    
    # Create publisher for end pose commands
    pose_pub = rospy.Publisher('/endpose_command', Pose, queue_size=10)
    
    # Give time for the publisher to connect
    rospy.sleep(1.0)
    
    # Fixed position values (x, z) in meters
    x, z = 0.5, 0.5
    
    # Initial y position
    
    # Orientation values (roll, pitch, yaw) in radians
    roll, pitch, yaw = 0, 0, 0
    
    # Calculate movement parameters
    y_start = -0.2
    y_end = 0.2
    y_speed = 0.1  # units per second
    total_distance = y_end - y_start
    total_time_needed = total_distance / y_speed  # seconds
    
    # Publishing rate
    rate = rospy.Rate(50)  # 50 Hz
    
    # Convert Euler angles to quaternion
    rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
    quat = rot.as_quat()  # Returns [x, y, z, w]
    
    # Create the pose message
    pose_msg = Pose()
    
    # Set orientation as quaternion (won't change)
    pose_msg.orientation.x = quat[0]
    pose_msg.orientation.y = quat[1]
    pose_msg.orientation.z = quat[2]
    pose_msg.orientation.w = quat[3]
    
    # Log the start of the test
    rospy.loginfo("Starting end pose test:")
    rospy.loginfo(f"Initial position: ({x}, {y_start}, {z})")
    rospy.loginfo(f"Will move Y coordinate from {y_start} to {y_end} at {y_speed} units/sec")
    rospy.loginfo(f"Total movement time: {total_time_needed:.1f} seconds")
    
    # First phase: hold initial position for 2 seconds
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < 2.0:
        # Set initial position
        pose_msg.position.x = x
        pose_msg.position.y = y_start
        pose_msg.position.z = z
        
        # Publish the message
        pose_pub.publish(pose_msg)
        rate.sleep()
    
    # Second phase: gradually move Y from -0.5 to 0.5
    start_time = rospy.Time.now()
    current_time = 0.0
    
    while current_time < total_time_needed:
        # Update current time
        current_time = (rospy.Time.now() - start_time).to_sec()
        
        # Calculate current y position based on time and speed
        current_y = y_start + (y_speed * current_time)
        current_y = min(current_y, y_end)  # Make sure we don't exceed the end value
        
        # Set position
        pose_msg.position.x = x
        pose_msg.position.y = current_y
        pose_msg.position.z = z
        
        # Log progress (but not too often)
        # if int(current_time * 10) % 50 == 0:  # Log every 5 seconds
        #     rospy.loginfo(f"Current Y position: {current_y:.3f}, Time: {current_time:.1f}s")
        
        # Publish the message
        pose_pub.publish(pose_msg)
        rate.sleep()
    
    # Hold final position for 2 seconds
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < 2.0:
        # Set final position
        pose_msg.position.x = x
        pose_msg.position.y = y_end
        pose_msg.position.z = z
        
        # Publish the message
        pose_pub.publish(pose_msg)
        rate.sleep()
    
    rospy.loginfo("End pose movement completed!")
    rospy.loginfo(f"Final position: ({x}, {y_end}, {z})")

if __name__ == "__main__":
    try:
        test_endpose_publisher()
    except rospy.ROSInterruptException:
        pass
