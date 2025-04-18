#!/usr/bin/env python3
import sys
import os
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

# Import Z1 SDK library using ROS package-relative path
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('umi_robots')
sdk_path = os.path.join(package_path, "libs/z1_sdk")
sys.path.append(sdk_path)
import z1_arm_interface

class Z1ArmController:
    """
    ROS node for controlling the Unitree Z1 robot arm.
    Provides two control interfaces:
    1. Joint control through /joint_commands
    2. End effector pose control through /endpose_command (using IK)
    """
    def __init__(self):
        rospy.init_node('z1_arm_controller')
        
        # Initialize Z1 arm interface without gripper
        self.arm = z1_arm_interface.ArmInterface(hasGripper=False)
        self.arm_model = self.arm._ctrlComp.armModel
        
        # Get robot parameters
        self.joint_max = self.arm_model.getJointQMax()
        self.joint_min = self.arm_model.getJointQMin()
        self.joint_speed_max = self.arm_model.getJointSpeedMax()
        
        # Arm state
        self.current_joint_state = np.zeros(6)
        
        # Default speeds
        self.max_joint_speed = 0.5  # rad/s
        self.max_cart_speed = 0.1   # m/s
        
        # Start the control loop
        self.arm.loopOn()
        
        # Get into joint control mode
        rospy.loginfo("Setting arm to JOINTCTRL mode...")
        if self.arm.getCurrentState() != z1_arm_interface.ArmFSMState.JOINTCTRL:
            self.arm.setFsm(z1_arm_interface.ArmFSMState.PASSIVE)
            rospy.sleep(0.5)
            self.arm.startTrack(z1_arm_interface.ArmFSMState.JOINTCTRL)
            rospy.sleep(0.5)
        
        # Update current joint state
        self.current_joint_state = self.arm.q
        
        # Create ROS subscribers
        self.joint_cmd_sub = rospy.Subscriber('/joint_commands', Float64MultiArray, self.joint_cmd_callback, queue_size=1)
        self.endpose_cmd_sub = rospy.Subscriber('/endpose_command', Pose, self.endpose_cmd_callback, queue_size=1)
        
        rospy.loginfo("Z1 Arm Controller initialized and ready")
    
    def joint_cmd_callback(self, msg):
        """
        Callback for joint commands.
        Expects a Float64MultiArray with 6 values for the 6 arm joints
        """
        if len(msg.data) < 6:
            rospy.logwarn("Joint command needs at least 6 values")
            return
        
        joint_cmd = np.array(msg.data[:6])
        
        # Check joint limits
        for i in range(6):
            joint_cmd[i] = np.clip(joint_cmd[i], self.joint_min[i], self.joint_max[i])
        
        # Send joint command to the arm
        rospy.loginfo(f"Moving to joint positions: {joint_cmd}")
        self.arm.MoveJ(joint_cmd, 0.0, self.max_joint_speed)  # Passing 0.0 as gripper position
        
        # Update internal state
        self.current_joint_state = joint_cmd
    
    def endpose_cmd_callback(self, msg):
        """
        Callback for end effector pose commands.
        Uses inverse kinematics to calculate joint commands.
        """
        # Convert ROS Pose message to Z1 pose format [roll, pitch, yaw, x, y, z]
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        
        # Convert quaternion to Euler angles using SciPy's Rotation class
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        
        # Create a rotation object from quaternion
        rot = Rotation.from_quat([qx, qy, qz, qw])
        
        # Get Euler angles in 'xyz' convention (roll, pitch, yaw)
        roll, pitch, yaw = rot.as_euler('xyz')
        
        # Create target pose
        target_pose = np.array([roll, pitch, yaw, x, y, z])
        
        # Convert pose to homogeneous transformation matrix
        T_target = z1_arm_interface.postureToHomo(target_pose)
        
        # Compute inverse kinematics
        success, joint_cmd = self.arm_model.inverseKinematics(T_target, self.current_joint_state, True)
        
        if not success:
            rospy.logwarn("IK failed for the requested pose")
            return
        
        # Send command to arm
        rospy.loginfo(f"Moving to pose: [r={roll:.3f}, p={pitch:.3f}, y={yaw:.3f}], pos=[{x:.3f}, {y:.3f}, {z:.3f}]")
        rospy.loginfo(f"Calculated joint positions: {joint_cmd}")
        
        self.arm.MoveJ(joint_cmd, 0.0, self.max_joint_speed)  # Passing 0.0 as gripper position
        
        # Update internal state
        self.current_joint_state = joint_cmd
    
    def shutdown(self):
        """Clean shutdown of the arm controller"""
        rospy.loginfo("Shutting down Z1 Arm Controller")
        self.arm.loopOff()
        # Set the arm to PASSIVE mode for safety
        self.arm.setFsm(z1_arm_interface.ArmFSMState.PASSIVE)

def main():
    controller = Z1ArmController()
    
    # Register shutdown hook
    rospy.on_shutdown(controller.shutdown)
    
    # Spin to keep the script alive
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
