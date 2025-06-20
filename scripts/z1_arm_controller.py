#!/usr/bin/env python3
import sys
import os
import numpy as np
import rospy
import threading
import time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation

# Import Z1 SDK library using ROS package-relative path
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('umi_robots')
sdk_path = os.path.join(package_path, "libs/z1_sdk/lib")
sys.path.append(sdk_path)
print(f"Adding Z1 SDK path: {sdk_path}")
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
        self.switch_to_jointctrl()
        
        # Move to forward position using labelRun
        rospy.loginfo("Moving to forward position...")
        self.arm.labelRun("forward")
        # self.arm.backToStart()


        rospy.sleep(2.0)  # Give time for the movement to complete
        # self.swithch_to_lowcmd()

        # Update current joint state
        self.current_joint_state = self.arm.q
        
        # Create ROS subscribers
        self.joint_cmd_sub = rospy.Subscriber('/joint_commands', Float64MultiArray, self.joint_cmd_callback, queue_size=1)
        self.endpose_cmd_sub = rospy.Subscriber('/endpose_command', Pose, self.endpose_cmd_callback, queue_size=1)
        
        # Create joint state publisher
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # Create publisher for internal joint commands
        self.joint_commands_internal_pub = rospy.Publisher('/joint_commands_internal', JointState, queue_size=10)
        
        # Joint names for the Z1 robot arm
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Create a timer to publish joint states at 50 Hz
        self.joint_state_timer = rospy.Timer(rospy.Duration(1.0/50.0), self.publish_joint_states)
        
        rospy.loginfo("Z1 Arm Controller initialized and ready")

    def swithch_to_lowcmd(self):
        """
        Switch to LOWCMD mode
        """
        if self.arm.getCurrentState() != z1_arm_interface.ArmFSMState.LOWCMD:
            rospy.loginfo("Switching to LOWCMD mode")
            self.arm.setFsm(z1_arm_interface.ArmFSMState.PASSIVE)
            rospy.sleep(0.2)
            self.arm.setFsmLowcmd()
            rospy.sleep(0.5)
            rospy.loginfo("Switched to LOWCMD mode")

    def switch_to_jointctrl(self):
        """
        Switch to JOINTCTRL mode
        """
        if self.arm.getCurrentState() != z1_arm_interface.ArmFSMState.JOINTCTRL:
            rospy.loginfo("Switching to JOINTCTRL mode")
            self.arm.setFsm(z1_arm_interface.ArmFSMState.PASSIVE)
            rospy.sleep(0.2)
            self.arm.startTrack(z1_arm_interface.ArmFSMState.JOINTCTRL)
            rospy.sleep(0.5)
            rospy.loginfo("Switched to JOINTCTRL mode")

    def joint_cmd_callback(self, msg):
        """
        Callback for joint commands.
        Expects a Float64MultiArray with 6 values for the 6 arm joints
        Uses direct joint position control in JOINTCTRL mode
        """
        if len(msg.data) < 6:
            rospy.logwarn("Joint command needs at least 6 values")
            return
        
            
        target_position = np.array(msg.data[:6])
        
        # Check joint limits
        for i in range(6):
            target_position[i] = np.clip(target_position[i], self.joint_min[i], self.joint_max[i])
        
        # Log the movement
        rospy.loginfo(f"Moving to joint positions: {target_position}")
        
        # Set joint command directly
        self._set_joint_positions(target_position)
        
        # Update target state
        self.target_joint_state = target_position
    
    def _execute_joint_trajectory(self, start_pos, end_pos, duration, dt):
        """
        Execute a smooth trajectory from start to end position
        Similar to the approach in example_lowcmd.py
        """
        try:
            arm_model = self.arm_model
            
            for i in range(duration+1):
                # Interpolate position (same as example_lowcmd.py)
                position = start_pos * (1 - i/duration) + end_pos * (i/duration)
                
                # Calculate velocity
                velocity = (end_pos - start_pos) / (duration * dt)
                
                # Calculate torque using inverse dynamics
                torque = arm_model.inverseDynamics(position, velocity, np.zeros(6), np.zeros(6))
                
                # Apply command to the arm
                self.arm.setArmCmd(position, velocity, torque)
                
                # Send the command via UDP (critical step from example_lowcmd.py)
                self.arm.sendRecv()
                
                # Publish joint commands to internal topic
                self.publish_joint_commands(position, velocity, torque)
                
                # Update internal state
                self.current_joint_state = position
                
                # Sleep to maintain control rate
                time.sleep(dt)
            
            rospy.loginfo("Joint trajectory completed")
        
                
        except Exception as e:
            rospy.logerr(f"Error during trajectory execution: {e}")

    def _set_joint_positions(self, joint_positions):
        """
        Set joint positions directly using JOINTCTRL mode.
        This method implements joint control similar to the C++ example in Z1HW::write.
        
        Args:
            joint_positions: Array of 6 joint position commands in radians
        """
        try:
            # Make sure we're in JOINTCTRL mode
            if self.arm.getCurrentState() != z1_arm_interface.ArmFSMState.JOINTCTRL:
                rospy.loginfo("Switching to JOINTCTRL mode")
                self.switch_to_jointctrl()
            
            # In the Python SDK, for JOINTCTRL mode we use setTargetQ method
            # or setArmCmd method depending on the version
            try:
                # Newer Z1 SDK versions have setTargetQ
                self.arm.setTargetQ(joint_positions)
            except AttributeError:
                # Fall back to using setArmCmd for older versions
                # Only set position, with zero velocity and torque
                zero_array = np.zeros(6)
                self.arm.setArmCmd(joint_positions, zero_array, zero_array)
            
            # Send the command
            self.arm.sendRecv()
            
            # Create and publish a joint state message for the commands
            cmd_msg = JointState()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.header.frame_id = "joint_commands"
            cmd_msg.name = self.joint_names
            cmd_msg.position = joint_positions
            cmd_msg.velocity = [0.0] * 6  # Zero velocity for position control
            cmd_msg.effort = [0.0] * 6    # No specific effort
            
            # Publish the command
            self.joint_commands_internal_pub.publish(cmd_msg)
            
            # Update internal state
            self.current_joint_state = joint_positions
            
        except Exception as e:
            rospy.logerr(f"Error setting joint positions: {e}")

    
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
        # target_pose = np.asarray([0.5,0.1,0.1,0.5,-0.2,0.5])
        
        tic = time.time()
        self.arm.setWait(false)
        self.arm.MoveJ(target_pose, self.max_joint_speed)
        print("Time taken for MoveJ: ", time.time() - tic)
    
    def publish_joint_states(self, event):
        """
        Timer callback to publish joint states at 50 Hz.
        Gets the current joint positions from the arm interface and publishes them
        to the /joint_states topic in the standard JointState format.
        """
        try:
            # Get current joint positions from arm
            current_positions = self.arm.q
            
            # Create JointState message
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = current_positions
            
            # Get current joint velocities if available
            if hasattr(self.arm, 'dq'):
                joint_state_msg.velocity = self.arm.dq
                
            # Publish the joint state message
            self.joint_state_pub.publish(joint_state_msg)
            
            # Update the internal state variable (useful for other methods)
            self.current_joint_state = current_positions
            
        except Exception as e:
            rospy.logerr(f"Error publishing joint states: {e}")
    
    def publish_joint_commands(self, position, velocity, torque):
        """
        Publish joint commands to the internal topic.
        
        Args:
            position: Joint positions array
            velocity: Joint velocities array
            torque: Joint torques array
        """
        cmd_msg = JointState()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.name = self.joint_names
        cmd_msg.position = position
        cmd_msg.velocity = velocity
        cmd_msg.effort = torque
        self.joint_commands_internal_pub.publish(cmd_msg)
    
    def shutdown(self):
        """Clean shutdown of the arm controller"""
        rospy.loginfo("Shutting down Z1 Arm Controller")
        self.switch_to_jointctrl()

        
        # Proper shutdown sequence based on example code
        try:
            # First make sure we're in a proper control state
            current_state = self.arm.getCurrentState()
            rospy.loginfo(f"Current arm state: {current_state}")
            
            # Use the backToStart method from the example code which handles state transitions properly
            rospy.loginfo("Moving arm to safe position")
            self.arm.backToStart()
            
            # Wait for the motion to complete
            rospy.sleep(2.0)
            
            rospy.loginfo("Arm moved to safe position")
        except Exception as e:
            rospy.logwarn(f"Error during arm positioning: {e}")
        
        # Finally turn off the control loop (this should handle state transitions properly)
        rospy.loginfo("Turning off control loop")
        
        # Setting to PASSIVE mode after loop is off
        rospy.loginfo("Setting to passive mode")
        try:
            self.arm.setFsm(z1_arm_interface.ArmFSMState.PASSIVE)
        except Exception as e:
            rospy.logwarn(f"Error setting passive mode: {e}")
        rospy.loginfo("Setting to passive mode complete")
        self.arm.loopOff()

        rospy.loginfo("Arm shutdown complete")



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
