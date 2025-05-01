#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This script moves the ROBOTIS Open Manipulator 6DOF robot along any trajectory.
It first moves to home position before executing the specified trajectory.
"""

import rospy
import sys
import math
import time
import numpy as np
from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState, JointPosition
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

class OpenManipulator6DOFTrajectory:
    def __init__(self):
        rospy.init_node('open_manipulator_trajectory', anonymous=True)
        
        # Set the namespace for the 6DOF robot
        self.ns = '/open_manipulator_6dof'
        
        # Connect to services
        rospy.loginfo("Connecting to the manipulator services...")
        try:
            # Service for absolute positioning in task space
            service_name = self.ns + '/goal_task_space_path'
            rospy.wait_for_service(service_name, timeout=5.0)
            self.task_space_path_client = rospy.ServiceProxy(service_name, SetKinematicsPose)
            rospy.loginfo("Connected to %s" % service_name)
            
            # Service for relative positioning in task space
            service_name = self.ns + '/goal_task_space_path_from_present'
            rospy.wait_for_service(service_name, timeout=5.0)
            self.task_space_path_from_present_client = rospy.ServiceProxy(service_name, SetKinematicsPose)
            rospy.loginfo("Connected to %s" % service_name)
            
            # Service for joint space positioning (for home position)
            service_name = self.ns + '/goal_joint_space_path'
            rospy.wait_for_service(service_name, timeout=5.0)
            self.joint_space_path_client = rospy.ServiceProxy(service_name, SetJointPosition)
            rospy.loginfo("Connected to %s" % service_name)

            service_name = '/compute_ik'
            rospy.wait_for_service(service_name)
            self.ik_client = rospy.ServiceProxy(service_name, GetPositionIK)
            rospy.loginfo("Connected to %s" % service_name)

        except rospy.ROSException as e:
            rospy.logerr("Service not available: %s" % str(e))
            sys.exit(1)
        
        # Subscribe to robot state
        self.robot_state = "STOPPED"
        rospy.Subscriber(self.ns + '/states', OpenManipulatorState, self.robot_state_callback)
        
        # Subscribe to current end-effector pose - FIXED: Added debug output
        self.current_pose = None
        self.pose_updates = 0
        rospy.loginfo("Subscribing to %s" % (self.ns + '/gripper/kinematics_pose'))
        rospy.Subscriber(self.ns + '/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
        
        # Wait for current pose data to be available - FIXED: Better waiting logic and debug
        timeout = 10.0  # Increased timeout
        start_time = rospy.get_time()
        rospy.loginfo("Waiting for pose data...")
        
        while not rospy.is_shutdown():
            if self.current_pose is not None:
                rospy.loginfo("Received initial pose data!")
                break
                
            if rospy.get_time() - start_time > timeout:
                rospy.logwarn("Timed out waiting for current pose. Will try to continue anyway.")
                break
                
            rospy.loginfo("Waiting for pose data... (%.1f seconds remaining)" % 
                         (timeout - (rospy.get_time() - start_time)))
            rospy.sleep(1.0)
        
        # ADDED: Force getting a pose reading if we didn't get one from the subscriber
        if self.current_pose is None:
            try:
                # Try to manually get the current position using a rostopic command
                import subprocess
                cmd = ["rostopic", "echo", "-n", "1", self.ns + "/gripper/kinematics_pose"]
                result = subprocess.check_output(cmd).decode('utf-8')
                rospy.loginfo("Manual pose reading attempted: %s" % result)
                
                # Create a default pose with reasonable values
                from geometry_msgs.msg import Pose
                self.current_pose = KinematicsPose()
                self.current_pose.pose = Pose()
                self.current_pose.pose.position.x = 0.0
                self.current_pose.pose.position.y = 0.0
                self.current_pose.pose.position.z = 0.0
                self.current_pose.pose.orientation.x = 0.0
                self.current_pose.pose.orientation.y = 0.0
                self.current_pose.pose.orientation.z = 0.0
                self.current_pose.pose.orientation.w = 1.0
                
                rospy.logwarn("Using default pose. Movement may not be accurate.")
            except Exception as e:
                rospy.logerr("Failed to get pose manually: %s" % str(e))
                rospy.logerr("Using default values, movements may be erratic")
        
        rospy.loginfo("Robot is ready!")
        
    def robot_state_callback(self, msg):
        # Remove quotes if they exist in the state string
        self.robot_state = msg.open_manipulator_moving_state.strip('"')
        
    def kinematics_pose_callback(self, msg):
        # FIXED: Added debug output to verify we're getting pose updates
        self.current_pose = msg
        self.pose_updates += 1
        
        # Log every 10th update to avoid flooding the console
        if self.pose_updates % 10 == 0:
            rospy.logdebug("Pose update #%d: x=%.3f, y=%.3f, z=%.3f" % 
                         (self.pose_updates,
                          msg.pose.position.x,
                          msg.pose.position.y,
                          msg.pose.position.z))
        
    def wait_for_completion(self):
        """Wait for the current movement to complete."""
        rospy.loginfo("Waiting for movement to complete...")
        rate = rospy.Rate(10)  # 10 Hz
        
        # FIXED: Added timeout to prevent indefinite waiting
        start_time = rospy.get_time()
        timeout = 30.0  # 30 seconds timeout
        
        while not rospy.is_shutdown():
            if self.robot_state != "MOVING":
                rospy.loginfo("Movement completed.")
                return True
                
            if rospy.get_time() - start_time > timeout:
                rospy.logwarn("Movement timed out after %.1f seconds!", timeout)
                return False
                
            rate.sleep()
    
    def move_to_home_position(self, time_seconds=3.0):
        """
        Move the robot to the home position.
        
        Args:
            time_seconds: Time to complete the movement (seconds)
        """
        rospy.loginfo("Moving to home position...")
        
        # Create request for joint space positioning
        request = SetJointPositionRequest()
        request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        # Set home position joint values (adjust these as needed for your robot)
        request.joint_position.position = [0.0, -0.78, 1.5, 0.0, 0.8, 0.0]  # Radians
        request.path_time = time_seconds
        
        # Send the request
        try:
            self.joint_space_path_client(request)
            
            # Wait for movement to complete
            if self.wait_for_completion():
                rospy.loginfo("Robot is now at home position.")
            else:
                rospy.logwarn("Movement to home position may not have completed properly.")
                
        except Exception as e:
            rospy.logerr("Failed to move to home position: %s" % str(e))
    
    def move_to_position(self, x=None, y=None, z=None, 
                        orientation_x=None, orientation_y=None, 
                        orientation_z=None, orientation_w=None, 
                        relative=False, time_seconds=2.0):
        """
        Move the end-effector to a specified position and orientation.
        
        Args:
            x, y, z: Target position (meters), None means maintain current value
            orientation_x, y, z, w: Target orientation as quaternion, None means maintain current
            relative: If True, movement is relative to current position
            time_seconds: Time to complete the movement (seconds)
        """
        # FIXED: Check if we have valid pose data
        if self.current_pose is None:
            rospy.logerr("Cannot move: No current pose available")
            return False
            
        # FIXED: Debug current pose
        rospy.loginfo("Current position before move: x=%.4f, y=%.4f, z=%.4f" % 
                    (self.current_pose.pose.position.x,
                     self.current_pose.pose.position.y,
                     self.current_pose.pose.position.z))
        
        # Get current values to use for any unspecified parameters
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        current_z = self.current_pose.pose.position.z
        current_ox = self.current_pose.pose.orientation.x
        current_oy = self.current_pose.pose.orientation.y
        current_oz = self.current_pose.pose.orientation.z
        current_ow = self.current_pose.pose.orientation.w
        
        # Set target values (use current for any that are None)
        target_x = x if x is not None else current_x
        target_y = y if y is not None else current_y
        target_z = z if z is not None else current_z
        target_ox = orientation_x if orientation_x is not None else current_ox
        target_oy = orientation_y if orientation_y is not None else current_oy
        target_oz = orientation_z if orientation_z is not None else current_oz
        target_ow = orientation_w if orientation_w is not None else current_ow
        
        # Create request
        request = SetKinematicsPoseRequest()
        request.end_effector_name = "gripper"
        
        # Set position and orientation
        request.kinematics_pose.pose.position.x = target_x
        request.kinematics_pose.pose.position.y = target_y
        request.kinematics_pose.pose.position.z = target_z
        request.kinematics_pose.pose.orientation.x = target_ox
        request.kinematics_pose.pose.orientation.y = target_oy
        request.kinematics_pose.pose.orientation.z = target_oz
        request.kinematics_pose.pose.orientation.w = target_ow
        
        request.path_time = time_seconds
        
        # Log movement information
        if relative:
            move_type = "relative"
            x_display = x if x is not None else 0
            y_display = y if y is not None else 0
            z_display = z if z is not None else 0
            rospy.loginfo("Moving %s: dx=%.3f, dy=%.3f, dz=%.3f" % (move_type, x_display, y_display, z_display))
            
            try:
                # Send the relative movement request
                self.task_space_path_from_present_client(request)
            except Exception as e:
                rospy.logerr("Failed to execute relative movement: %s" % str(e))
                return False
        else:
            move_type = "absolute"
            rospy.loginfo("Moving %s: x=%.3f, y=%.3f, z=%.3f" % (move_type, target_x, target_y, target_z))
            
            try:
                # Send the absolute movement request
                self.task_space_path_client(request)
            except Exception as e:
                rospy.logerr("Failed to execute absolute movement: %s" % str(e))
                return False
        
        # Wait for movement to complete
        move_completed = self.wait_for_completion()
        
        # FIXED: Wait a short time to ensure pose is updated
        rospy.sleep(0.2)
        
        # Get and display the new position
        if self.current_pose is not None:
            new_x = self.current_pose.pose.position.x
            new_y = self.current_pose.pose.position.y
            new_z = self.current_pose.pose.position.z
            
            rospy.loginfo("Movement %s. New position: x=%.3f, y=%.3f, z=%.3f" % 
                        ("complete" if move_completed else "may not be complete",
                         new_x, new_y, new_z))
            return move_completed
        else:
            rospy.logerr("No pose data available after movement")
            return False
    
    def execute_linear_trajectory(self, waypoints, time_per_segment=2.0):
        """
        Execute a linear trajectory through a series of waypoints.
        
        Args:
            waypoints: List of dictionaries with x, y, z, and optional orientation keys
                       None values mean no change from current value
            time_per_segment: Time to move between each waypoint (seconds)
        """
        rospy.loginfo("Executing trajectory with %d waypoints" % len(waypoints))
        
        for i, point in enumerate(waypoints):
            rospy.loginfo("Moving to waypoint %d/%d" % (i+1, len(waypoints)))
            success = self.move_to_position(
                x=point.get('x'), 
                y=point.get('y'), 
                z=point.get('z'),
                orientation_x=point.get('orientation_x'),
                orientation_y=point.get('orientation_y'),
                orientation_z=point.get('orientation_z'),
                orientation_w=point.get('orientation_w'),
                relative=point.get('relative', False),
                time_seconds=time_per_segment
            )
            
            if not success:
                rospy.logwarn("Failed to reach waypoint %d, aborting trajectory" % (i+1))
                return False
                
            # FIXED: Add a small delay between waypoints
            rospy.sleep(0.5)
            
        return True
    
    def execute_circular_trajectory(self, center_x, center_y, center_z, radius, num_points=8, time_per_segment=1.0):
        """
        Execute a circular trajectory in the XY plane.
        
        Args:
            center_x, center_y, center_z: Center of the circle
            radius: Radius of the circle (meters)
            num_points: Number of points to use for the circle
            time_per_segment: Time to move between each point (seconds)
        """
        rospy.loginfo("Executing circular trajectory: center=(%.3f, %.3f, %.3f), radius=%.3f" % (center_x, center_y, center_z, radius))
        
        # Generate waypoints for the circle
        waypoints = []
        for i in range(num_points + 1):  # +1 to close the circle
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            waypoints.append({'x': x, 'y': y, 'z': center_z})
        
        # Execute the circular trajectory
        return self.execute_linear_trajectory(waypoints, time_per_segment)
    
    def execute_DMP_Trajectory(self,waypoints, time_per_segment):

        rospy.loginfo("Executing DMP Trajectory")

        # Prepare the IK request
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = "arm"  
        ik_request.ik_request.ik_link_name = "end_effector_link"  # spaeter "end_effector_link"
        ik_request.ik_request.timeout = rospy.Duration(0.1)

        # Prepare set position request
        position_request = SetJointPositionRequest()
        position_request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        success = True

        for i, point in enumerate(waypoints):
            rospy.loginfo("Moving to waypoint %d/%d" % (i+1, len(waypoints)))

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "world"
            pose_stamped.pose.position.x=point.get('x')
            pose_stamped.pose.position.y=point.get('y') 
            pose_stamped.pose.position.z=point.get('z')
            pose_stamped.pose.orientation.x=point.get('orientation_x')
            pose_stamped.pose.orientation.y=point.get('orientation_y')
            pose_stamped.pose.orientation.z=point.get('orientation_z')
            pose_stamped.pose.orientation.w=point.get('orientation_w')
        
            ik_request.ik_request.pose_stamped = pose_stamped
            try:
                response = self.ik_client(ik_request)
                if response.error_code.val == response.error_code.SUCCESS:
                    rospy.loginfo("IK solution found for waypoint %d:" % (i+1))
                    joint_values = response.solution.joint_state.position[:6]
                    position_request.joint_position.position = joint_values
                    position_request.path_time = time_per_segment
                    try:
                        self.joint_space_path_client(position_request)
                        if not self.wait_for_completion():
                            rospy.logwarn("Movement did not complete for waypoint %d" % (i+1))
                    except Exception as e:
                        rospy.logerr("Failed to move to waypoint position %d: %s" % ((i+1), str(e)))
                else:
                    rospy.logwarn("IK failed for waypoint %d with error code: %d" % ((i+1), response.error_code.val))
                    success = False
            except rospy.ServiceException as e:
                    rospy.logwarn("Service call failed for waypoint %d: %s" % ((i+1), str(e)))

            return success

    def run_demo(self):
        from tf.transformations import quaternion_from_euler
        roll = 0.0
        pitch = 1.5708
        yaw = 0.0
        q = quaternion_from_euler(roll, pitch, yaw)
        """Run a demonstration of various trajectories"""
        # First move to home position
        self.move_to_home_position()
        rospy.sleep(5.0)  # Pause
        
        # FIXED: Check current pose before continuing
        if self.current_pose is None:
            rospy.logerr("No pose data available. Cannot continue demo.")
            return
            
        # Example 1: Simple linear movement up and down (Z-axis)
        rospy.loginfo("Demo 1: Simple  movement")
        waypoints = [
            {'x': 0.12,'y':0.0,'z':0.26,'orientation_x':q[0],'orientation_y':q[1],'orientation_z':q[2],'orientation_w':q[3]},  # Move up 5cm
                                
        ]
        self.execute_DMP_Trajectory(waypoints,2.0)
        rospy.sleep(1.0)  # Pause
        
        # Return to home position
        rospy.loginfo("Returning to home position")
        self.move_to_home_position()

if __name__ == '__main__':
    try:
        robot = OpenManipulator6DOFTrajectory()
        robot.run_demo()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error in trajectory execution: %s" % str(e))
        sys.exit(1)