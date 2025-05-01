#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from tf.transformations import quaternion_from_euler

def get_parameters():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('compute_ik_node', anonymous=True)

    # Initialize MoveIt! Commander with the robot's MoveIt! configuration
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Specify the planning group (replace with the correct group name)
    group_name = "arm"  # Ensure this is the correct planning group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    eef_link = move_group.get_end_effector_link()
    print("============ End effector: %s" % eef_link)
    planning_frame = move_group.get_planning_frame()
    print("============ Reference frame: %s" % planning_frame)
    # Define the target pose you want to calculate IK for
    # target_pose = Pose()
    # target_pose.position.x = 0.1
    # target_pose.position.y = 0.25
    # target_pose.position.z = 0.2
    # target_pose.orientation.w = 1.0  # No rotation here (identity quaternion)

    # # Set the pose target
    # move_group.set_pose_target(target_pose)

    # # Get the inverse kinematics solution
    # ik_solution = move_group.get_inv_kin(target_pose)

    # # Check if an IK solution was found
    # if ik_solution:
    #     rospy.loginfo("Found IK solution!")
    #     rospy.loginfo("Joint angles: %s", ik_solution)
    # else:
    #     rospy.logwarn("No IK solution found for the given pose.")

    moveit_commander.roscpp_shutdown()


def compute_ik_via_service():
    rospy.init_node('ik_service_client')

    rospy.wait_for_service('/compute_ik')
    ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)

    # Prepare the IK request
    ik_request = GetPositionIKRequest()
    ik_request.ik_request.group_name = "arm"  # your planning group
    ik_request.ik_request.ik_link_name = "link7"  # spaeter "end_effector_link"
    ik_request.ik_request.timeout = rospy.Duration(0.1)

    # Fill in desired pose
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "world"  # your robot's base link
    pose_stamped.pose.position.x = 0.12
    pose_stamped.pose.position.y = 0.0
    pose_stamped.pose.position.z = 0.26

    roll = 0.0
    pitch = 1.5708
    yaw = 0.0
    q = quaternion_from_euler(roll, pitch, yaw)

    pose_stamped.pose.orientation.x = q[0]
    pose_stamped.pose.orientation.y = q[1]
    pose_stamped.pose.orientation.z = q[2]
    pose_stamped.pose.orientation.w = q[3]

    

    ik_request.ik_request.pose_stamped = pose_stamped

    try:
        response = ik_service(ik_request)
        if response.error_code.val == response.error_code.SUCCESS:
            print("IK solution found:")
            joint_names = response.solution.joint_state.name
            joint_values = response.solution.joint_state.position
            for name, value in zip(joint_names, joint_values):
                print("%s: %.4f" % (name, value))
        else:
            print("IK failed with error code:", response.error_code.val)
    except rospy.ServiceException as e:
        print("Service call failed:", e)


if __name__ == '__main__':
    compute_ik_via_service()


