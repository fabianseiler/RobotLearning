"""
====================
Visualiser for recorded Trajectories
====================

A recorded trajectory preprocessed and visualised with open3d.  
"""
print(__doc__)

import numpy as np
import pytransform3d.visualizer as pv
import pytransform3d.trajectories as ptr
from movement_primitives.kinematics import Kinematics
import rosbag
from tf.transformations import quaternion_matrix


recording_path = '/root/catkin_ws/src/my_scripts_rl/recordings/rec/grip1.bag'

def animation_callback(step, graph, chain, joint_trajectory):
    chain.forward(joint_trajectory[step])
    graph.set_data()
    return graph


def remove_outliers_mad(data, threshold=3.5):
    """
    Removes outliers from a Nx3 array using Median Absolute Deviation (MAD).
    Returns a mask for inlier indices and the filtered data.
    """
    median = np.median(data, axis=0)
    diff = np.abs(data - median)
    mad = np.median(diff, axis=0)
    modified_z_score = 0.6745 * diff / (mad + 1e-6)
    mask = np.all(modified_z_score < threshold, axis=1)
    return mask, data[mask]


use_carthesian_space = False

with open('/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf', 'r') as f:
    kin = Kinematics(f.read(), mesh_path='/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/meshes')

chain = kin.create_chain(
    ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    "world", "end_effector_link")

bag = rosbag.Bag(recording_path)

if use_carthesian_space:
    print("Using the Cartesian space trajectory")

    q0 = np.array([0.0, -0.78, 1.5, 0., 0.8, 0.])
    chain.forward(q0)

    transforms = []
    traj_pos_quat = []
    time_stamp = []

    for topic, msg, t in bag.read_messages(topics=['/current_pose']):
        pos = msg.pose.position
        ori = msg.pose.orientation
        time = msg.header.stamp.to_sec()
        time_stamp.append(time)
        transform = np.eye(4)
        transform[:3, :3] = quaternion_matrix([ori.w, ori.x, ori.y, ori.z])[0:3, 0:3]

        transform[:3, 3] = [pos.z, pos.y, pos.x]
        transforms.append(transform)
        traj_pos_quat.append([pos.z, pos.y, pos.x, ori.w, ori.x, ori.y, ori.z])

    transforms = np.array(transforms)
    traj_pos_quat = np.array(traj_pos_quat)
    time_stamp = np.array(time_stamp)
    time_stamp = time_stamp - time_stamp[0]
    bag.close()

    joint_trajectory = chain.inverse_trajectory(transforms, q0)

    fig = pv.figure()
    fig.plot_transform(s=0.3)
    graph = fig.plot_graph(
        kin.tm, "world", show_visuals=False, show_collision_objects=True,
        show_frames=True, s=0.1, whitelist=["world", "end_effector_link"])

    pv.Trajectory(transforms, s=0.05).add_artist(fig)

    fig.view_init()
    fig.animate(
        animation_callback, len(transforms), loop=True,
        fargs=(graph, chain, joint_trajectory))
    fig.show()

else:
    print("Using the joint trajectory")

    transforms_1 = []
    joint_trajectory_1 = []

    for topic, msg, t in bag.read_messages(topics=['/gravity_compensation_controller/traj_joint_states']):
        joint_trajectory_1.append(msg.position[:6])
        transforms_1.append(chain.forward(msg.position[:6]))

    transforms_1 = np.array(transforms_1)
    joint_trajectory_1 = np.array(joint_trajectory_1)

    # ---- Outlier removal ----
    positions = np.array([T[:3, 3] for T in transforms_1])
    mask, filtered_positions = remove_outliers_mad(positions, threshold=20.0) # threshold = 3.5
    filtered_transforms = transforms_1[mask]
    filtered_joint_trajectory = joint_trajectory_1[mask]

    bag.close()

    fig = pv.figure()
    fig.plot_transform(s=0.3)
    graph = fig.plot_graph(
        kin.tm, "world", show_visuals=False, show_collision_objects=True,
        show_frames=True, s=0.1, whitelist=["world", "end_effector_link"])

    pv.Trajectory(filtered_transforms, s=0.05).add_artist(fig)

    fig.view_init()
    fig.animate(
        animation_callback, len(filtered_transforms), loop=True,
        fargs=(graph, chain, filtered_joint_trajectory))
    fig.show()
