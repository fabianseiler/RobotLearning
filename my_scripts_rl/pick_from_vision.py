import numpy as np
import pytransform3d.visualizer as pv
import pytransform3d.trajectories as ptr
from movement_primitives.kinematics import Kinematics
import rosbag
from tf.transformations import quaternion_matrix
from tf.transformations import quaternion_from_matrix
from movement_primitives.dmp import CartesianDMP
import tf
import pickle
import os
import time
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
import rospy
from sensor_msgs.msg import JointState

#-------------------------------------- Classes --------------------------------------# 

class DMPMotionGenerator:
    def __init__(self, urdf_path, mesh_path=None, joint_names=None, base_link="world", end_effector_link="end_effector_link"):
        """
        Initialize DMP Motion Generator
        
        Parameters:
        -----------
        urdf_path : str
            Path to the URDF file
        mesh_path : str, optional
            Path to mesh files
        joint_names : list, optional
            List of joint names to use
        base_link : str
            Name of the base link
        end_effector_link : str
            Name of the end effector link
        """
        self.urdf_path = urdf_path
        self.mesh_path = mesh_path
        self.kin = self._load_kinematics(urdf_path, mesh_path)
        self.joint_names = joint_names or ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.base_link = base_link
        self.end_effector_link = end_effector_link
        self.chain = self.kin.create_chain(self.joint_names, base_link, end_effector_link)
        self.dmp = None
        self.IK_joint_trajectory = None
        
    def _load_kinematics(self, urdf_path, mesh_path=None):
        """Load robot kinematics from URDF"""
        with open(urdf_path, 'r') as f:
            return Kinematics(f.read(), mesh_path=mesh_path)

    def learn_from_rosbag(self, bag_path, joint_topic, dt=None, n_weights=10):
        """Learn DMP from rosbag recording"""
        transforms, joint_trajectory,gripper_trajectory, time_stamp = self._process_rosbag(bag_path, joint_topic)
                
        # Convert transforms to PQS representation
        Y = ptr.pqs_from_transforms(transforms)
        if dt is None:
            dt = 1/self.frequency
        # Create and train DMP
        self.dmp = CartesianDMP(execution_time=max(time_stamp), dt=dt, n_weights_per_dim=n_weights)
        self.dmp.imitate(time_stamp, Y)
        
        return Y, transforms, joint_trajectory, gripper_trajectory

    def _process_rosbag(self, bag_path, joint_topic):
        """Process rosbag and extract trajectories"""
        transforms = []
        joint_trajectory = []
        gripper_trajectory = []
        time_stamp = []
        
        print(f"Reading bag file: {bag_path}")
        bag = rosbag.Bag(bag_path)
        for topic, msg, t in bag.read_messages(topics=[joint_topic]):
            joint_pos = msg.position[:6]
            gripper_pos = msg.position[6]
            joint_trajectory.append(joint_pos)
            gripper_trajectory.append(gripper_pos)

            transforms.append(self.chain.forward(joint_pos))
            time_stamp.append(msg.header.stamp.to_sec())    
        bag.close()
        
        # Convert to numpy arrays
        
        transforms = np.array(transforms)
        joint_trajectory = np.array(joint_trajectory)
        gripper_trajectory = np.array(gripper_trajectory)
        time_stamp = np.array(time_stamp)
        
        dt = []
        for i in range(1, time_stamp.shape[0]):
            dt.append(time_stamp[i]- time_stamp[i-1])
        self.frequency = 1/ np.average(np.array(dt))
        # print(f"Average frequency: { self.frequency}")
        # First filter outliers
        positions = np.array([T[:3, 3] for T in transforms])
        mask, _ = self.remove_outliers_mad(positions, threshold=12.0)
        
        # Then normalize time (important to do it in this order)
        filtered_time = time_stamp[mask]
        normalized_time = filtered_time - filtered_time[0]
        
        # print(f"Shape of filtered transforms: {transforms[mask].shape}")
        # print(f"Shape of time stamp: {normalized_time.shape}")
        
        return transforms[mask], joint_trajectory[mask], gripper_trajectory[mask] , normalized_time

    def remove_outliers_mad(self, data, threshold=3.5):
        """Remove outliers using Median Absolute Deviation"""
        median = np.median(data, axis=0)
        diff = np.abs(data - median)
        mad = np.median(diff, axis=0)
        modified_z_score = 0.6745 * diff / (mad + 1e-6)
        mask = np.all(modified_z_score < threshold, axis=1)
        return mask, data[mask]

    def generate_trajectory(self, start_y=None, goal_y=None):
        """
        Generate trajectory using the learned DMP
        
        Parameters:
        -----------
        start_y : array-like, shape (7,)
            Start state in PQS format [x,y,z,qw,qx,qy,qz]
        goal_y : array-like, shape (7,)
            Goal state in PQS format [x,y,z,qw,qx,qy,qz]
        """
        print(f"Generating trajectory")
        if self.dmp is None:
            raise ValueError("No DMP model available. Learn or load a model first.")
            
        if start_y is not None:
            self.dmp.start_y = start_y
            print(f"Using custom start: {start_y}")
        else:
            print(f"Using default start: {self.dmp.start_y}")
            
        if goal_y is not None:
            self.dmp.goal_y = goal_y
            print(f"Using custom goal: {goal_y}")
        else:
            print(f"Using default goal: {self.dmp.goal_y}")
        
        T, Y = self.dmp.open_loop()
        trajectory = ptr.transforms_from_pqs(Y)
        return T, trajectory

    def save_dmp(self, filepath):
        """Save the learned DMP to file"""
        if self.dmp is None:
            raise ValueError("No DMP model available to save")
        with open(filepath, 'wb') as f:
            pickle.dump(self.dmp, f)
        print(f"DMP saved to {filepath}")

    def load_dmp(self, filepath):
        """Load a DMP from file"""
        print(f"Loading DMP from {filepath}")
        with open(filepath, 'rb') as f:
            self.dmp = pickle.load(f)
        print(f"DMP loaded successfully")
    
    def compute_IK_trajectory(self, trajectory,  time_stamp, q0=None, subsample_factor=1): # 1
        if q0 is None:
            q0 = np.array([0.0, -0.78, 1.5, 0., 0.8, 0.])
        
        # Subsample the trajectory if requested
        if subsample_factor > 1:
            subsampled_trajectory = trajectory[::subsample_factor]
            subsampled_time_stamp = time_stamp[::subsample_factor]
             
            print(f"Subsampled time from {len(time_stamp)} to {len(subsampled_time_stamp)} points")
            print(f"Subsampled trajectory from {len(trajectory)} to {len(subsampled_trajectory)} points")
        else:
            subsampled_trajectory = trajectory
            subsampled_time_stamp = time_stamp
            
        print(f"Solving inverse kinematics for {len(subsampled_trajectory)} points...")
        
        start_time = time.time()
        
        # Use the same random state as in dmp_test_1.py
        random_state = np.random.RandomState(0)
        joint_trajectory = self.chain.inverse_trajectory(
            subsampled_trajectory,  random_state=random_state)
            
        print(f"IK solved in {time.time() - start_time:.2f} seconds")
        
        return subsampled_trajectory, joint_trajectory ,subsampled_time_stamp

   
    def _smooth_trajectory(self, trajectory, window_size=5):
        """Apply moving average smoothing to trajectory"""
        smoothed = np.copy(trajectory)
        half_window = window_size // 2
        
        for i in range(len(trajectory)):
            # Calculate window indices with boundary handling
            start = max(0, i - half_window)
            end = min(len(trajectory), i + half_window + 1)
            
            # Calculate average for each component of the pose
            for row in range(4):
                for col in range(4):
                    if row < 3 and col < 3:  # Only smooth rotation part
                        smoothed[i, row, col] = np.mean(trajectory[start:end, row, col])
                    elif col == 3:  # Position part
                        smoothed[i, row, col] = np.mean(trajectory[start:end, row, col])
        
        return smoothed

    def compute_IK_trajectory_KDL(self, trajectory, time_stamp, q0=None, max_iterations=1000, eps=1e-2):
        # Import necessary KDL modules
        try:
            import PyKDL
            from urdf_parser_py.urdf import URDF
            from kdl_parser_py.urdf import treeFromUrdfModel
        except ImportError:
            print("Error: PyKDL or URDF parser modules not found. Install with:")
            print("sudo apt-get install python3-pyKDL ros-noetic-kdl-parser-py ros-noetic-urdfdom-py")
            raise

        if q0 is None:
            q0 = np.array([0.0, -0.78, 1.5, 0., 0.8, 0.])
        
        start_time = time.time()
        
        # Load robot model from URDF
        robot_model = URDF.from_xml_file(self.urdf_path)
        success, kdl_tree = treeFromUrdfModel(robot_model)
        if not success:
            raise ValueError("Failed to construct KDL tree from URDF")
        
        # Create KDL Chain
        kdl_chain = kdl_tree.getChain(self.base_link, self.end_effector_link)
        num_joints = kdl_chain.getNrOfJoints()
        
        # Create KDL IK solvers
        fk_solver = PyKDL.ChainFkSolverPos_recursive(kdl_chain)
        ik_vel_solver = PyKDL.ChainIkSolverVel_pinv(kdl_chain)

        # Create joint limit arrays - initially set all to max range for simplicity
        # In a real application, you should get these from the URDF
        lower_limits = PyKDL.JntArray(num_joints)
        upper_limits = PyKDL.JntArray(num_joints)
        # Get joint limits from URDF
        for i, joint in enumerate(self.joint_names):
            # Find the joint in the robot model
            urdf_joint = None
            for j in robot_model.joints:
                if j.name == joint:
                    urdf_joint = j
                    break
            
            if urdf_joint and urdf_joint.limit:
                lower_limits[i] = urdf_joint.limit.lower
                upper_limits[i] = urdf_joint.limit.upper
            else:
                # Default limits if not found
                lower_limits[i] = -3.14
                upper_limits[i] = 3.14
        
        # Create the IK position solver with joint limits
        ik_solver = PyKDL.ChainIkSolverPos_NR_JL(
            kdl_chain, lower_limits, upper_limits, fk_solver, ik_vel_solver, 
            max_iterations, eps
        )
        
        # Initialize joint trajectory array
        joint_trajectory = np.zeros_like((len(trajectory), num_joints))
        
        # Set initial joint positions
        q_kdl = PyKDL.JntArray(num_joints)
        for i in range(min(len(q0), num_joints)):
            q_kdl[i] = q0[i]
        
        # Smooth the trajectory
        # smooth_traj = self._smooth_trajectory(trajectory)
        
        
        # Solve IK for each point in the trajectory
        for i in range(len(trajectory)):
            # Extract current pose
            pose = trajectory[i]
            
            #Convert to KDL Frame
            frame = PyKDL.Frame(
                PyKDL.Rotation(
                    pose[0, 0], pose[0, 1], pose[0, 2],
                    pose[1, 0], pose[1, 1], pose[1, 2],
                    pose[2, 0], pose[2, 1], pose[2, 2]
                ),
                PyKDL.Vector(pose[0, 3], pose[1, 3], pose[2, 3])
            )
            # frame = PyKDL.Frame(
            #     PyKDL.Rotation.Identity(),
            #     PyKDL.Vector(pose[0, 3], pose[1, 3], pose[2, 3])
            # )
            
            # Prepare output joint array
            q_out = PyKDL.JntArray(num_joints)
            
            # Solve IK
            result = ik_solver.CartToJnt(q_kdl, frame, q_out)
            
            if result < 0:
                print(f"Warning: IK failed at point {i} with error code {result}")
                # If the first point fails, use initial guess
                if i == 0:
                    for j in range(num_joints):
                        q_out[j] = q_kdl[j]
                # Otherwise use previous solution
                else:
                    for j in range(num_joints):
                        q_out[j] = joint_trajectory[i-1, j]
            
            # Store the solution
            for j in range(num_joints):
                joint_trajectory[i, j] = q_out[j]
            
            # Use this solution as the seed for the next point
            q_kdl = q_out
            
            # Progress indicator for long trajectories
            if i % 50 == 0 and i > 0:
                print(f"Solved {i}/{len(trajectory)} points...")
        
        print(f"KDL IK solved in {time.time() - start_time:.2f} seconds")
        
        return trajectory, joint_trajectory, time_stamp
        
    
    def visualize_trajectory(self, trajectory, joint_trajectory, q0=None ):
        """
        Visualize the generated trajectory with optional subsampling
        
        Parameters:
        -----------
        trajectory : array-like
            The trajectory to visualize as homogeneous transformation matrices
        q0 : array-like, optional
            Initial joint configuration for inverse kinematics
        subsample_factor : int, optional
            Factor by which to subsample the trajectory. 
            1 means use all points, 2 means use every second point, etc.
        """
        
        print(f"Plotting trajectory...")
        fig = pv.figure()
        fig.plot_transform(s=0.3)
        
        # Use the same whitelist as in dmp_test_1.py
        graph = fig.plot_graph(
            self.kin.tm, "world", show_visuals=False, show_collision_objects=True,
            show_frames=True, s=0.1, whitelist=[self.base_link, self.end_effector_link])

        # Plot start and end pose for clarity
        fig.plot_transform(trajectory[0], s=0.15)
        fig.plot_transform(trajectory[-1], s=0.15)
        
        # Always show the full trajectory in the visualization
        pv.Trajectory(trajectory, s=0.05).add_artist(fig)
        
        fig.view_init()
        fig.animate(
            animation_callback, len(trajectory), loop=True,
            fargs=(graph, self.chain, joint_trajectory))
        fig.show()


class ROS_OM_Node:
    def __init__(self, joint_names, topic_name='/gravity_compensation_controller/traj_joint_states', rate_hz=20): # 20
        if not rospy.core.is_initialized():
            rospy.init_node("om_pick_and_place", anonymous=True)
        self.publisher = rospy.Publisher(topic_name, JointState, queue_size=10)
        self.listener = tf.TransformListener()
        
        joint_names.append("gripper")
        # print(f"joint names: {joint_names}")
        self.joint_names = joint_names
        self.rate = rospy.Rate(rate_hz)
        print(f"[ROS] Initialized publisher on topic {topic_name} at {rate_hz}Hz")
        self.gripper = 0.01
        self.position = []

    def publish_trajectory(self, joint_trajectory, timestamps):
        """
        Publishes joint trajectory as JointState messages at fixed rate.

        Parameters:
        -----------
        joint_trajectory : np.ndarray
            Interpolated joint trajectory (M, D)
        timestamps : np.ndarray
            Corresponding timestamps (M,)
        """
        start_time = rospy.Time.now()
        for i in range(len(joint_trajectory)):
            if rospy.is_shutdown():
                break
            msg = JointState()
            msg.header.stamp = start_time + rospy.Duration.from_sec(timestamps[i] - timestamps[0])
            msg.name = self.joint_names
            self.position = joint_trajectory[i].tolist() + [self.gripper] 
            
            # position.append(0.0) # gripper
            # print(f"Position: {position}")
            vel_eff = np.zeros(7).tolist()
            msg.velocity =  vel_eff   
            msg.effort = vel_eff
            # print(f"velocity: {vel_eff}")
            msg.position = self.position
            self.publisher.publish(msg)
            self.rate.sleep()

    def set_gripper(self, gripper_position=0.01):
        self.gripper = gripper_position
        msg = JointState()
        msg.name = self.joint_names
        vel_eff = np.zeros(7).tolist()
        msg.velocity =  vel_eff   
        msg.effort = vel_eff 
        msg.position = self.position
        msg.position[-1] = self.gripper
        self.publisher.publish(msg)
        rospy.sleep(1.5)
        self.rate.sleep()

    def execute_pick_and_place(self,pick, place,pick_angle=0.0, place_angle=0.0):
        open_gripper = -0.01
        close_gripper_full = 0.01

        position_pick = pick
        position_place = place
        joint_traj_1, time_stamps_1 = gen_pick_and_place( '/root/catkin_ws/src/my_scripts_rl/recordings/dmp/home2pick.pkl',
                                                        pick_position=position_pick,
                                                        place_position=position_place,
                                                        pick_angle=pick_angle,
                                                        place_angle=place_angle
                                                        )
        # ROS Publishing
        try:
            self.gripper = open_gripper
            self.publish_trajectory(joint_traj_1[0], time_stamps_1[0])
            self.publish_trajectory(joint_traj_1[1], time_stamps_1[1])
            self.set_gripper(gripper_position=close_gripper_full)
            self.publish_trajectory(joint_traj_1[2], time_stamps_1[2])
            self.publish_trajectory(joint_traj_1[3], time_stamps_1[3])
            self.publish_trajectory(joint_traj_1[4], time_stamps_1[4])
            self.set_gripper(gripper_position=open_gripper)
            self.publish_trajectory(joint_traj_1[5], time_stamps_1[5])
            self.publish_trajectory(joint_traj_1[6], time_stamps_1[6])
           
        except rospy.ROSInterruptException:
            print("ROS publishing interrupted.")
    
    def get_object_pose_world(self,target_frame="world", object_frame="detected_object"):
        try:
            self.listener.waitForTransform(target_frame, object_frame, rospy.Time(0), rospy.Duration(0))
            (trans, rot) = self.listener.lookupTransform(target_frame, object_frame, rospy.Time(0))

            rospy.loginfo("Object position in %s frame:", target_frame)
            rospy.loginfo("  Translation: x=%.3f, y=%.3f, z=%.3f", *trans)
            rospy.loginfo("  Orientation (quaternion): x=%.3f, y=%.3f, z=%.3f, w=%.3f", *rot)

            return np.array(trans), np.array(rot)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform failed: %s", str(e))
            return None, None
# -------------------------------------- Helper functions --------------------------------------# 
def animation_callback(step, graph, chain, joint_trajectory):
    """Animation callback for visualization"""
    chain.forward(joint_trajectory[step])
    graph.set_data()
    return graph

def save_trajectory_data(joint_trajectory, timestamps, filepath):
    """
    Save trajectory data to a pickle file

    Parameters:
    -----------
    joint_trajectory : np.ndarray
        Joint trajectory array (N, D)
    timestamps : np.ndarray
        Timestamps array (N,)
    filepath : str
        Path to save the pickle file
    """
    data = {
        'trajectory': joint_trajectory,
        'timestamps': timestamps
    }
    with open(filepath, 'wb') as f:
        pickle.dump(data, f)
    print(f"[SAVE] Trajectory data saved to {filepath}")

def load_trajectory_data(filepath):
    """
    Load trajectory data from a pickle file

    Parameters:
    -----------
    filepath : str
        Path to load the pickle file

    Returns:
    --------
    joint_trajectory : np.ndarray
        Loaded joint trajectory
    timestamps : np.ndarray
        Loaded timestamps
    """
    with open(filepath, 'rb') as f:
        data = pickle.load(f)
    
    joint_trajectory = data['trajectory']
    timestamps = data['timestamps']
    print(f"[LOAD] Loaded trajectory from {filepath} (length={len(joint_trajectory)})")
    return joint_trajectory, timestamps

def interpolate_joint_trajectory(joint_traj,  time_stamps, target_freq=20.0):
    """
    Interpolate joint trajectory to the target frequency

    Parameters:
    -----------
    joint_traj : np.ndarray
        Original joint positions (N, D)
    time_stamps : np.ndarray
        Original timestamps (N,)
    target_freq : float
        Target frequency in Hz

    Returns:
    --------
    interp_traj : np.ndarray
        Interpolated joint trajectory (M, D)
    new_timestamps : np.ndarray
        New timestamps (M,)
    """
    num_joints = joint_traj.shape[1]
    duration = time_stamps[-1] - time_stamps[0]
    num_samples = int(duration * target_freq)
    new_timestamps = np.linspace(time_stamps[0], time_stamps[-1], num_samples)
    
    interp_traj = np.zeros((num_samples, num_joints))
    for i in range(num_joints):
        interpolator = interp1d(time_stamps, joint_traj[:, i], kind='linear', fill_value="extrapolate")
        interp_traj[:, i] = interpolator(new_timestamps)
    
    return interp_traj, new_timestamps

def gen_trajectory(dmp_path, start=np.array([0,0,0,0,0,0,0]), goal=np.array([0,0,0,0,0,0,0]),visualize=False,store_cart_traj=False, name=''):
    urdf_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf'
    mesh_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/meshes'
    bag_path = '/root/catkin_ws/src/my_scripts_rl/recordings/pick.bag'

    dmp_gen = DMPMotionGenerator(
        urdf_path, 
        mesh_path,
        base_link="world"
    )
    dmp_gen.load_dmp(dmp_path)
    
    ## Generate new trajectory
    
    # Define new goal 
    if np.array_equal(start,np.array([0,0,0,0,0,0,0])):
         new_start = dmp_gen.dmp.start_y.copy()
    else:
        new_start = start

    if np.array_equal(goal,np.array([0,0,0,0,0,0,0])):
         new_goal = dmp_gen.dmp.goal_y.copy()
    else:
        new_goal = goal
 
    print(f"New goal: {new_goal}")
    # Generate
    T, trajectory = dmp_gen.generate_trajectory(start_y=new_start, goal_y=new_goal)

    # Store cartesian Trajectory
    if store_cart_traj:
        store_cart_traj_path = dmp_path.replace('/dmp/', '/cart_traj/')
        store_cart_traj_path = store_cart_traj_path.replace('.pkl', f'_{name}.pkl')
        save_trajectory_data(trajectory, T,store_cart_traj_path)

    # Visualize the trajectory
    trajectory, IK_joint_trajectory ,T = dmp_gen.compute_IK_trajectory(trajectory, T ,subsample_factor=100) # 10
    #trajectory, IK_joint_trajectory, T = dmp_gen.compute_IK_trajectory_KDL(trajectory, T)
    if visualize == True:
        dmp_gen.visualize_trajectory(trajectory, IK_joint_trajectory)
    
    traj_length = IK_joint_trajectory.shape[0]
    # Algin length of gripper traj and generated traj
    #gripper_traj = gripper_traj[:traj_length]
    IK_joint_trajectory = IK_joint_trajectory[:traj_length,:]
    
    #full_trajectory = np.hstack((IK_joint_trajectory, gripper_traj.reshape(-1, 1)))
    # # Interpolate to 20Hz and Save
    interpolated_traj, interpolated_time = interpolate_joint_trajectory(IK_joint_trajectory, T, target_freq=20.0)
    #save_trajectory_data(interpolated_traj, interpolated_time, "/root/catkin_ws/recordings/traj/interpolated_traj.pkl")

    # Later, you can reload and publish it
    #joint_traj, time_stamps = load_trajectory_data("/root/catkin_ws/recordings/traj/interpolated_traj.pkl")
    joint_traj = interpolated_traj
    time_stamps = interpolated_time

    

    return joint_traj, time_stamps, new_goal

def gripper_orientation_pick(position):
    """
    Computes orientation such that the gripper is allway in pointing down the local z axis
    and the x-axis is alway pointing toward the origin of the xy-plane

    Args:
        position (np.ndarray): [x, y, z] in world frame
    
    Returns:
        np.ndarray: Quaternion [x, y, z, w]
    """
    x_, y_, z_ = position
    x = z_
    y = x_
    z = y_

    gripper_x = np.array([-1.0, 0.0, 0.0])

    to_origin_zy = np.array([0.0, -y, -z])
    norm = np.linalg.norm(to_origin_zy)
    if norm < 1e-6:
        raise ValueError("Position too close to origin in ZY plane; Y-axis undefined.")
    gripper_y = to_origin_zy / norm

    gripper_z = np.cross(gripper_x, gripper_y)

    # Re-orthogonalize (for numerical stability)
    gripper_y = np.cross(gripper_z, gripper_x)
    gripper_y /= np.linalg.norm(gripper_y)
    gripper_z /= np.linalg.norm(gripper_z)

    # Build homogeneous rotation matrix
    rot_matrix = np.eye(4)
    rot_matrix[:3, 0] = gripper_x  # local x
    rot_matrix[:3, 1] = gripper_y  # local y
    rot_matrix[:3, 2] = gripper_z  # local z

    quat = quaternion_from_matrix(rot_matrix)
    pose = np.concatenate([position, quat])
    return pose

def rotate_pose_around_y(pose, phi=0.0):
    # Extract position and orientation
    position = pose[:3]
    orientation_quat = pose[3:]

    # Convert to scipy Rotation object
    r_orig = R.from_quat(orientation_quat)

    # Create Y-axis rotation quaternion (in world or local frame depending on order)
    r_y = R.from_euler('y', phi)

    # Apply rotation: r_y * r_orig rotates the pose in world frame
    r_new = r_y * r_orig  # For world frame rotation

    # If you want to rotate in local pose frame, use: r_new = r_orig * r_y

    # Combine position with new orientation
    new_pose = np.concatenate([position, r_new.as_quat()])

    return new_pose

def gen_pick_and_place(path, pick_position, place_position, pick_angle = 0.0, place_angle=0.0):
    """
    Computes orientation such that the gripper is allway in pointing down the local z axis
    and the x-axis is alway pointing toward the origin of the xy-plane

    Args:
        path (str): "path_to_the_dmp.pkl"
        pick_position (np.ndarray): [x, y, z] in world frame
        place_position (np.ndarray): [x, y, z] in world frame
    
    Returns:
        np.ndarray: joint trajectories 
    """

    #up_orientation = np.array([0.3826834, 0, 0.9238795, 0 ])
    position_home = np.array([0.05 ,0 ,0.17]) 
    #pose_home = np.concatenate([position_home, up_orientation])
    joint_traj = np.empty(7,dtype=object)
    time_stamps = np.empty(7,dtype=object)

    # HOME --> PICK MOTION Above
    dmp_path = '/root/catkin_ws/src/my_scripts_rl/recordings/dmp/home2pick.pkl'
    position = pick_position.copy()
    position[2] = 0.2
    pose = gripper_orientation_pick(position)
    pose = rotate_pose_around_y(pose,np.deg2rad(45))
    joint_traj[0], time_stamps[0], goal = gen_trajectory(dmp_path,goal=pose,visualize=False, store_cart_traj=False, name='pick1')
    #save_trajectory_data(joint_traj[0], time_stamps[0], "/root/catkin_ws/src/my_scripts_rl/recordings/traj/traj_home2pick.pkl")

    # Pick above --> Pick
    new_start = goal
    position = pick_position
    pose = gripper_orientation_pick(position)
    pose =  rotate_pose_around_y(pose,pick_angle)
    joint_traj[1], time_stamps[1], goal = gen_trajectory(dmp_path,start=new_start,goal=pose,visualize=False, store_cart_traj=False, name='pick2')

    # Pick --> PickUp
    new_start = goal
    position = pick_position.copy()
    position[2] = 0.2
    pose = gripper_orientation_pick(position)
    pose = rotate_pose_around_y(pose,np.deg2rad(45))
    joint_traj[2], time_stamps[2], goal = gen_trajectory(dmp_path,start=new_start,goal=pose,visualize=False, store_cart_traj=False, name='move')

    # PickUp --> PlaceUp
    new_start = goal
    position = place_position.copy()
    position[2] = 0.2
    pose = gripper_orientation_pick(position)
    pose = rotate_pose_around_y(pose,np.deg2rad(45))
    joint_traj[3], time_stamps[3], goal = gen_trajectory(dmp_path,start=new_start,goal=pose,visualize=False, store_cart_traj=False, name='place1')

    # PlaceUp --> PlaceDown
    new_start = goal
    position = place_position
    pose = gripper_orientation_pick(position)
    pose = rotate_pose_around_y(pose,place_angle)
    joint_traj[4], time_stamps[4], goal = gen_trajectory(dmp_path,start=new_start,goal=pose,visualize=False, store_cart_traj=False, name='place2')

    # PlaceDown --> PlaceUp
    new_start = goal
    position = place_position.copy()
    position[2] = 0.2
    pose = gripper_orientation_pick(position)
    pose = rotate_pose_around_y(pose,np.deg2rad(45))
    joint_traj[5], time_stamps[5], goal = gen_trajectory(dmp_path,start=new_start,goal=pose,visualize=False, store_cart_traj=False, name='place3')

    # PlaceUp --> Home
    new_start = goal
    position = position_home
    pose = gripper_orientation_pick(position)
    pose = rotate_pose_around_y(pose,np.deg2rad(45))
    joint_traj[6], time_stamps[6], goal = gen_trajectory(dmp_path,start=new_start,goal=pose,visualize=False, store_cart_traj=False, name='home')


    return joint_traj, time_stamps

# -------------------------------------- MAIN --------------------------------------# 
if __name__ == "__main__":
    try:
        om_node = ROS_OM_Node(['joint1', 'joint2','joint3','joint4','joint5','joint6'])

    except rospy.ROSInterruptException:
        print("ROS node interrupted.")

    # Get object pose
    #object_position, object_orienation = om_node.get_object_pose_world()

    # Execute given pick position
    position_pick = np.array([0.0, 0.2, 0.0])
    pick_angle = np.deg2rad(0)
    place_angle = np.deg2rad(0)
    position_place = np.array([0.12, -0.12, 0.0])
    om_node.execute_pick_and_place(position_pick,
                                   position_place,
                                   pick_angle,      
                                   place_angle
                                  )

    
