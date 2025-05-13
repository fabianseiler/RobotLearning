import numpy as np
import pytransform3d.visualizer as pv
import pytransform3d.trajectories as ptr
from movement_primitives.kinematics import Kinematics
import rosbag
from tf.transformations import quaternion_matrix
from movement_primitives.dmp import CartesianDMP
import pickle
import os
import time
from scipy.interpolate import interp1d
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
    
    def compute_IK_trajectory(self, trajectory,  time_stamp, q0=None, subsample_factor=1):
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
            
            # Convert to KDL Frame
            frame = PyKDL.Frame(
                PyKDL.Rotation(
                    pose[0, 0], pose[0, 1], pose[0, 2],
                    pose[1, 0], pose[1, 1], pose[1, 2],
                    pose[2, 0], pose[2, 1], pose[2, 2]
                ),
                PyKDL.Vector(pose[0, 3], pose[1, 3], pose[2, 3])
            )
            
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


class ROSTrajectoryPublisher:
    def __init__(self, joint_names, topic_name='/gravity_compensation_controller/traj_joint_states', rate_hz=20): # 20
        rospy.init_node("dmp_trajectory_publisher", anonymous=True)
        self.publisher = rospy.Publisher(topic_name, JointState, queue_size=10)
        
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
        rospy.sleep(3)
        self.rate.sleep()


    def publish_gripper(self, position_value=0.01):
        self.gripper = position_value
       # msg = JointState()
       # msg.header.stamp = rospy.Time.now()
       # msg.name = ["gripper"]  # Ensure it matches the actual URDF joint name
       # msg.position = [position_value]
       # msg.velocity = [0.0]
       # msg.effort = [0.0]
        
       # self.publisher.publish(msg)
       # print(f"[ROS] Published gripper position: {position_value}")
       # self.rate.sleep()
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
    trajectory, IK_joint_trajectory ,T = dmp_gen.compute_IK_trajectory(trajectory, T ,subsample_factor=10)
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

# -------------------------------------- MAIN --------------------------------------# 
if __name__ == "__main__":
    open_gripper = -0.01
    close_gripper_full = 0.01

    # PICK MOTION
    dmp_path = '/root/catkin_ws/src/my_scripts_rl/recordings/dmp/home2pick.pkl'
    new_goal = np.array([0.0, -0.2, 0.0  ,  0, 0.7071068, 0.7071068, 0 ])
    joint_traj_pick, time_stamps_pick, goal = gen_trajectory(dmp_path,goal=new_goal,visualize=False, store_cart_traj=True, name='pick')
    save_trajectory_data(joint_traj_pick, time_stamps_pick, "/root/catkin_ws/src/my_scripts_rl/recordings/traj/traj_home2pick.pkl")

    # UP MOTION
    dmp_path = '/root/catkin_ws/src/my_scripts_rl/recordings/dmp/pick2home.pkl'
    new_start = np.array([0.0, -0.2, 0.0  ,  0, 0.7071068, 0.7071068, 0 ])
    joint_traj_up, time_stamps_up, goal = gen_trajectory(dmp_path,start=new_start,visualize=False,store_cart_traj=True, name="move_with_cube")
    save_trajectory_data(joint_traj_up, time_stamps_up,  "/root/catkin_ws/src/my_scripts_rl/recordings/traj/pick2home.pkl")


    # PLACE MOTION
    dmp_path = '/root/catkin_ws/src/my_scripts_rl/recordings/dmp/home2pick.pkl'
    new_start = goal
    new_goal = np.array([0.14, 0.14, 0,   0, 0.7071068, -0.7071068, 0])
    joint_traj_place, time_stamps_place, goal = gen_trajectory(dmp_path,start=new_start,goal=new_goal,visualize=False,store_cart_traj=True, name='place_cube')
    save_trajectory_data(joint_traj_place, time_stamps_place,  "/root/catkin_ws/src/my_scripts_rl/recordings/traj/traj_home2place.pkl")


    # HOME MOTION
    dmp_path = '/root/catkin_ws/src/my_scripts_rl/recordings/dmp/pick2home.pkl'
    new_start = goal
    joint_traj_home, time_stamps_home, goal = gen_trajectory(dmp_path,start=new_start,visualize=False,store_cart_traj=True, name='back_home')
    save_trajectory_data(joint_traj_home, time_stamps_home,  "/root/catkin_ws/src/my_scripts_rl/recordings/traj/traj_place2home.pkl")


    # ROS Publishing
    try:
        publisher = ROSTrajectoryPublisher(['joint1', 'joint2','joint3','joint4','joint5','joint6'])
        publisher.publish_gripper(open_gripper)
        publisher.publish_trajectory(joint_traj_pick, time_stamps_pick)
        publisher.set_gripper(gripper_position=close_gripper_full)
        publisher.publish_trajectory(joint_traj_up, time_stamps_up)
        publisher.publish_trajectory(joint_traj_place, time_stamps_place)
        publisher.set_gripper(gripper_position=open_gripper)
        publisher.publish_trajectory(joint_traj_home, time_stamps_home)
        
    except rospy.ROSInterruptException:
        print("ROS publishing interrupted.")