
## Pick and Place Motion Generation with OpenManipulator 6DOF

This project provides guidelines for executing a pick-and-place task with the OpenManipulator 6DOF using Cartesian Dynamic Movement Primitives (DMPs). All related code is located in the `my_scripts_rl` folder. The main script for running the task is `pick_and_place_gen.py`.

## Implementation

The DMPs are created using the `dmp_motion.py` script and stored in the `my_scripts_rl_recordings/dmp` directory. The pick-and-place task consists of two primitive motions: one from the home position to the target (pick/place) location, and another returning from the target to the home position. By modifying the start and end poses (position + orientation), new trajectories can be generated using DMPs within the `pick_and_place_gen.py` script.

In the `main` function of `pick_and_place_gen.py`, task-specific trajectories are generated using the DMPs. Each sub-trajectory starts at the end pose of the previous one. Based on the logic in `dmp_motions.py`, inverse kinematics are applied to convert Cartesian DMPs into joint-space trajectories. These generated joint-space and Cartesian trajectories are saved in `my_scripts_rl_recordings/traj` and `my_scripts_rl_recordings/cart_traj`, respectively.
To support easier trajectory generation and future task extensions, the `gen_trajectory()` function was introduced, providing a streamlined pipeline for creating new trajectories.

To publish trajectories to the robot, the `publish_trajectory` function from `ROSTrajectoryPublisher` is used. It has been extended to handle gripper operations separately through the `set_gripper()` function, which manages opening and closing actions independently.

## Executing on the Real Robot

### Simulation

To start the simulation controller, execute:

```
roslaunch open_manipulator_6dof_gazebo open_manipulator_6dof_gazebo.launch controller:=position
roslaunch open_manipulator_6dof_controller open_manipulator_position_gazebo_controller.launch
```

To execute the pick-and-place motion, run:

```
cd src/my_scripts_rl/
python3 pick_and_place_gen.py
```

The system is now ready to play a generated trajectory.

### Real Robot

To deploy the generated trajectory on the real robot, execute:

```
roslaunch om_position_controller position_control.launch
```

Then, run:

```
cd src/my_scripts_rl/
python3 pick_and_place_gen.py
```

The trajectory is now ready to be executed on the real hardware.

---

