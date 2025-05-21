# Pick and Place Motion by Object Name with OpenManipulator 6DOF

This project demonstrates a **pick-and-place task** with the **OpenManipulator 6DOF** using **Cartesian Dynamic Movement Primitives (DMPs)** and **Intel RealSense** for object detection and pose estimation.  

All relevant scripts are located in the `/my_scripts_rl/` folder.

---

## Workflow Summary
1. **Run the pick-and-place execution script** `om_task_2.py`.
2. **Start the object detection node** by running `rs_detect.py`.
3. **Input the object name** (e.g., `one`, `two`, `three`) when prompted.
4. The robot will:
   - Detect the object using RealSense and YOLO.
   - Estimate its pose.
   - Perform a pick-and-place motion using pre-defined DMPs.

---

## Implementation Details

### Object Detection: `rs_detect.py`

This script handles object detection and pose estimation using YOLO and the RealSense camera.

- Subscribes to `/object_request` to receive the target object name.
- Uses YOLO to detect the object in the RGB stream.
- Computes 3D translation and orientation from depth and RGB.
- Publishes the object's pose via `/tf` for use by other nodes.


---

### Main Execution: `om_task_2.py`

This script controls the pick-and-place process using the `ContinuousOMNode` class.

#### __init__(self, joint_names, request_topic='/object_request', rate_hz=1)
- Initializes the ROS node.
- Creates a publisher to request object detection.
- Instantiates `ROS_OM_Node` to handle motion planning.
- Sets the loop rate for execution.

#### run_set_object(self)
- Interactive CLI loop for object-based picking.
- Workflow:
  1. Prompts user to input the object name (`one`, `two`, `three`, ...).
  2. Publishes this name to trigger detection.
  3. Waits for the pose (transform) of the object.
  4. Extracts the pick position via /tf `tf.TransformListener()`.
  5. Computes a fixed offset for the place position.
  6. Calls `execute_pick_and_place()` to move the arm.

> The function `execute_pick_and_place()`in the necessary `pick_from_vision_V3.py` file is extended to manage **gripper orientation**, always aligning it normally to the robot base for better IK solutions. The approach and departure angles can be customized via `pick_angle` and `place_angle`.

---

## Running on Real Robot vs. Simulation

1. **Start the simulation environment:**

```
roslaunch open_manipulator_6dof_gazebo open_manipulator_6dof_gazebo.launch controller:=position
```
```
roslaunch open_manipulator_6dof_controller open_manipulator_position_gazebo_controller.launch
```

2. **Start the RealSense detection node:**
```
cd src/my_scripts_rl/
python3 rl_detect.py
```

3. **Run the pick-and-place node:**

```
cd src/my_scripts_rl/
python3 om_task_2.py
```
4. **Input the object name** when prompted. The robot will perform the task in simulation.

### Real Robot

1. **Start the hardware controller:**

```
roslaunch om_position_controller position_control.launch
```

2. **Start the RealSense detection node:**
```
cd src/my_scripts_rl/
python3 rl_detect.py
```

3. **Run the pick-and-place node:**

```
cd src/my_scripts_rl/
python3 om_task_2.py
```
4. **Input the object name** when prompted. The robot will perform the task on real hardware.

---
### Notes
Supported object names are those defined and recognized by the YOLO model (one, two, three, etc.).

Ensure the RealSense node is active and publishing transforms (/tf) 
