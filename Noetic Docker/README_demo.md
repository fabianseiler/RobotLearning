# Open Manipulator 6DOF

This repository contains all the necessary packages to control and simulate the Open Manipulator 6DOF robot using ROS.
### Docker Setup

This project uses Docker to create a consistent environment with all the dependencies pre-installed.

1. **Build the Docker image**:
   ```bash
   docker build -t om_6 .
   ```

2. **Verify the image was created**:
   ```bash
   docker images
   ```

3. **Run the Docker container** with GPU and display support:
   ```bash
   docker run -it \
       --gpus all \
       --env="DISPLAY" \
       --env="QT_X11_NO_MITSHM=1" \
       --env="NVIDIA_VISIBLE_DEVICES=all" \
       --env="NVIDIA_DRIVER_CAPABILITIES=all" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
       --net=host \
       --privileged \
       om_6
   ```

4. **If GUI applications are not displaying**, run this on your host PC:
   ```bash
   xhost +local:root
   ```

### Working with Multiple Terminals

1. **Find your running container**:
   ```bash
   docker ps
   ```

2. **Connect to the running container** from a new terminal:
   ```bash
   docker exec -it <container_name> bash
   # Example: docker exec -it affectionate_lichterman bash
   ```

3. Alternatively, use **Terminator** inside the container to create split terminals.

## Basic Usage

The first thing to do in any new terminal session inside the container:

```bash
source /catkin_ws/devel/setup.bash
```

### Running in Simulation

1. **Start ROS master** in one terminal:
   ```bash
   roscore
   ```

2. **Launch the controller** in a new terminal:
   ```bash
   roslaunch open_manipulator_6dof_controller open_manipulator_6dof_controller.launch use_platform:=false
   ```

3. **Launch Gazebo simulation** in another terminal:
   ```bash
   roslaunch open_manipulator_6dof_gazebo open_manipulator_6dof_gazebo.launch controller:=position
   ```

4. **Launch the GUI control panel**:
   ```bash
   roslaunch open_manipulator_6dof_control_gui open_manipulator_6dof_control_gui.launch
   ```

5. **Run example scripts**:
   ```bash
   cd src/open_manipulator_friends/open_manipulator_6dof_controller/scripts/
   python example.py
   ```

### Running on Real Robot

1. **Start ROS master** in one terminal:
   ```bash
   roscore
   ```

2. **Launch the controller** with hardware communication:
   ```bash
   roslaunch open_manipulator_6dof_controller open_manipulator_6dof_controller.launch use_platform:=true dynamixel_usb_port:=/dev/ttyUSB0
   ```
   Note: Adjust the USB port if your device is connected to a different port.

3. **Launch the GUI control panel (after homing stop it)**:
   ```bash
   roslaunch open_manipulator_6dof_control_gui open_manipulator_6dof_control_gui.launch
   ```

4. **Run example scripts**:
   ```bash
   cd src/open_manipulator_friends/open_manipulator_6dof_controller/scripts/
   python example.py
   ```

## Recording and Playback

### Preparing for Recording

1. Ensure the robot is in a known position before recording.

2. Use Dynamixel Wizard to set all motors to current control mode.

3. Launch the gravity compensation controller:
   ```bash
   roslaunch open_manipulator_controllers gravity_compensation_controller.launch sim:=false
   ```

4. Start end-effector position republishing:
   ```bash
   cd src/open_manipulator_friends/open_manipulator_6dof_controller/scripts/
   python republish_pose.py
   ```

5. Verify position data is being published:
   ```bash
   rostopic echo /current_pose
   ```

### Recording Movements

1. Create a directory for recordings (if not exists):
   ```bash
   mkdir -p ~/catkin_ws/recordings
   ```

2. Start recording all topics to a ROS bag:
   ```bash
   rosbag record -a -O ~/catkin_ws/recordings/move1.bag
   ```

3. Move the robot as desired (manually or through other control methods).

4. Press Ctrl+C to stop recording.

### Examining and Playing Back Recordings

1. View information about the recorded bag:
   ```bash
   rosbag info ~/catkin_ws/recordings/move1.bag
   ```

2. Play back the recorded movements:
   ```bash
   rosbag play ~/catkin_ws/recordings/move1.bag
   ```

## Troubleshooting

- If Gazebo does not display, run `xhost +local:root` on the host machine.
- Check USB permissions if the robot is not detected.
- Ensure all dynamixel motors are in the correct control mode before recording.
- Verify the USB port in controller launch file matches your hardware setup.



## Acknowledgments

- ROBOTIS for the Open Manipulator platform
- ROS community for the control frameworks
