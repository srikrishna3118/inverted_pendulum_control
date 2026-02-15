# inverted_pendulum_control

Simple PID control of an inverted pendulum and a mobile robot, migrated to ROS 2 (Humble) for Ubuntu 22.04.

## Prerequisites (Ubuntu 22.04)

Install ROS 2 Humble and source it:

```bash
source /opt/ros/humble/setup.bash
```

Install required packages:

```bash
sudo apt update
sudo apt install -y \
	ros-humble-gazebo-ros-pkgs \
	ros-humble-gazebo-ros2-control \
	ros-humble-ros2-control \
	ros-humble-ros2-controllers \
	ros-humble-xacro \
	ros-humble-robot-state-publisher
```

## Build

From the workspace root:

```bash
cd ~/catkin_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select inverted_pendulum_control
source install/setup.bash
```

## Run

Launch Gazebo, spawn models, load controllers, and start the PID node:

```bash
ros2 launch inverted_pendulum_control myworld.launch.py
```

## Notes

- The pendulum controller command topic is `/lip/joint_effort_controller_j_0/commands` (ROS 2 controller interface).
- The joint state feedback topic is `/lip/joint_states`.

