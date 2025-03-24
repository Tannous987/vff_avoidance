# Toni-VFF-Avoidance

## Project Description
This project implements the Virtual Force Field (VFF) algorithm to enable a TurtleBot to move forward while avoiding obstacles. The algorithm uses three vectors: an attractive vector (to move forward), a repulsive vector (to avoid obstacles), and a resultant vector (to determine the robot's movement). The implementation is done in ROS 2, and the node publishes visual markers for debugging purposes in Rviz2.

## Group Members
- Toni Tannous

## Setup Instructions

### 1. Create Workspace and Source Folder
First, create a ROS 2 workspace and a `src` folder inside it:
```bash
mkdir -p ~/vff_ws/src
cd ~/vff_ws/src
```

### 2. Clone the Repository
Clone the GitHub repository containing the VFF avoidance code:
```bash
git clone https://github.com/Tannous987/vff_avoidance.git
```
### 3. Build the Workspace
Navigate back to the workspace root and build the package:
```bash
cd ~/vff_ws
colcon build
```

### 4. Launch the Avoidance Node
Launch the VFF avoidance node:
```bash
echo "source ~/vff_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
ros2 launch vff_avoidance avoidance_vff.launch.py
```
### 6. Run Rviz2
Open Rviz2 to visualize the robot's behavior and the debugging markers:
```bash
rviz2
```

### 7. Test the Algorithm
Publish a sample LaserScan message to test the obstacle avoidance behavior:
```bash
ros2 topic pub /scan sensor_msgs/msg/LaserScan "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
  angle_min: -1.57,
  angle_max: 1.57,
  angle_increment: 0.0175,
  time_increment: 0.0,
  scan_time: 0.1,
  range_min: 0.1,
  range_max: 10.0,
  ranges: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
           10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
           10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
           10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
           10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
           10.0, 10.0, 10.0, 10.0,10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
           10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
           10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
           10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
           10.0, 10.0, 10.0, 10.0, 10.0,
           10.0, 10.0, 10.0, 10.0, 10.0, 0.8, 10.0, 10.0, 10.0, 10.0,
           10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
  intensities: []
}" --once
```
### 8. Gazebo Simulation
To simulate the robot in Gazebo, follow these steps:

1. **Set the TurtleBot3 Model**:
   Before running the simulation, set the TurtleBot3 model (e.g., `burger`, `waffle`, or `waffle_pi`). For example:
   ```bash
   export TURTLEBOT3_MODEL=burger
   ```
2. **Launch Gazebo with TurtleBot3:**:
   Run the following command to launch Gazebo with the TurtleBot3 simulation:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

