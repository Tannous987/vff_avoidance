# TCR-VFF-Avoidance

## Project Description
This project implements the Virtual Force Field (VFF) algorithm to enable a TurtleBot to move forward while avoiding obstacles. The algorithm uses three vectors: an attractive vector (to move forward), a repulsive vector (to avoid obstacles), and a resultant vector (to determine the robot's movement). The implementation is done in ROS 2, and the node publishes visual markers for debugging purposes in Rviz2.

## Group Members
- Toni Tannous
- Charbel Abi Saad
- Rowan

## Setup Instructions

### 1. Create Workspace and Source Folder
First, create a ROS 2 workspace and a `src` folder inside it:
```bash
mkdir -p ~/vff_ws/src
cd ~/vff_ws/sr
'''

### 2. Clone the Repository
Clone the GitHub repository containing the VFF avoidance code:
```bash
git clone https://github.com/Tannous987/vff_avoidance.git
'''
