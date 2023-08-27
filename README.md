# hunter_ws from AIV

<p align="center">
  <img src="https://github.com/AIV-Autonomous-Intelligent-Vehicles/hunter_ws/assets/113443261/aea97be9-3727-415e-bfa2-a2239a522dba" alt="Team Image" width="500"/>
</p>



## About Us
Welcome to the official repository of AIV (Autonomous Intelligent Vehicles), the self-driving car club at Sejong University.  

We are a passionate group of students from Sejong University dedicated to the field of autonomous driving. Our mission is to explore, learn, and innovate within the realm of self-driving technology.<br><br>
## Development Environment

- Operating System: Ubuntu 20.04
- ROS Version: ROS 1 Noetic
- Matlab: 2023a
<br><br>
## Simulation Environment

We have utilized the **hunter2** model provided by [agilexrobotics](https://github.com/agilexrobotics/ugv_gazebo_sim) as the foundation for our simulation environment. To enhance its capabilities, we have integrated the following sensors:

- GPS
- IMU
- Ouster 64-channel LiDAR
- Left, Right Camera
<br><br>
## YOLOv5 Cone Detection and Path Planning

Our repository includes a built-in YOLOv5 model that has been trained to detect yellow and blue cones. Here's a deeper insight into our process:

- **Cone Detection with YOLOv5**: We trained the YOLOv5 model to detect both yellow and blue cones.

- **Lidar-Camera Calibration**:Post detection, we use lidar-camera calibration to cluster the detected regions of the cones.

- **Path Generation using Delaunay Triangulation**: We employ the Delaunay Triangulation method to generate an optimal path amongst the cones.

- **Path Tracking with Pure Pursuit**:The generated path is then tracked using the Pure Pursuit algorithm to ensure our vehicle follows the designated path while avoiding the cones.

### MATLAB Example Integration
We have also incorporated a MATLAB example that demonstrates the entire process – from cone detection to path tracking. Interested users can access and run the example from `~/hunter_ws/matlab/TrackExample.m`.
<p align="center">
  <img src="https://github.com/AIV-Autonomous-Intelligent-Vehicles/hunter_ws/assets/113443261/0a256086-1a51-4335-b14f-c5fa9bcaf9bb" alt="Team Image" width="500"/>
</p>
<br><br>

## Getting Started

To begin with our project, follow these steps:

### 1. Install ROS Dependencies

Execute the following commands to install ROS dependencies:

```bash
sudo apt-get install ros-noetic-ros-control -y
sudo apt-get install ros-noetic-ros-controllers -y
sudo apt-get install ros-noetic-gazebo-ros -y
sudo apt-get install ros-noetic-gazebo-ros-control -y
sudo apt-get install ros-noetic-joint-state-publisher-gui -y
sudo apt-get install ros-noetic-rqt-robot-steering -y
sudo apt-get install ros-noetic-hector-gazebo-plugins -y
sudo apt install ros-noetic-ackermann-steering-controller -y
sudo apt install ros-noetic-cob-perception-msgs -y # for yolo
sudo apt install ros-noetic-mavros-msgs -y # for gps
sudo apt install sharutils -y # for imu
pip install --upgrade python-dateutil # for yolo
```
### 2. Install Hunter2 Model

Create the hunter_ws directory and install the Hunter2 model using the following commands:

```bash
cd ~/
git clone https://github.com/AIV-Autonomous-Intelligent-Vehicles/hunter_ws.git
```
### 3. Set Up Dependencies and Build
Install necessary dependencies and build the project using the following commands:

```bash
cd ~/hunter_ws
source /opt/ros/noetic/setup.bash
pip install -r ./src/simulation/yolov5_ros/src/yolov5/requirements.txt
sudo chmod 777 -R ./src/realworld/imu/xsens_ros_mti_driver
pushd ./src/realworld/imu/xsens_ros_mti_driver/lib/xspublic && make && popd 
chmod +x ./src/simulation/yolov5_ros/src/detect.py
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

### 4. Run Gazebo Simulation

```bash
# Gazebo
cd ~/hunter_ws
source devel/setup.bash

roslaunch hunter2_gazebo hunter2_main.launch
```
### 5. Matlab Example
`~/hunter_ws/matlab/TrackExample.m`

![ezgif-4-bbe45ece8a](https://github.com/AIV-Autonomous-Intelligent-Vehicles/hunter_ws/assets/113443261/6eccb7c3-4d87-43f0-bf42-9e74edaf6abd)
<br><br>
## Contact Us

If you're interested in our work or have any questions, feel free to reach out to us:

- Follow us on social media: [Instagram](https://instagram.com/aiv_sejong?igshid=OGQ5ZDc2ODk2ZA==)

We look forward to collaborating with you on this incredible adventure towards autonomous driving excellence!



