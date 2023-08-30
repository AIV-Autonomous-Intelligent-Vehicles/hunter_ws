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
- Left, Right Camera<br>

### Future Enhancements for Real-World Implementation

While our primary focus is on the simulation environment, we are actively working towards implementing track driving in a real-world scenario. To achieve this, we are incorporating various sensors to enhance perception capabilities.   
You can find the additional files and information for the sensors intended for real-world use at the `~/hunter_ws/src/realworld` directory.



<details>
<summary>GPS (**C099-F9P** with applied RTK correction signals)</summary>
<div markdown="1">
  
```bash
roslaunch ntrip_client ntrip_client.launch # RTK
roslaunch ublox_gps ublox_device.launch    # GPS
```

</div>
</details>

<details>
<summary>IMU (Xsens mti-3 AHRS)</summary>
<div markdown="1">
  
```bash
roslaunch xsens_mti_driver display.launch
```

</div>
</details>




<details>
<summary>Lidar (Ouster OS-1 64ch)</summary>
<div markdown="1">
  
```bash
roslaunch ouster_ros sensor.launch sensor_hostname:=192.168.6.11 upd_dest:=192.168.6.99
```

</div>
</details>
  
<details>
<summary>Camera</summary>
<div markdown="1">

```bash
# pc에 연결된 cam port의 이름이 무엇인지 확인을 해야한다.
# 필자는 다음과 같은 명령어를 실행했을 때, /dev/video0, /dev/video1 이렇게 두가지가 나왔음 
ls /dev/video* 

# root 권한을 추가하기 위해 내 desktop의 이름을 확인한다. 
whoami

# root 권한으로 카메라를 사용할 수 있도록 허용한다. 
sudo adduser (whoami로 출력된 사용자 이름) video

# 리눅스에서 비디오 및 오디오 장치를 지원하기 위한 API와 도구 모음을 설치한다. 
sudo apt-get install v4l-utils -y 

# 현재 장착되어 있는 usb들을 확인을 한다.(HD Pro WebCam  C920 아래에 출력된 /dev/video* 들을 가지고 무엇이 맞는지 확인할 것임)  
v4l2-ctl --list-devices

# 카메라가 지닌 파라미터를 확인하는 명령어를 가지고 어떤 device를 골라야하는지 확인하면 된다.
v4l2-ctl -d /dev/(HD Pro WebCam C920 아래에 출력된 device들을 차례로 넣어보면서 parameter 값이 나오는 device를 선택하면 된다.) --list-ctrls


# 파라미터 수정을 위해 .sh 파일을 작성한다. 
gedit fixcam.sh

# 빈 텍스트 창이 열리면, 아래의 내용을 입력한다. 
v4l2-ctl -d /dev/(본인의 device) --set-ctrl=focus_automatic_continuous=0

# .sh 파일을 적용시킨다. 
source ~/fixcam.sh

# 바뀐 변수를 확인한다. 
v4l2-ctl -d /dev/(본인의 device) --list-ctrls

# 앞 내용을 전부 진행하고 hunter_ws/src/realworld 경로에서 실행시킬 경우, cam이 나오지 않음
# 진행중이던 터미널을 모두 닫고 새로 시작하면 cam이 정상적으로 작동하는 것을 확인할 수 있음
cd hunter_ws/src/realworld
noetic
roslaunch usb_cam usb_cam-test.launch
```

</div>
</details>

<br>

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
sudo apt-get install -y ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-joint-state-publisher-gui ros-noetic-rqt-robot-steering ros-noetic-hector-gazebo-plugins ros-noetic-ackermann-steering-controller -y
sudo apt install -y ros-noetic-cob-perception-msgs ros-noetic-cob-object-detection-msgs # for yolo
sudo apt install ros-noetic-mavros-msgs -y # for gps
sudo apt install sharutils -y # for imu
sudo apt install -y ros-noetic-pcl-ros ros-noetic-rviz ros-noetic-tf2-geometry-msgs # ouster lidar
sudo apt install -y build-essential libeigen3-dev libjsoncpp-dev libcurl4-openssl-dev libspdlog-dev # ouster lidar
sudo apt install -y libv4l-dev
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



