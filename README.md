# hunter_ws


```bash
sudo apt-get install ros-noetic-ros-control -y
sudo apt-get install ros-noetic-ros-controllers -y
sudo apt-get install ros-noetic-gazebo-ros -y
sudo apt-get install ros-noetic-gazebo-ros-control -y
sudo apt-get install ros-noetic-joint-state-publisher-gui -y
sudo apt-get install ros-noetic-rqt-robot-steering -y
sudo apt-get install ros-noetic-hector-gazebo-plugins -y
sudo apt install ros-noetic-ackermann-steering-controller -y
sudo apt install ros-noetic-cob-perception-msgs -y
pip install --upgrade python-dateutil
```

```bash
mkdir hunter_ws
cd hunter_ws
mkdir src
cd src

git clone https://github.com/Ethan-KoSeungHyun/hunter2.git
```
```bash
cd ~/hunter_ws
source /opt/ros/noetic/setup.bash
pip install -r ./src/yolov5_ros/src/yolov5/requirements.txt
chmod +x ./src/yolov5_ros/src/detect.py
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```


```bash
# Rviz
cd hunter_ws
source devel/setup.bash

roslaunch hunter2_base display_xacro.launch
```

![Screenshot from 2023-08-22 22-01-09](https://github.com/AIV-Autonomous-Intelligent-Vehicles/hunter_ws/assets/113443261/0b82c9bb-e685-4be6-be17-4f54bb118dfc)



```bash
# Rviz
cd hunter_ws
source devel/setup.bash

roslaunch hunter2_base display_xacro.launch
```
![RVIZ](https://github.com/Ethan-KoSeungHyun/hunter2/assets/113443261/161d68b4-372e-4986-8326-bd5203a5c996)


```bash
# Gazebo
cd hunter_ws
source devel/setup.bash

roslaunch hunter2_gazebo hunter2_empty_world.launch
```
![GAZEBO](https://github.com/Ethan-KoSeungHyun/hunter2/assets/113443261/90bfab3a-ab56-48b4-b3bd-abe4cb11cb60)


