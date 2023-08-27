# hunter2
Hunter2 Gazebo with GPS, IMU, Camera, Lidar (Ouster 64ch),
 Ubuntu 20.04 Ros1 Noetic



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

```bash
# Final
cd hunter_ws
source devel/setup.bash

roslaunch hunter2_gazebo hunter2_OS.launch
```
![Screenshot from 2023-07-09 21-00-57](https://github.com/Ethan-KoSeungHyun/hunter2/assets/113443261/73dbdeb3-6742-4689-96d3-0ef1e65d2981)

