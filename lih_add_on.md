### 安装以下未安装依赖：
```bash
sudo apt install ros-noetic-gazebo-ros-pkgs 
sudo apt install ros-noetic-pcl-conversions
sudo apt install ros-noetic-pcl-ros 
```

### 安装qgc:
https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

---
更改环境变量
### Add UAVros package path:
```bash
echo "source ~/iusc_outdoor/devel/setup.bash" >> ~/.bashrc
```

### Add px4 path:
#### if you use px4 version v1.13 or lower:
```bash
echo "
source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo
" >> ~/.bashrc
```

### 安装控制命令
```bash
echo "alias iuscland='rosrun iusc_land land_control.py'" >> ~/.bashrc
```

### USAGE
启动相关 ros 节点:
```
roslaunch iusc_land iusc_outdoor_land.launch
```

修改px4 参数:
```
COM_RCL_EXCEPT set to 7 for every uav
```

控制无人机起飞：
```
iuscland 1 takeoff
```