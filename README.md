# 1 install dependencies
## 1.1 Ubuntu 18.04+melodic
```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-joint-state-publisher ros-melodic-robot-state-publisher ros-melodic-slam-gmapping ros-melodic-dwa-local-planner ros-melodic-joint-state-publisher-gui ros-melodic-cartographer-ros ros-melodic-cartographer-rviz
```
## 1.2 Ubuntu 20.04+Noetic
```
sudo apt-get install ros-noetic-map-server ros-noetic-move-base ros-noetic-navigation ros-noetic-dwa-local-planner ros-noetic-ira-laser-tools ros-noetic-teleop-twist-keyboard
```

### Install Google-Cartographer
```
sudo apt-get update
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
```
After the tools are installed, create a new 'carto_ws' workspace inside your existing ‘catkin_ws’.
```
mkdir ~/catkin_ws/carto_ws
cd ~/catkin_ws/carto_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
```
```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```
if you face a problem, do the following:
```
sudo nano ~/catkin_ws/carto_ws/src/cartographer/package.xml
```
go to line number 46 and comment it out
```
<!-- <depend>libabsl-dev</depend> -->
```
then cd into src and clone the following repositories
```
git clone -b melodic-devel https://github.com/ros-perception/perception_pcl.git
git clone https://github.com/ros-perception/pcl_msgs
git clone -b noetic-devel https://github.com/jsk-ros-pkg/geometry2_python3.git
```
and then go back to carto_ws and run
```
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```
```
src/cartographer/scripts/install_abseil.sh
```
Run this from carto_ws:
```
catkin_make_isolated --install --use-ninja -j4 -l4
```

### Source both the workspace in the given order:
(http://wiki.ros.org/catkin/Tutorials/workspace_overlaying) First source the carto_ws, Now we need to overlay the catkin_ws so delete the devel and build folder from catkin_ws and then redo catkin_make then source your main catkin_ws
```
source ~/catkin_ws/devel/setup.bash
```
(https://google-cartographer-ros.readthedocs.io/en/latest/)

# 2 Instructions
## 2.1 Download the compiled package
```
mkdir -p cleaning_robot_ws/src && cd clean_robot_ws/src
git clone 
cd ..
catkin_make
```

















