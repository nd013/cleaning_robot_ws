# 1. install dependencies
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
Once you have successfully installed the necessary tools, it's time to create a new workspace called 'carto_ws' within your existing 'catkin_ws'.
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
In case you encounter any difficulties or run into an issue, take the following steps to address the situation:
```
sudo nano ~/catkin_ws/carto_ws/src/cartographer/package.xml
```
To deactivate or disable the functionality of line number 46, you should comment it out.
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

### Sources:
(http://wiki.ros.org/catkin/Tutorials/workspace_overlaying) First source the carto_ws, Now we need to overlay the catkin_ws so delete the devel and build folder from catkin_ws and then redo catkin_make then source your main catkin_ws
```
source ~/catkin_ws/devel/setup.bash
```
(https://google-cartographer-ros.readthedocs.io/en/latest/)

# 2. Instructions
## 2.1 Download the compiled package
```
mkdir -p cleaning_robot_ws/src && cd clean_robot_ws/src
git clone https://github.com/nd013/cleaning_robot_ws.git
cd ..
catkin_make
```
## 2.2 Self-cleaning
Here, You need to make updates to the launch files 
```
cd cleaning_robot_ws
source devel/setup.bash
roslaunch serve_clean clean_work.launch
roslaunch serve_clean clean_work.launch map_file:="/home/xxx/cleaning_robot_ws/yourmap.yaml"
```

## 2.3 Sources 
(https://www.sciencedirect.com/science/article/pii/S2352664516300050)

# 3. Implimenting on Hardware
## 3.1 Connecting to Robot
The first thing you need to do is to access your robot through SSH.
This connects a specific terminal on your PC to the robot so that you can run commands in the robot's terminal.
OR Else you can use VNC server.
refer "What are the ways to establish a remote connection with Jetson Nano?"

## 3.2 Setting Up Robot

If you are using SSH. you have to run this on jetson nano terminal. 
```
roslaunch serve_firmware bringup.launch
```
(If you are utilizing a VNC server, please ensure that you execute all these commands within the terminal.)
After successfully completing the robot and sensor initialization process, including the bringup, you will notice the LiDAR on top of the robot begins rotating. Next, you should establish the server connection from your personal computer (PC) in order to assume control and send commands to the robot. To achieve this, execute the following command within your PC's terminal.
```
roslaunch serve_firmware server_bringup.launch
```
Congratulations! Your robot is now fully prepared and ready for action!

## 3.3 Visualizing Sensor Data
Once you have completed the setup, you can now visualize data from different sensors on your robot, such as LiDAR and Camera. We will utilize RViz for data visualization. To launch RViz, execute the following command in your PC's terminal.
```
roslaunch serve_slam view_sensors.launch
```

## 3.4 Creating a map of the surrounding environment

To generate a map of the surrounding area, begin by running the "bringup.launch" file on the robot's terminal.
```
roslaunch serve_firmware server_bringup.launch
```
After running "bringup.launch" on the robot's terminal, proceed by executing "server_bringup.launch" on your PC. This will establish the necessary server connection to facilitate map generation and control of the robot.
```
roslaunch serve_firmware server_bringup.launch
```
Next, open another terminal and launch the "serve_slam.launch" file using the following command:
```
roslaunch serve_slam serve_slam.launch
```
You can now initiate the teleoperation node in a separate terminal and begin maneuvering the robot to create a comprehensive map of its surroundings.
```
rosrun serve_control serve_teleop.py
```
To save the generated map within the maps folder, execute the following command in the terminal. Feel free to replace "my_map" with your desired name for the map:
```
rosrun map_server map_saver -f maps/my_map
```
This command will save the map with the specified name ("my_map") in the maps folder.

Finally, you can launch the navigation and clean_work.launch files to enable the robot to perform navigation and cleaning tasks, respectively.
