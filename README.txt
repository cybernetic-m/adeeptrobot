---------------------------------------------------------------------------------------------------------------------------------------
Hi, I'm Massimo Romano, a Cybernetic Engineer! I created a Robot Manipulator URDF file (ROS/Gazebo enviroment) in collaboration with Adeept (https://www.adeept.com/) and I wrote this simple Genetic Algorithm for my thesis work at the University of Palermo. To understand everything about this project you must have a good experience with the ROS environment, with the Rviz viewer, with the Gazebo simulator and finally with the Genetic Algorithms !. I know it seems like a lot of work, but don't worry with the documents and sites that I will give you you will be able to easily understand everything !!! 

This project is protected by Creative Commons Attribution 4.0 International license (https://tldrlegal.com/license/creative-commons-attribution-4.0-international-(cc-by-4)), and it is modificable, redistributable and usable for commercial, but you must give me credit please! 
----------------------------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------------------------
!!!!!The project needs!!!!!

- Linux Ubuntu Focal 20.04 or later version (guide to installation: https://www.ubuntu-it.org/download)

- ROS Noetic (guide to installation: http://wiki.ros.org/noetic/Installation)

- How to initialize ROS (catkin_ws and Enviroment's Variables): http://wiki.ros.org/it/ROS/Tutorials/InstallingandConfiguringROSEnvironment

- Joint State Publisher (repository: https://github.com/ros/joint_state_publisher -> sudo apt install ros-noetic-joint-state-publisher sudo apt install ros-noetic-joint-state-publisher-gui)

- Gazebo (guide to installation: http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

- gazebo_ros (guide to installation: http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
-----------------------------------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------------------------------
!!!!!How to download this project!!!!!

1. Download this repository of GitHub

2. Copy "adeeptrobot" directory into /catkin_ws/src

3. Go to /catkin_ws and write the command "catkin_make"

4. Now you are able to use my project!
-----------------------------------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------------------------------
!!!!!How to visualize the Robot Model in Rviz!!!!!

1. Go to shell 

2. Go to /catkin_ws/src/adeeptrobot/launch

3. Write the command "roslaunch rviz.launch"

4. You visualize the Robot in Rviz and move it by the GUI of joint_state_publisher
------------------------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------------------------
!!!!!How to simulate the Robot Model in Gazebo!!!!!

1. Go to shell 

2. Go to /catkin_ws/src/adeeptrobot/launch

3. Write the command "roslaunch gazebo.launch"

4. Go to another shell

5. Go to /catkin_ws/src/adeeptrobot/src

6. Write the command: "rosrun adeeptrobot controller.py"

7. You visualize the flow of GA (Genetic Algorithm) and see that the Robot moves in Gazebo (tends to the vertical position)

8. You visualize the result of fitness calculated during the GA in file results.csv in /catkin_ws/src/adeeptrobot/src 
------------------------------------------------------------------------------------------------------------------------------------------
