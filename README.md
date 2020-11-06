# double_panda

This repository contains a simulation of a double armed robot with a gripping demonstration with ROS, MoveIt and WeBots. Both arms are franka emika's Panda.
Special thanks to [@erdalpekel](https://github.com/erdalpekel/panda_simulation) whose urdf I reused.

## Demo

https://youtu.be/XInSYrOIZGQ

## How to launch the project

```bash
cd <catkin_ws>/src
git clone https://github.com/ddonatien/double_panda.git
cd ..
catkin_make ## or catkin build
```

Open 3 terminals (from your catkin_ws) and follow the instructions in order :

### Terminal 1
```bash
export WEBOTS_HOME=/usr/local/webots ## or your usual WEBOTS_HOME
source devel/setup.bash
roslaunch double_panda panda_world.launch
```

### Terminal 2
```bash
source devel/setup.bash
roslaunch double_panda move_group.launch
```

### Terminal 3
```bash
source devel/setup.bash
roslaunch double_panda moveit_rviz.launch
```

You can also control the robot with Rviz using
```bash
source devel/setup.bash
roslaunch double_panda moveit_rviz.launch
```
