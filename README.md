# panda_grasp

This project depends on franka_description (part of [franka_ros](https://github.com/frankaemika/franka_ros)) and orocos KDL.
If you have installed them with ros, check if the version for the visuals in protos/panda.proto is the same as yours (for the visuals) and check if the Makefiles in controllers/kdl_controller and controllers/kdl_controller2 link to the right path. The current ROS version is noetic.  
If you have not installed these dependencies, it's up to you ton install them and link the files accordingly.
