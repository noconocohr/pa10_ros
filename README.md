# pa10_ros

forked from akrouch-robi in bitbucket

## How to Use

1. Install dependencies

```sh
# MoveIt! packages and controller manager for ROS Kinetic
sudo apt install ros-kinetic-moveit* ros-kinetic-controller-manager*

# Industrial packages
sudo apt-get install ros-kinetic-industrial-core ros-kinetic-industrial-robot-client ros-kinetic-industrial-robot-simulator
```

2. build source

```sh
cd ~/<Your Workspace>/
catkin_make
catkin build #if you use catkin_tools
```

Please see pa10_tutor repo [here](https://github.com/noconocohr/pa10_tutor).