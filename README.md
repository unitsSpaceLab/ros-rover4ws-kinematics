# ros-rover4ws-kinematics
A ROS package for the kinematics of a four wheel steering rover. This package use the python package you can find at the following [repository](https://github.com/unitsSpaceLab/rover4ws-kinematics). In order to use this package you need to install it as a python package. Please follow the instructions reported in its README.


## Installation
```
mkdir -p catkin_ws/srcˆ
cd catkin_ws/src
git clone https://github.com/unitsSpaceLab/ros-rover4ws-kinematics.git
cd ..
catkin_make
```


### (Recommended) Python 3.8 installation and virtual environment creation
```
sudo apt-get install python3.8*
```

Create now the virtual environment:

```
python3.8 -m venv p38Env
```

Test now if the environment has been successfully created:

```
source ~/p38Env/bin/activate
deactivate
```

### Installation of the requirements in the virtual environment
```
source ~/p38Env/bin/activate

mkdir -p ~/repositories/python
cd ~/repositories/python
git clone https://github.com/unitsSpaceLab/rover4ws-kinematics.git
cd rover4ws-kinematics
pip install -r requirements.txt
pip install -e .
pip install rospy
pip install rospkg
```


## Basic usage
``` 
source ~/catkin_ws/devel/setup.bash
source ~/p38Env/bin/activate
roslaunch robot4ws_kinematics kinematics.launch
```

## Description
* nodes:
    * rover_kinematic_node: This node contains the whole core of the kinematics of the four wheels steering rover
        * published topics:
            * /cmd_vel_motors

        * subscribed topics:
            * /cmd_vel
            * /kinematics_reset
            * /plot

        * services
            * /kinematic_mode


## Rviz visualization
This package allows also the visualization of some entities in RViz. In order to proceed with the visualization, the steps are:

1. open a terminal and execute the following:
    ```roslaunch robot4ws_kinematics kinematics.launch```

2. open a new terminal and execute the following>

    ```
    rostopic pub \plot std_msgs/Empty "{}" &
    rviz
    ```


## Requirements
* ROS installation
* python3.8
* python package kinematics [repository](https://github.com/unitsSpaceLab/rover4ws-kinematics)


## Notes

