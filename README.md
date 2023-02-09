# machine_tool_scan
machine_tool_scan


### 1. Requirements
System: Ubuntu 18.04, Ubuntu 20.04   

ROS: Melodic, Noetic    
 
Moveit   

Gazebo   

### 2. Other Dependencies
For some dependencies were missed for the project, please check the following instructions.
```
sudo apt-get install ros-melodic-ros-controllers
```

**NOTE:**There might be more dependencies needed due to vairous reasons. If you found any, please let me know.(archer7wang@outlook.com)

### 3. Usage
Like ordinary ros projects, the simluation needs a clean workspace.
```
mkdir -p machine_tool_scan/src
cd src
git clone https://github.com/wangarcher/machine_tool_scan.git
cd ..
catkin_make
source devel/setup.bash
``` 

##### 1. Initialization
To initialize the machine_tool_scan 
```
roslaunch machine_tool_bringup test_new.launch
```

##### 2. Control
To run the cartesian pid control
```
roslaunch machine_tool_control machine_tool_scan.launch
```

##### INPUTS:
1. a_side_length
2. b_side_length
3. corner_radius
4. scan_velocity