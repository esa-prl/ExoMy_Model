# rover_gazebo_plugins 

## Overview

This package contains all custom Gazebo plugins 

**Keywords:** gazebo, plugins

### License

The source code is released under a [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.en.html).

**Author: Maximilian Ehrhardt<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Maximilian Ehrhardt, maximilian.ehrhardt@esa.int**

The rover_gazebo_plugins package has been tested under [ROS2] Foxy Fitzroy and Ubuntu 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation


### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Gazebo](http://gazebosim.org/) (simulation software)
- [gazebo-ros-pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/) (connection between ROS2 and Gazebo)
- [rover_msgs](https://github.com/esa-prl/rover_msgs)(message definitions)



#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/rover_gazebo_plugins.git
	cd ../
	colcon build


### Unit Tests
## Usage
The plugins defined in this package can be used in a `*.gazebo` file, that describes the setup for a simulation in Gazebo. 
The plugins are the connection between ROS2 and Gazebo.

## Plugins
### rover_gazebo_joint_plugin
This plugin forwards joint commands from ROS2 to Gazebo.  

First a PID controller for every deployment, drive and steering joint in the model definition (.xacro or .sdf) is created. 
The joints are classified by their names, containing one of the keywords DEP, DRV or STR.
The values of the PID controllers can be defined by plugin parameters.

The plugin subscribes to the `joint_cmds` topic and sets the target values of the PID controllers accordingly.

#### Parameters
* **update_rate** {float}:
* **pid_deploy** {float, float, float}: P, I and D value for the controllers of the deployment joints
* **pid_steer** {float, float, float}: P, I and D value for the controllers of the steering joints
* **pid_drive** {float, float, float}: P, I and D value for the controllers of the driving joints

## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.


[ROS2]: http://www.ros.org
[rover_msgs]: https://github.com/esa-prl/rover_msgs
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
