# IRL Robot Drivers

The irl_robot_drivers package contains drivers for interfacing with various robots in CoppeliaSim via a generic controller. This controller enables both publishing states and reading control commands from ROS topics and interfaces with the devices in [IntPrim Framework ROS](https://github.com/ir-lab/intprim_framework_ros).

For more information on how to use this package, please consult: [IntPrim Framework ROS](https://github.com/ir-lab/intprim_framework_ros).

## Installation

Simply clone this package into a directory and run catkin_make.

Note: CoppeliaSim must first be installed on the system and the environment variable "COPPELIA_SIM_PATH" must be set to the root path of the simulator. For example:

```
export COPPELIA_SIM_PATH="/root/Data/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04"
```

See [installation instructions](https://github.com/ir-lab/intprim_framework_ros/blob/master/docs/tutorials/sub_tutorials/install_locally.md#installing-irl_robot_drivers) for further details.

## Contact
[Joseph Campbell](https://sites.google.com/asu.edu/jcampbell/), <jacampb1@asu.edu>
Arizona State University, Interactive Robotics Lab

Michael Drolet, <mdrolet@asu.edu>
Arizona State University, Interactive Robotics Lab

[Heni Ben Amor](http://henibenamor.weebly.com/), <hbenamor@asu.edu>
Arizona State University, Interactive Robotics Lab