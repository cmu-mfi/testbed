# Autonomous Mobile Robots

```{contents}
```

## Hardware Setup

The AMR system is based on the [Neobotix MP-400 Platform](https://www.neobotix-robots.com/products/mobile-robots/mobile-robot-mp-400) which was also built upon by the [Dock Dock Go MRSD team at CMU](https://mrsdprojects.ri.cmu.edu/2023teamh/media/). The AMR platform includes the Neobotix base, two Intel Realsense cameras, one facing the front and one the back. Through a top module that holds build plates for additive manufacturing, there are also corner pieces that protect the AMR from any collisions through the top module. It also includes an adjustable tablet holder connected to the top module. *Insert photo of AMR here*,  *Insert Diagram of Robot*

The platform also includes its own integrated router that allows ros2 isolation from the bigger CMU network. This provides security for the robot, keeping it a completely closed system that does not need to connect to other networks. 

*Include Pictures of the robot*


## Software Architecture

*Insert AMR Control Stack Image Here*

The AMR system is built upon the Neobotix platform and all of its included packages. The waypoint server and predocking server I have made communicate with neobotix packages, nav2, as well as the docking with fiducial markers package that has been custom made [here](https://github.com/cmu-mfi/amr_docking_fiducial/tree/main). The front end provides easy communication with all the components of the AMR to allow for easy docking and waypoint saving


### Description of topics and services

**Topics:**

**/waypoint_amount** - Publishes the number of waypoints

**/robot_state** - Publishes the current state of the robot

**Services:**

**/waypoint_maker** - Waypoint Service that takes SetFlag message and controls the control flow from flags and modes given

**/pre_docker_offset** - Takes input from the Waypoint server and communicates with docking_with_fiducial package to save the aruco marker offset

**/pre_docker_docking** - Called from waypoint server and communicates with docking_with_fiducial package to dock to a detected aruco marker

### Communication Flow

Through the frontend application, you communicate with the waypoint server directly to send commands to the control stack, which communicates to the rest of the packages included (See the control flow graph above)

## Tutorials

```{toctree}
:maxdepth: 2

tutorials.md
```