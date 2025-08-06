# FANUC ROS1 Interface

The <a href="https://github.com/cmu-mfi/fanuc_ros1" class="inline-button"><i class="fab fa-github"></i>fanuc_ros1</a> repository is a ROS1 interface for FANUC robot arms, specifically designed to work with the R-30iB+ controller. It provides a framework for motion planning and execution using MoveIt, along with various skills for industrial tasks.

```{contents}
```

## Motion Stack and MoveIt 

ROS interface uses the ROS-Industrial stack to plan and execute trajectory. The figure below represents the stack. Details and tutorials to setup the driver and interface can be found [here](http://wiki.ros.org/fanuc/Tutorials).

![MotoROS](../files/ros_industrial_architecture.png)
*Source: [http://wiki.ros.org/Industrial](http://wiki.ros.org/Industrial)*

- **ROS-I Controller Layer** is setup at the robot controller (R-30iB+). [Details](https://wiki.ros.org/fanuc/Tutorials/hydro/Installation) \

</br></br>

- **ROS-I Interface Layer** \
[fanuc_lrmate200id_support](https://github.com/cmu-mfi/fanuc_ros1/tree/main/fanuc_lrmate200id_support) \
[Open-source community](http://wiki.ros.org/fanuc) has a support package for many industrial robots, including LR Mate 200iD.
</br></br>

- **MoveIt Layer** \
[fanuc_lrmate200id7l_moveit_config](https://github.com/cmu-mfi/fanuc_ros1/tree/main/fanuc_lrmate200id7l_moveit_config) \
We developed moveit configuration package for LR Mate 200iD robots which is used for trajectory planning.
</br></br>

- **Application Layer** \
[fc_tasks](https://github.com/cmu-mfi/fanuc_ros1/tree/main/fc_tasks) \
We developed a set of services and actions to perform various industrial tasks using the robot arm. These allow to develop custom application using the robot arm without worrying about the underlying motion planning and execution.

<hr>

## Software Architecture

The fanuc_ros1 package is an interface layer that enables planning and execution via MoveIt, allows for peripheral control and launches the RViz GUI. In the standard ROS-I architecture, it would fit as shown below:

![SoftwareArchitecture](../files/layers.jpg)


## FC Interface Class

Initializes the MoveIt! MoveGroup interface and sets up all relevant ROS interfaces including action servers, service servers, and topic subscribers for controlling a robotic manipulator.

Interface Name = move_group \
Group Name = manipulator \
Default Planning Pipeline = pilz_industrial_motion_planner

> Note: All below ROS elements will be prefixed with the chosen namespace. (Ex: /sim1/fc_get_pose)

### ROS Services

- getPose - <i>'/fc_get_pose'</i>
- setPose - <i>'/fc_set_pose'</i>
- setJoints - <i>'/fc_set_joints'</i> (Uses BiTRRT)
- executeTrajectory - <i>'/fc_execute_trajectory'</i>
- stopTrajectory - <i>'/fc_stop_trajectory' </i>
- executeCartesianTrajectory - <i>'/fc_execute_cartesian_trajectory'</i>
- executeCartesianTrajectoryAsync - <i>'/fc_execute_cartesian_trajectory_async'</i>

### ROS Actions

- GoToPose - <i>'/fc_go_to_pose'</i>
- GoToPose (Async) - <i>'/fc_go_to_pose_async'</i>
- GoToJoints - <i>'/fc_go_to_joints'</i> (Uses BiTRRT)
- ExecuteCartesianTrajectory - <i>'/fc_execute_cartesian_trajectory_action'</i>

### ROS Topic Subscribers

- CheckMoving - <i>'/check_moving'</i>
- TrajectoryStatus - <i>'/execute_trajectory/status'</i>
- JointStates - <i>'/joint_states'</i>

### ROS Topic Publishers

- End Effector Pose - <i>'/tool0_pose'</i>

> Note: All methods called by these elements are detailed in the Doxygen formatting style.

## Dependencies

- fc_launch
- fc_msgs
- fanuc_lrmate200id_support
- fanuc_lrmate200id7l_moveit_config
- fanuc

## Fanuc IO Interface

This package also enables Fanuc I/O control using [comet_rpc](https://github.com/gavanderhoorn/comet_rpc). Using the io.launch file, a Fanuc_IO ROS node is launched with the following functionality -

### ROS Services

- setIOValue - <i>/set_io_value</i>
- readIOValue - <i>/read_io_value</i>

Note: These services use the SetIO.srv and ReadIO.srv messages from fc_msgs. Please use rosservice info to get all input details.

### ROS Publishers

- <i>/io_states_DOUT</i> - Publishes all Digital Out I/O states
- <i>/io_states_DIN</i> - Publishes all Digital In I/O states
- <i>/io_states_AOUT</i> - Publishes all Analog Out I/O states
- <i>/io_states_AIN</i> - Publishes all Analog In I/O states


## Tutorial 1 - Testing functionalities and using scripts to test actions and services

### Testing Functionalities
1. Use the fanuc tutorials to ensure that connection with the robot is established and the required drivers are running. 
2. Ensure the moveit_config works standalone, before testing fc_launch. Use the following commands: 

```shell
roslaunch fanuc_lrmate200id7l_moveit_config moveit_planning_execution.launch sim:=true 
roslaunch fanuc_lrmate200id7l_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=<robot_ip>
```
3. Check fc_tasks runs in simulation by keeping sim:=true with no namespace. 

```shell
roslaunch fc_launch moveit.launch sim:=true namespace:=''
```
4. Test with a namespace.

```shell
roslaunch fc_launch moveit.launch sim:=true namespace:='sim1'
```


5. Test on a real robot. 

```{warning}
* Ensure the robot is in a safe state and the environment is clear of obstacles before executing commands on a real robot.
* Keep e-stop button ready to stop the robot in case of any unexpected behavior.
```

```shell
roslaunch fc_launch moveit.launch sim:=false namespace:='real'
```
6. Check 'plan and execute' works in RViz by setting a 'random valid' goal. 
7. Check all ROS services and actions work. An example is shown below:

```shell
rosservice call /sim1/fc_get_pose sim1/base_link 
```

```{note}
This tutorial can be adapted to non-Fanuc robot arms. However, problems may occur due to version difference of support and moveit configs. It is recommended to use this interface repository as a reference and build the new package from the beginning. 
```
This repository comes with example scripts to test actions and services in 'fc_tasks/scripts'. These are outlined below :


### SetPose Service Test
The setPose_test.py script contains an example pose to test the /fc_set_pose service. Modify this as required to test. It's a good idea to execute the same pose multiple times to ensure that all frames are calibrated correctly. 
```shell
python3 ~/ros1_ws/src/fc_tasks/scripts/setPose_test.py
```
![SetPose](../files/setPoseService.gif)

### ExecuteCartesianTrajectory Service Test
The cartTraj_test.py script contains a trajectory with 2 waypoints to test the /fc_execute_cartesian_trajectory service. Modify this as required to test. 
```shell
python3 ~/ros1_ws/src/fc_tasks/scripts/cartTraj_test.py
```
![CartTraj](../files/cartTrajService.gif)

### GoToPose Action Test 
The goToPoseAction_test.py script contains an example pose to test the /fc_go_to_pose action server. 
```shell
python3 ~/ros1_ws/src/fc_tasks/scripts/goToPoseAction_test.py
```
Same output as SetPose Service Test.
### ExecuteCartesianTrajectory Action Test 
The execTrajAction_test.py contains a trajectory with 2 waypoints to test the /fc_execute_cartesian_trajectory_action action server. 
```shell
python3 ~/ros1_ws/src/fc_tasks/scripts/execTrajAction_test.py
```
Same output as ExecuteCartesianTrajectory Service Test.

<hr>

## Tutorial 2 - Configuring the Interface for another Fanuc Robot Arm

<a href="https://github.com/cmu-mfi/fanuc_ros1" class="inline-button"><i class="fab fa-github"></i>fanuc_ros1</a> can be used for any FANUC robot arm with a R-30iB+ controller. This tutorial explains how to configure the interface for a different FANUC robot arm, such as the LR Mate 200iD/7L.

```{note}
This tutorial assumes that the new package is built in a docker container with the same image as fanuc_ros1. If using a different environment, all dependencies must be satisfied manually as Noetic is now EOL. Follow [installations](https://github.com/cmu-mfi/fanuc_ros1?tab=readme-ov-file#installation)
```

### Making changes

1. Find the required support package and moveit config for the robot arm at [fanuc](https://github.com/ros-industrial/fanuc). 
2. Change the package.xml dependencies of <i>fc_tasks</i> and <i>fc_launch</i> to match the new moveit config package (fanuc_lrmate200id7l_moveit_config).
3. Use the urdf file from the downloaded support package to find DH parameters of the new robot arm. Update the .json file in '/fc_tasks/config' with the new values. 
4. In 'fc_launch/launch/moveit.launch', change the 'fanuc_lrmate200id7l_moveit_config' to the new moveit config package and update the 'robot_ip'.
5. Inside the moveit_config package, add the namespaced controllers to the 'config/controllers.yaml' file. Refer 'fanuc_lrmate200id7l_moveit_config'. 
6. Go through the moveit_config launch and config files to see if any file are missing/have different structures due to software updates. 

<!-- **TODO: Add a PR to the fanuc_ros1 repository by doing the above changes for a new robot arm.** -->

```{note}
Most common differences are found in 'move_group.launch', 'moveit_planning_execution.launch' and 'trajectory_execution.launch.xml'. Check all namespaces match this repository
```

### Testing changes

Use the testing steps described in Tutorial1 to test all interface functionalities.

