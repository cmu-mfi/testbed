# FANUC ROS1 Interface
<a href="https://github.com/cmu-mfi/fanuc_ros1" class="inline-button"><i class="fab fa-github"></i>fanuc_ros1</a>

The fanuc_ros1 repository is a ROS1 interface for FANUC robot arms, specifically designed to work with the R-30iB+ controller. It provides a framework for motion planning and execution using MoveIt, along with various skills for industrial tasks.

## Software Architecture

The fanuc_ros1 package is an interface layer that enables planning and execution via MoveIt, allows for peripheral control and launches the RViz GUI. In the standard ROS-I architecture, it would fit as shown below. Details and tutorials to setup the driver and interface can be found [here](http://wiki.ros.org/fanuc/Tutorials).

![FANUC_ROS](../../files/ros_industrial_architecture.png)
_Source: [http://wiki.ros.org/Industrial](http://wiki.ros.org/Industrial)_

- **ROS-I Controller Layer** is setup at the robot controller (R-30iB+). [Details](https://wiki.ros.org/fanuc/Tutorials/hydro/Installation)
<br></br>

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

## ROS Packages

- fc_launch
- fc_msgs
- fanuc_lrmate200id_support
- fanuc_lrmate200id7l_moveit_config
- fanuc
- **fc_tasks**

```{note}
`fc_tasks` is the core package which either uses other listed packages or packages, like fc_launch, use fc_tasks to launch the interface.
```

<msg/srv/action type> - *<name of topic/service/action>*

**TODO - consistent formatting for all elements below**

### ROS Services

- getPose - <i>'/fc_get_pose'</i>
- setPose - <i>'/fc_set_pose'</i>
- setJoints - <i>'/fc_set_joints'</i> (Uses BiTRRT)
- executeTrajectory - <i>'/fc_execute_trajectory'</i>
- stopTrajectory - <i>'/fc_stop_trajectory' </i>
- executeCartesianTrajectory - <i>'/fc_execute_cartesian_trajectory'</i>
- executeCartesianTrajectoryAsync - <i>'/fc_execute_cartesian_trajectory_async'</i>
- setIOValue - <i>/set_io_value</i>
- readIOValue - <i>/read_io_value</i>

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
- <i>/io_states_DOUT</i> - Publishes all Digital Out I/O states
- <i>/io_states_DIN</i> - Publishes all Digital In I/O states
- <i>/io_states_AOUT</i> - Publishes all Analog Out I/O states
- <i>/io_states_AIN</i> - Publishes all Analog In I/O states

```{note}
Note: All methods called by these elements are detailed in the Doxygen formatting style.
```

```{note}
This package also enables Fanuc I/O control using [comet_rpc](https://github.com/gavanderhoorn/comet_rpc). Using the io.launch file, a Fanuc_IO ROS node is launched with the following functionality 
```

## Tutorials

```{toctree}
:maxdepth: 2
tutorials.md
```