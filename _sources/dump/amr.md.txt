# Autonomous Mobile Robots

<a href="https://github.com/cmu-mfi/amr_testbed_v1.git" class="inline-button"><i class="fab fa-github"></i>amr_testbed_v1</a>

![AMR](https://mrsd-project.herokuapp.com/images/dock_undock.gif)

```{contents}
```

***

## System Architecture

![AMR System Architecture](https://mrsd-project.herokuapp.com/images/robot_bringup/High%20Level.png)

Below are relevant packages of the system:

- **Global Planner** (`ddg_multi_robot_planner`): Conflict-Based-Search (CBS) based ROS2 planner. It uses CBSH2-RTC mapf planner developed by Prof. Jiaoyang Li
- **Mission Control** (`robot_mission_control`): It launches a node to handle multiple AMRs and their higher-level commands
- **Motion Servers** (`robot_motion_server`): ROS2 action servers and client onboard robot to give navigation commands. There is a node for both navigation and docking/undocking
- **Low Level Packages** (`neo_*`): These packages are originally provided by the manufacturer, [Neobotix](https://neobotix-docs.de/ros/), and handle local navigation, localization and motion.

## Simulation Environment

*<TODO: insert simulation video/gif>*

*<TODO: describe packages to use for the simulation>*

## Integration with MES

- The [amr_fleet_offboard_infra_backend](https://github.com/cmu-mfi/amr_mes_integration/xxx) package receives the web request for the AMRs, and then communicates it over the ROS2 network.
- The [offboard_comms](https://github.com/cmu-mfi/amr_mes_integration/xxx) package handles the translated web request and sends it over to the mission control
- The [amr_fleet_offboard_infra_frontend](https://github.com/cmu-mfi/amr_mes_integration/xxx) is an alternative to MES sending web request for a task for the AMRs.

## Other Resources

<iframe width="704" height="396" src="https://www.youtube.com/embed/ae9kbRRNe_k" title="MRSD Team H Capstone Project - Autonomous Material Handling for Warehouses using a fleet of AMRs" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

[https://mrsd-project.herokuapp.com/](https://mrsd-project.herokuapp.com/)
[https://mrsdprojects.ri.cmu.edu/2023teamh/](https://mrsdprojects.ri.cmu.edu/2023teamh/)
