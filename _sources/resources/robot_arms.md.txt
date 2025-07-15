# Robot Arms

![Yaskawa GP4](../files/motoman_gp4.png)
*Four [Yaskawa GP4 robots](https://www.motoman.com/en-us/products/robots/industrial/assembly-handling/gp-series/gp4) at the testbed space*

```{contents}
```
***
<!-- 
Moveit Config

Skills

> lego assembly 
>   > simulation, assembly task, integration

> pallet pick and place

Other Utils 
-->

## Motion Stack and MoveIt 
<a href="https://github.com/cmu-mfi/motoman_ros1" class="inline-button"><i class="fab fa-github"></i>motoman_ros1</a>

ROS interface with the Yaskawa GP4 uses [`motoman_driver`](http://wiki.ros.org/motoman_driver) and ROS-Industrial stack to plan and execute trajectory. The figure below represents the stack. Details and tutorials to setup the driver and interface can be found [here](http://wiki.ros.org/motoman_driver).

![MotoROS](../files/motoros_stack.jpg)
*Source: [http://wiki.ros.org/motoman_driver](http://wiki.ros.org/motoman_driver)*

- **MotoROS Layer** is setup at the robot controller (YRC1000). [Details](https://wiki.ros.org/motoman_driver/Tutorials/indigo/InstallServer)
</br></br>

- **ROS-I Interface Layer** 
[motoman_gp4_support](https://github.com/ros-industrial/motoman/tree/433be15f6a27cb9a5715cc3887b1cd15ff33a8cb/motoman_gp4_support) \
Open-source community has a support package for many industrial robots, including GP4.
</br></br>

- **MoveIt Layer** [motoman_gp4_moveit_config](https://github.com/cmu-mfi/motoman_ros1/tree/f6bb0a967dcad931a87ea3ccb57094d18ffced40/motoman_gp4_moveit_config) \
We developed moveit configuration package for GP4 robots which is used for trajectory planning.

***
## Skills

In v1.0 the robots are skilled to pick and place pallets of LEGO® baseplate and bricks, and manipulate bricks to assemble/disassemble structures.

### LEGO® Manipulation 
<a href="https://github.com/cmu-mfi/gp4-lego-assembly" class="inline-button"><i class="fab fa-github"></i>gp4-lego-assembly</a>
<!-- TODO: insert GIF of LEGO assemble/disassemble -->

#### > Simulation
![LEGO Simulation](../files/gp4_sim.gif)



#### > Assembly/Disassembly Task
<!-- TODO: diagram of service call and motion planning -->

#### > Integration with MES

### Pallet Pick and Place 
<a href="https://github.com/cmu-mfi/pallet_pick_and_place" class="inline-button"><i class="fab fa-github"></i>pallet_pick_and_place</a>

***

## Other Utilities
<a href="https://github.com/cmu-mfi/testbed_camera_utils" class="inline-button"><i class="fab fa-github"></i>testbed_camera_utils</a> <a href="https://github.com/cmu-mfi/testbed_lin_actuator" class="inline-button"><i class="fab fa-github"></i>testbed_lin_actuator</a>