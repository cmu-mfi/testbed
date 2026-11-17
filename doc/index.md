# Robotics Workcell Cluster

```{button-link} https://cmu-mfi.github.io/
:color: primary
:shadow:
cmu-mfi.github.io
```

**Vision: Automated high-mix manufacturing accomplished through generative multi-robot-arm cooperative manipulation**

![RWC Quad](/files/rwc.png)

Conventional automation relies on fixed, pre-programmed systems—PLC-controlled conveyors, sensors, and robotics—to perform repetitive tasks. In high-mix manufacturing, complex tasks still depend heavily on skilled labor, making it difficult to scale without additional hiring and training.

As personalization, agile supply chains, and rapidly changing SKUs reshape manufacturing, automation must evolve to handle complex tasks and adapt to new processes. This is where the top-right quadrant of the 2×2 focuses: intelligent, connected robotic systems.

<hr>

**What is Robotics Workcell Cluster?**

The workcells within the Robotics Workcell Cluster (RWC) provide a platform for researchers to develop and integrate robot skills that enable teams of autonomous agents to accomplish collaborative tasks intelligently in dynamic environments. The projects focuses on building safe, scalable, and efficient robotic systems that can measure uncertainty and work across different robot brands. The tools should transfer to new tasks, support real manufacturing needs, and be shared as open-source utilities for others to use.

**Technical Themes**
* Coordinated robotic skill execution <!-- skill graph? -->
* Fault detection and automatic recovery <!-- prompt to product -->
* Collaborative multi-robot systems <!-- Philip's work -->
<!-- * Optimization factors (e.g., assembly time, robot-arm utilization, other)  -->
* Spatially aware virtualization for mixed reality <!-- anthony rowe's work -->
* Fine-grained manipulation with multi-modal sensing (audio, tactile, eye-in-hand, force-torque) <!-- vibrotactile, ssd insertion, lego -->
* Context-driven learning with foundational models <!-- GTA -->
* Foundation Planning Model <!-- RoboFin3d? GTA? -->
* Foundation Perception Models <!-- GTA -->
* Grounded Skill Controllers and Perception <!-- GTA, Skill graph -->
* Safe human worker collaboration <!-- PLC - Karnik -->
* Natural language programming <!-- GTA -->

<!-- insert figure -->

<hr>

## Site Map

```{toctree}
:maxdepth: 2

publications.md
```

```{toctree}
:maxdepth: 2
:caption: Workcells

workcells/assembly-disassembly.md
workcells/grinding-polishing.md
workcells/welding.md
```
```{toctree}
:maxdepth: 2
:caption: Guides

guides/ros1_robot_interface.md
guides/ros2_robot_interface.md
guides/autonomous_mobile_robots.md
guides/robot_welding_le_classmate.md
```