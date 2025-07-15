# Safety Light Curtains

<a href="https://cmu-mfi.github.io/plc-safety/resources/results.mp4" target="_blank" style="cursor:zoom-in;"><video autoplay loop muted playsinline width="100%"><source src="https://cmu-mfi.github.io/plc-safety/resources/results.mp4" type="video/mp4">
</video>


```{contents}
```

***

## System Overview

![plc-system](../files/plc-system.png)

We have two downward-facing PLCs mounted on a 80/20 grid that is attached to the ceiling at a height of 3.35m from the ground. Each PLC has a Raspberry-Pi 4 onboard which runs all the code for designing and imaging the light curtains. The total dimensions of the testbed are 9.3x5.9 m<sup>2</sup> and all the communications are carried over ethernet to avoid latencies. The pose between each robot and PLC is calibrated with an eye-on-base calibration procedure using Apriltags.

![plc-annotate](../files/plc-annotate.png)

## Research Paper(s)

[Detailed Report](https://cmu-mfi.github.io/plc-safety/resources/report-v1.pdf)

As factories continue to evolve into collaborative spaces, with multiple robots working together with human supervisors in the loop, the problem of ensuring safety for all actors involved becomes critical. Presently, laser-based light curtain sensors are widely used in factories for safety monitoring. While these sensors have high accuracy standards, they are difficult to reconfigure and can only monitor a fixed user-defined region of space, and are typically expensive. We leverage a recently-developed controllable depth sensor, Programmable Light Curtains, for building an inexpensive and flexible real-time safety monitoring system. This system can project tight dynamic safety envelopes that enables fence-less human-robot collaboration, can scale to monitor multiple robots with few sensors, and by utilizing each sensor as a 3D depth sensor the system can also monitor the entire scene within its field of view. We deploy this system in a real testbed environment with four robot arms and demonstrate its capabilities as a powerful safety monitoring solution while being significantly cheaper and not compromising on accuracy.