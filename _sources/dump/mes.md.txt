# Manufacturing Executive System

<a href="https://github.com/cmu-mfi/xx" class="inline-button"><i class="fab fa-github"></i>ICLL_MES</a>

*<TODO: insert image of executive timeline and order status>*

```{contents}
```

The executive uses "domain" .json files for the resource and job setup. Once the executive gets an order, it uses the scheduler to plan the tasks to respective resources.

Below are important components of the executive:

- [**Executable Scheduler**](https://github.com/cmu-mfi/xx) \
Executable file for intel linux platforms.
- [**MES Browser Portal**](https://github.com/cmu-mfi/xx) \
Browser UI to input order, monitor tasks timeline, and order status.
- [**Domain Problems**](https://github.com/cmu-mfi/xx) \
Domain files for testbed v1.0 each specified with specific constraints and resources.

***

## Assembly/Disassembly Order Workflow

*<TODO: insert a table of tasks to be done for an order>*

The above order can be modified by making appropiate changes to the domain .json files.

## Virtual Resources

<a href="https://github.com/cmu-mfi/xx" class="inline-button"><i class="fab fa-github"></i>virtual_resources</a>

Other than robot arms, and autonomous mobile robots, there are some tasks which requires humans (like Depot station to load/unload LEGO kits) and some that are virtual (like Inspection where AMRs just pause for few seconds). They both have a web server to receive tasks and send back completion message.

### Human operated Resources

Depot has a human to load/unload kit. Once MES assigns a task to the Depot, the portal queues the task for human to perform. Once complete, the operator can click **complete** and MES moves on to the next task for the respective order.

*<TODO: insert image of depot browser UI and human loading a kit on AMR>*

### No task Resources

Some task in the v1.0 of the testbed, have not been assigned a clear objective, like Inspection. The inspection web server receives the tasks, wait for a few seconds, and sends back the completion message. 
Such servers are placeholder to develop inspection tasks in future versions.