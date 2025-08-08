## Tutorial 1 : Qt Application

To launch the application from inside the robot, you run 

``` 
ros2 launch launch_dir launch.py
```

The Qt application should launch as well as the rest of the nodes and services needed for the application to work. 


**Saving a Marker Position**

To save a marker position, make sure the back camera of the AMR is facing a marker and detecting it (the camera feed should show a number and coordinate axis on the marker). Once you are in a desired docking position, have the docking toggle set to ON and click "Save Marker Pose". This should save the marker position to be used for docking later.

**Appending A Waypoint Position for Docking**

To have a position set up for docking, drive the AMR anywhere in front of a marker where the back camera can detect it, save the position, and name your waypoint to whatever you want. This should make a waypoint that can see a marker to dock to.

**Docking the AMR to a Marker**

If you have a waypoint set up in front of a marker as well as a docking position saved, you can hit "Go" to that marker with the docking toggle set to ON to have the robot go to your set waypoint and dock to the marker.

*Insert Videos Here of these three processes*

## Tutorial 2 : ROS 2 Interfaces

My project exposes many interfaces that I have made to anyone who wants to use them, including many topics and classes that can be used in future applications. 

### AMR MP-400 Interface

This interface includes two custom server messages ```SetFlag.srv``` and ```GroundTruth.srv```. 


More information on the definition and use of these two messages can be found on the [GitHub](https://github.com/cmu-mfi/amr_mp400_ws/tree/main/src/amr_mp400_interfaces) here

### Waypoint Server

The Waypoint server is an integral part of the control flow, as it takes in requests from the frontend Qt Application and sends it to the rest of the control stack. 

To use the server through the CLI, you can use the command
```
ros2 service call /waypoint_maker amr_mp400_interfaces/srv/SetFlag <flags as described in the github>
```

The waypoint server processes requests given to it and does the control flow for the whole system. 

### Waypoint Client

This waypoint Client is an example of how to send requests to the server. The script includes 

```
def send_request(self, flag, index, docking, label):
    self.request.flag = flag
    self.request.index = index
    self.request.docking = docking
    self.request.label = label

    self.future = self.client.call_async(self.request)
    rclpy.spin_until_future_complete(self, self.future)
    return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    waypoint_client = WaypointClientAsync()

    response = waypoint_client.send_request(ord(sys.argv[1]), int(sys.argv[2]), ord(sys.argv[3]))

    waypoint_client.get_logger().info(
        'Success: %d \nReturn Message: %s' % (response.success, response.msg)
    )

    waypoint_client.destroy_node()
    rclpy.shutdown()

```

showing how to recieve requests from the command line, process them, and send them to the server to be processed


## Tutorial 3: Mapping

To map, first connect to the robot through a VNC client. The configuration for this client should include: 

Protocol: Remmina VNC Plugin (For Remmina Use)

Server: 172.26.190.49

Username: neobotix

Password: neobotix


Once you are connected to the robot, make sure to stop all processes that are already running on the robot and launch ROS on the robot throught the startROS shell file on the desktop. Once that is running, there are two options to start the mapping process, one being a fallback. 

The first way to launch the mapping process is through the shell commands on the desktop, just like the ros commands. Simply start both startMapping and rvizMapping and map the area needed. Once finished, start mapSave and once the terminal closes without any red errors you are free to close all terminals and use this new map. However, if the desktop scripts dont work, the backup is just as simple. On the robots desktop, there is a text file named "Map Saving" and inside are three lines of code that should be run in seperate terminals and treated the same as the launch scripts. 

*Insert Video here of creating the map (Show a video of both ways)*