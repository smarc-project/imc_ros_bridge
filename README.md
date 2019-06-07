# imc_ros_bridge

Minimal library for bridging ROS and IMC messages

## Building

A simple `catkin_make` should do.

## Launching

The command
```
roslaunch imc_ros_bridge bridge.launch server_addr:=127.0.0.1 server_port:=6002
```
will launch the `imc_bridge` node.

## Trying with Neptus

These instructions have been tested for Ubuntu 16.04 and 18.04. For 18.04, you
may have to comment a line in a jdk file to get it running (TODO: check path to file).

First, install dependencies: `sudo apt install openjdk-8-jdk-headless ant-optional`.

Then clone and build neptus:
```
git clone https://github.com/LSTS/neptus.git
cd neptus
ant
```
If successful, run it with `./neptus.sh`. Open the comm monitor by clicking `Communications > IMC Comm. Monitor`.
In the status panel, click the button with the IMC logo (with hover text `Start IMC comms`) to start up IMC.
Go to the all messages panel to see the messages that you publish from ROS.

You can publish e.g. the `/heartbeat` message in ROS and see that it appears in the all messages panel (see "existing conversions" below):
```
rostopic pub /heartbeat std_msgs/Empty "{}" --once
```

If you go to the systems list panel, you should also see the ROS auv displayed as a cyan panel.
This indicates that neptus can communicate with the auv. Any other color indicates som problem,
see [this link](https://www.lsts.pt/neptus/manual/trunk/elements.html#systems-list).

## Existing conversions

### ros_to_imc

* `std_msgs/Empty` on topic `/heartbeat` -> `IMC::Heartbeat`
* `sensor_msgs/NavSatFix` on topic `/gps_fix` -> `IMC::GpsFix`
* `geometry_msgs/Pose` on topic `/goto_input` -> `IMC::Goto`

### imc_to_ros

* `IMC::Goto` -> `geometry_msgs/Pose` on topic `/goto_waypoint`

## Creating new conversions

For each conversion in either direction, you need to create a new library
that contains a specialized `convert` function for the types you want to convert.
You then need to add a `BridgeServer` to the node, see details below.

### ros_to_imc

Check out examples in the `include/imc_ros_bridge/ros_to_imc` and `src/ros_to_imc` folders
for more examples. All conversions specialize the `convert` function like this:

```cpp
namespace ros_to_imc {
template <>
bool convert(const sensor_msgs::NavSatFix& ros_msg, IMC::GpsFix& imc_msg)
{
    imc_msg.lat = ros_msg.latitude;
    imc_msg.lon = ros_msg.longitude;
    imc_msg.height = ros_msg.altitude;

    return true;
}
}
```

And they also add the bridge server to the `bridge_node` like this:
```cpp
ros_to_imc::BridgeServer<sensor_msgs::NavSatFix, IMC::GpsFix> gpsfix_server(ros_node, imc_handle, "/gps_fix");
```
And link the convert libary into `ros_to_imc_node` in the `CMakeLists.txt` file.

### imc_to_ros

Check out examples in the `include/imc_ros_bridge/imc_to_ros` and `src/imc_to_ros` folders
for more examples. All conversions specialize the `convert` function like this:

```cpp
namespace imc_to_ros {
template <>
bool convert(const IMC::Goto& imc_msg, geometry_msgs::Pose& ros_msg)
{
    ros_msg.position.x = imc_msg.lon;
    ros_msg.position.y = imc_msg.lat;
    ros_msg.position.z = imc_msg.z;
    return true;
}
}
```

And they also add the bridge server to the `bridge_node` like this:
```cpp
imc_to_ros::BridgeServer<IMC::Goto, geometry_msgs::Pose> goto_server(imc_handle, ros_node, "/goto_waypoint");
```
And link the convert libary into `imc_to_ros_node` in the `CMakeLists.txt` file.

### Adding SAM to Neptus

Link `sam_files/00-sam-auv.nvcl` into `.../neptus/vehicle-defs/` and `sam_files/sam` folder into `.../neptus/vehicle_files`.
This will add SAM to the list of vehicles available in the list, with SAM's visuals.


### Moving SAM around

Using the console (opened by going to Vehicles -> SAM\_AUV -> console) generate a plan (In the console window: Tools -> Generate plan). Select the plan on the right section of the console and click the blue arrow towards the top. This will create and send out a JSON formatted plan to the ros topic `/plan_db`. Parse this JSON, and control SAM.

In order to see the updated pose of SAM in the Neptus console, publish to the ros topic `/estimated_state`. Currently only lat, lon, altitude are used. The update rate on the Neptus console is about once every 1-2 seconds, be patient.

### Emergency

From the Neptus console, the big red ABORT button can be used to send an empty message to the ros topic `/abort`. Probably a good idea to subscribe to this topic.






