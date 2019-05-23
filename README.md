# imc_ros_bridge

Minimal library for bridging ROS and IMC messages

## Building

A simple `catkin_make` should do.

## Launching

The command
```
roslaunch imc_ros_bridge bridge.launch server_addr:=127.0.0.1 server_port:=6001
```
will launch the `imc_to_ros_node` and `ros_to_imc_node` nodes.

## Trying with Neptus

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
I have not figured out how to check if the other direction works yet, feel free to investigate this! The IMC Message Sender
seems to broadcast only over UDP, but we just subscribe to TCP so that doesn't work. I might be wrong on this though.

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
You then need to add a `BridgeServer` to the nodes, see details below.

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

And they also add the bridge server to the `ros_to_imc_node` like this:
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

And they also add the bridge server to the `imc_to_ros_node` like this:
```cpp
imc_to_ros::BridgeServer<IMC::Goto, geometry_msgs::Pose> goto_server(imc_handle, ros_node, "/goto_waypoint");
```
And link the convert libary into `imc_to_ros_node` in the `CMakeLists.txt` file.
