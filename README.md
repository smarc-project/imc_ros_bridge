# imc_ros_bridge

Minimal library for bridging ROS and IMC messages

## Building

A simple `catkin_make` should do.

## Launching

The command
```
roslaunch imc_ros_bridge bridge.launch
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

## Existing conversions

### ros_to_imc

* `std_msgs/Empty` on topic `/heartbeat` -> `IMC::Heartbeat`
* `sensor_msgs/NavSatFix` on topic `/gps_fix` -> `IMC::GpsFix`
* `geometry_msgs/Pose` on topic `/goto_input` -> `IMC::Goto`

### imc_to_ros

* `IMC::Goto` -> `geometry_msgs/Pose` on topic `/goto_waypoint`

## Creating new conversions
