#ifndef ROS_TO_IMC_HEARTBEAT_H
#define ROS_TO_IMC_HEARTBEAT_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <std_msgs/Empty.h>
#include <IMC/Spec/Heartbeat.hpp>

namespace ros_to_imc {

template <>
bool convert(const std_msgs::Empty& ros_msg, IMC::Heartbeat& imc_msg);

} // namespace ros_to_imc

#endif // ROS_TO_IMC_HEARTBEAT_H
