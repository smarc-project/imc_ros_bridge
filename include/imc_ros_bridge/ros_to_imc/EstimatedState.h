#ifndef ROS_TO_IMC_ESTIMATEDSTATE_H
#define ROS_TO_IMC_ESTIMATEDSTATE_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <sensor_msgs/NavSatFix.h>
#include <IMC/Spec/EstimatedState.hpp>

namespace ros_to_imc {

template <>
bool convert(const sensor_msgs::NavSatFix& ros_msg, IMC::EstimatedState& imc_msg);

} // namespace ros_to_imc

#endif // ROS_TO_IMC_HEARTBEAT_H
