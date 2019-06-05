#ifndef IMC_TO_ROS_ABORT_H
#define IMC_TO_ROS_ABORT_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <std_msgs/Empty.h>
#include <IMC/Spec/Abort.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::Abort& imc_msg, std_msgs::Empty& ros_msg);

} // namespace imc_to_ros

#endif // IMC_TO_ROS_GOTO_H
