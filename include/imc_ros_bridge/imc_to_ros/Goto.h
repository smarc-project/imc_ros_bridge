#ifndef IMC_TO_ROS_GOTO_H
#define IMC_TO_ROS_GOTO_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <geometry_msgs/Pose.h>
#include <IMC/Spec/Goto.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::Goto& imc_msg, geometry_msgs::Pose& ros_msg);

} // namespace imc_to_ros

#endif // IMC_TO_ROS_GOTO_H
