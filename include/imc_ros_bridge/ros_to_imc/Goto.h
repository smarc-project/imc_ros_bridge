#ifndef ROS_TO_IMC_GOTO_H
#define ROS_TO_IMC_GOTO_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <geometry_msgs/Pose.h>
#include <IMC/Spec/Goto.hpp>

namespace ros_to_imc {

template <>
bool convert(const geometry_msgs::Pose& ros_msg, IMC::Goto& imc_msg);

} // namespace ros_to_imc

#endif // ROS_TO_IMC_GOTO_H
