#ifndef IMC_TO_ROS_PLANDB_H
#define IMC_TO_ROS_PLANDB_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <std_msgs/String.h>
#include <IMC/Spec/PlanDB.hpp>


namespace imc_to_ros {

template <>
bool convert(const IMC::PlanDB& imc_msg, std_msgs::String& ros_msg);

} // namespace imc_to_ros

#endif // IMC_TO_ROS_GOTO_H
