#ifndef IMC_TO_ROS_PLANCONTROL_H
#define IMC_TO_ROS_PLANCONTROL_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include "imc_ros_bridge/PlanControl.h"
#include <IMC/Spec/PlanControl.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::PlanControl& imc_msg, imc_ros_bridge::PlanControl& ros_msg);

} // namespace imc_to_ros

#endif // IMC_TO_ROS_GOTO_H
