#ifndef IMC_TO_ROS_PLANCONTROLSTATE_H
#define IMC_TO_ROS_PLANCONTROLSTATE_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include "imc_ros_bridge/PlanControlState.h"
#include <IMC/Spec/PlanControlState.hpp>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_bridge::PlanControlState& ros_msg, IMC::PlanControlState& imc_msg);

} // namespace imc_to_ros

#endif // IMC_TO_ROS_GOTO_H
