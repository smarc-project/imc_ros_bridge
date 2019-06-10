#include <imc_ros_bridge/ros_to_imc/PlanControlState.h>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_bridge::PlanControlState& ros_msg, IMC::PlanControlState& imc_msg)
{
    imc_msg.state = ros_msg.state;
    imc_msg.plan_id = ros_msg.plan_id;
    imc_msg.plan_eta = ros_msg.plan_eta;
    imc_msg.plan_progress = ros_msg.plan_progress;
    imc_msg.man_id = ros_msg.man_id;
    imc_msg.man_type = ros_msg.man_type;
    imc_msg.man_eta = ros_msg.man_eta;
    imc_msg.last_outcome = ros_msg.last_outcome;
       
    return true;
}

} // namespace imc_to_ros
