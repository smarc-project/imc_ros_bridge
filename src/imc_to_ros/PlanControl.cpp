#include <imc_ros_bridge/imc_to_ros/PlanControl.h>

namespace imc_to_ros {

template <>
bool convert(const IMC::PlanControl& imc_msg, imc_ros_bridge::PlanControl& ros_msg)
{
    ros_msg.type = imc_msg.type;
    ros_msg.op = imc_msg.op;
    ros_msg.request_id = imc_msg.request_id;
    ros_msg.plan_id = imc_msg.plan_id;
    ros_msg.flags = imc_msg.flags;
    ros_msg.info = imc_msg.info;
       
    return true;
}

} // namespace imc_to_ros
