#include <imc_ros_bridge/ros_to_imc/Heartbeat.h>

namespace ros_to_imc {

template <>
bool convert(const std_msgs::Empty& ros_msg, IMC::Heartbeat& imc_msg)
{
    return true;
}

} // namespace ros_to_imc
