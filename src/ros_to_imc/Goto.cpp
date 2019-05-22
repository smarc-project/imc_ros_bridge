#include <imc_ros_bridge/ros_to_imc/Goto.h>

namespace ros_to_imc {

template <>
bool convert(const geometry_msgs::Pose& ros_msg, IMC::Goto& imc_msg)
{
    imc_msg.lon = ros_msg.position.x;
    imc_msg.lat = ros_msg.position.y;
    imc_msg.z = ros_msg.position.z;
    return true;
}

} // namespace ros_to_imc
