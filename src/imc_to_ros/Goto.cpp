#include <imc_ros_bridge/imc_to_ros/Goto.h>

namespace imc_to_ros {

template <>
bool convert(const IMC::Goto& imc_msg, geometry_msgs::Pose& ros_msg)
{
    ros_msg.position.x = imc_msg.lon;
    ros_msg.position.y = imc_msg.lat;
    ros_msg.position.z = imc_msg.z;
    return true;
}

} // namespace imc_to_ros
