#include <imc_ros_bridge/imc_to_ros/Abort.h>

namespace imc_to_ros {

template <>
bool convert(const IMC::Abort& imc_msg, std_msgs::Empty& ros_msg)
{
    return true;
}

} // namespace imc_to_ros
