#include <imc_ros_bridge/ros_to_imc/EstimatedState.h>

namespace ros_to_imc {

template <>
bool convert(const sensor_msgs::NavSatFix& ros_msg, IMC::EstimatedState& imc_msg)
{
    imc_msg.lat = ros_msg.latitude;
    imc_msg.lon = ros_msg.longitude;
    imc_msg.alt = ros_msg.altitude;
    
    return true;
}

} // namespace ros_to_imc
