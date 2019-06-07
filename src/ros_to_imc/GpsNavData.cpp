#include <imc_ros_bridge/ros_to_imc/GpsNavData.h>

namespace ros_to_imc {

template <>
bool convert(const sensor_msgs::NavSatFix& ros_msg, IMC::GpsNavData& imc_msg)
{
    imc_msg.lat = ros_msg.latitude;
    imc_msg.lon = ros_msg.longitude;
    imc_msg.height_sea = ros_msg.altitude;

    return true;
}

} // namespace ros_to_imc

