#ifndef ROS_TO_IMC_GPSNAVDATA_H
#define ROS_TO_IMC_GPSNAVDATA_H

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <sensor_msgs/NavSatFix.h>
#include <IMC/Spec/GpsNavData.hpp>

namespace ros_to_imc {

template <>
bool convert(const sensor_msgs::NavSatFix& ros_msg, IMC::GpsNavData& imc_msg);

} // namespace ros_to_imc

#endif // ROS_TO_IMC_GPSFIX_H
