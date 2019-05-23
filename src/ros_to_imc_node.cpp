
#include <ros/ros.h>
#include <iostream>

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_bridge/ros_to_imc/Heartbeat.h>
#include <imc_ros_bridge/ros_to_imc/GpsFix.h>
#include <imc_ros_bridge/ros_to_imc/Goto.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_to_imc_node");
    ros::NodeHandle ros_node;

    std::string tcp_addr = "127.0.0.1";
    std::string tcp_port = "6001";

    IMCHandle imc_handle(tcp_addr, tcp_port);

    ros_to_imc::BridgeServer<std_msgs::Empty, IMC::Heartbeat> heartbeat_server(ros_node, imc_handle, "/heartbeat");
    ros_to_imc::BridgeServer<sensor_msgs::NavSatFix, IMC::GpsFix> gpsfix_server(ros_node, imc_handle, "/gps_fix");
    ros_to_imc::BridgeServer<geometry_msgs::Pose, IMC::Goto> goto_server(ros_node, imc_handle, "/goto_input");

    ros::spin();

    return 0;
}
