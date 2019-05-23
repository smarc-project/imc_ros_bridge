#include <ros/ros.h>
#include <iostream>

#include <imc_tcp_link/imc_handle.h>
#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_bridge/imc_to_ros/Goto.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imc_to_ros_node");
    ros::NodeHandle ros_node;

    std::string tcp_addr;
    std::string tcp_port;

    ros::param::param<std::string>("~server_addr", tcp_addr, "127.0.0.1");
    ros::param::param<std::string>("~server_port", tcp_port, "6001");

    IMCHandle imc_handle(tcp_addr, tcp_port);

    imc_to_ros::BridgeServer<IMC::Goto, geometry_msgs::Pose> goto_server(imc_handle, ros_node, "/goto_waypoint");

    ros::spin();

    return 0;
}
