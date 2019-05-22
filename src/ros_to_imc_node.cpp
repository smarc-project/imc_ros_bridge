
#include <ros/ros.h>
#include <iostream>

#include <imc_ros_bridge/imc_ros_bridge_server.h>
#include <imc_ros_bridge/ros_to_imc/Heartbeat.h>
#include <imc_ros_bridge/ros_to_imc/GpsFix.h>

using namespace std;

void imc_callback(const IMC::Message* msg)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_to_imc_node");
    ros::NodeHandle ros_node;

    std::string tcp_addr;
    std::string tcp_port;

    ros_imc_broker::TcpLink tcp_client_(&imc_callback);
    tcp_client_.setServer(tcp_addr, tcp_port);

    //! TCP client thread.
    boost::thread tcp_client_thread_(boost::ref(tcp_client_));

    ros_to_imc::BridgeServer<std_msgs::Empty, IMC::Heartbeat> heatbeat_server(ros_node, tcp_client_, "/heartbeat");
    ros_to_imc::BridgeServer<sensor_msgs::NavSatFix, IMC::GpsFix> gpsfix_server(ros_node, tcp_client_, "/goto_waypoint");

    ros::spin();

    // clean up the thread
    tcp_client_thread_.interrupt();
    tcp_client_thread_.join();

    return 0;
}
