#include <ros/ros.h>
#include <iostream>

#include <imc_ros_bridge/imc_ros_bridge_server.h>

#include <imc_ros_bridge/ros_to_imc/Heartbeat.h>
#include <imc_ros_bridge/ros_to_imc/GpsFix.h>
#include <imc_ros_bridge/ros_to_imc/Goto.h>

#include <imc_ros_bridge/imc_to_ros/Goto.h>
#include <imc_ros_bridge/imc_to_ros/Heartbeat.h>
#include <imc_ros_bridge/imc_to_ros/Abort.h>
#include <imc_ros_bridge/imc_to_ros/PlanDB.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imc_bridge");
    ros::NodeHandle ros_node;

    std::string tcp_addr;
    std::string tcp_port;
    std::string sys_name;
    int imc_id;

    ros::param::param<std::string>("~server_addr", tcp_addr, "127.0.0.1");
    ros::param::param<std::string>("~server_port", tcp_port, "6002");
    ros::param::param<std::string>("~sys_name", sys_name, "sam-auv-1");
    ros::param::param<int>("~imc_id", imc_id, 30);

    IMCHandle imc_handle(tcp_addr, tcp_port, sys_name, imc_id);

    ros_to_imc::BridgeServer<std_msgs::Empty, IMC::Heartbeat> heartbeat_server(ros_node, imc_handle, "/heartbeat");
    ros_to_imc::BridgeServer<sensor_msgs::NavSatFix, IMC::GpsFix> gpsfix_server(ros_node, imc_handle, "/gps_fix");
    ros_to_imc::BridgeServer<geometry_msgs::Pose, IMC::Goto> goto_server_dummy(ros_node, imc_handle, "/goto_input");

    // 450
    imc_to_ros::BridgeServer<IMC::Goto, geometry_msgs::Pose> goto_server(imc_handle, ros_node, "/goto_waypoint");
    // 150
    imc_to_ros::BridgeServer<IMC::Heartbeat, std_msgs::Empty> imc_heartbeat_server(imc_handle, ros_node, "/imc_heartbeat");
    // 550
    imc_to_ros::BridgeServer<IMC::Abort, std_msgs::Empty> abort_server(imc_handle, ros_node, "/abort");
    // 556
    imc_to_ros::BridgeServer<IMC::PlanDB, std_msgs::String> plandb_server(imc_handle, ros_node, "/plan_db");

    auto announce_callback = [&](const ros::TimerEvent&) { imc_handle.announce(); };
    auto heartbeat_callback = [&](const ros::TimerEvent&) { imc_handle.publish_heartbeat(); };

    ros::Timer announce_timer = ros_node.createTimer(ros::Duration(10.), announce_callback);
    ros::Timer heartbeat_timer = ros_node.createTimer(ros::Duration(1.), heartbeat_callback);

    ros::spin();

    return 0;
}
