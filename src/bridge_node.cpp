/* Copyright 2019 The SMaRC project (https://smarc.se/)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <iostream>

#include <imc_ros_bridge/imc_ros_bridge_server.h>

#include <imc_ros_bridge/ros_to_imc/Heartbeat.h>
#include <imc_ros_bridge/ros_to_imc/GpsFix.h>
#include <imc_ros_bridge/ros_to_imc/Goto.h>
#include <imc_ros_bridge/ros_to_imc/GpsNavData.h>
#include <imc_ros_bridge/ros_to_imc/EstimatedState.h>
#include <imc_ros_bridge/ros_to_imc/PlanControlState.h>
#include <imc_ros_bridge/ros_to_imc/VehicleState.h>
#include <imc_ros_bridge/ros_to_imc/RemoteState.h>
#include <imc_ros_bridge/ros_to_imc/SonarData.h>
#include <imc_ros_bridge/ros_to_imc/DesiredHeading.h>
#include <imc_ros_bridge/ros_to_imc/DesiredHeadingRate.h>
#include <imc_ros_bridge/ros_to_imc/DesiredPitch.h>
#include <imc_ros_bridge/ros_to_imc/DesiredRoll.h>
#include <imc_ros_bridge/ros_to_imc/DesiredSpeed.h>
#include <imc_ros_bridge/ros_to_imc/DesiredZ.h>
#include <imc_ros_bridge/ros_to_imc/PlanDB.h>

#include <imc_ros_bridge/imc_to_ros/Goto.h>
#include <imc_ros_bridge/imc_to_ros/Heartbeat.h>
#include <imc_ros_bridge/imc_to_ros/Abort.h>
#include <imc_ros_bridge/imc_to_ros/PlanDB.h>
#include <imc_ros_bridge/imc_to_ros/PlanControl.h>


using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imc_bridge");
    ros::NodeHandle ros_node;

    std::string neptus_addr;
    std::string bridge_tcp_addr;
    std::string bridge_tcp_port;
    std::string sys_name;
    int imc_id;
	int imc_src;
    ros::param::param<std::string>("~neptus_addr", neptus_addr, "127.0.0.1");
    ros::param::param<std::string>("~bridge_addr", bridge_tcp_addr, "127.0.0.1");
    ros::param::param<std::string>("~bridge_port", bridge_tcp_port, "6002");
    ros::param::param<std::string>("~system_name", sys_name, "imc_ros_bridge");
	// Neptus seems to have swapped these values for SOME REASON. so I swap here too...
	// tldr: imc_id is the vehicle TYPE and imc_src is the SPECIFIC VEHICLE
    ros::param::param<int>("~imc_id", imc_src, 30);
	ros::param::param<int>("~imc_src", imc_id, 5);

    IMCHandle imc_handle(bridge_tcp_addr, bridge_tcp_port, neptus_addr, sys_name, imc_id, imc_src);

    ros_to_imc::BridgeServer<std_msgs::Empty, IMC::Heartbeat> heartbeat_server(ros_node, imc_handle, "heartbeat");
    ros_to_imc::BridgeServer<sensor_msgs::NavSatFix, IMC::GpsFix> gpsfix_server(ros_node, imc_handle, "gps_fix");
    ros_to_imc::BridgeServer<geometry_msgs::Pose, IMC::Goto> goto_server_dummy(ros_node, imc_handle, "goto_input");
    ros_to_imc::BridgeServer<sensor_msgs::NavSatFix, IMC::GpsNavData> gpsnavdata_server(ros_node, imc_handle, "gps_nav_data");
    ros_to_imc::BridgeServer<imc_ros_bridge::EstimatedState, IMC::EstimatedState> estimatedstate_server(ros_node, imc_handle, "estimated_state");
    ros_to_imc::BridgeServer<imc_ros_bridge::PlanControlState, IMC::PlanControlState> plancontrolstate_server(ros_node, imc_handle, "plan_control_state");
    ros_to_imc::BridgeServer<imc_ros_bridge::VehicleState, IMC::VehicleState> vehiclestate_server(ros_node, imc_handle, "vehicle_state");
    ros_to_imc::BridgeServer<imc_ros_bridge::RemoteState, IMC::RemoteState> RemoteState_server(ros_node, imc_handle, "remote_state");
    ros_to_imc::BridgeServer<imc_ros_bridge::SonarData, IMC::SonarData> sonar_server(ros_node, imc_handle, "sonar_data");
    ros_to_imc::BridgeServer<imc_ros_bridge::DesiredHeading, IMC::DesiredHeading> DesiredHeading_server(ros_node, imc_handle, "desired_heading");
    ros_to_imc::BridgeServer<imc_ros_bridge::DesiredHeadingRate, IMC::DesiredHeadingRate> DesiredHeadingRate_server(ros_node, imc_handle, "desired_heading_rate");
    ros_to_imc::BridgeServer<imc_ros_bridge::DesiredPitch, IMC::DesiredPitch> DesiredPitch_server(ros_node, imc_handle, "desired_pitch");
    ros_to_imc::BridgeServer<imc_ros_bridge::DesiredRoll, IMC::DesiredRoll> DesiredRoll_server(ros_node, imc_handle, "desired_roll");
    ros_to_imc::BridgeServer<imc_ros_bridge::DesiredSpeed, IMC::DesiredSpeed> DesiredSpeed_server(ros_node, imc_handle, "desired_speed");
    ros_to_imc::BridgeServer<imc_ros_bridge::DesiredZ, IMC::DesiredZ> DesiredZ_server(ros_node, imc_handle, "desired_z");
    ros_to_imc::BridgeServer<imc_ros_bridge::PlanDB, IMC::PlanDB> to_imc_plandb_server(ros_node, imc_handle, "plan_db");
  
    // 450
    imc_to_ros::BridgeServer<IMC::Goto, geometry_msgs::Pose> goto_server(imc_handle, ros_node, "goto_waypoint");
    // 150
    imc_to_ros::BridgeServer<IMC::Heartbeat, std_msgs::Empty> imc_heartbeat_server(imc_handle, ros_node, "imc_heartbeat");
    // 550
    imc_to_ros::BridgeServer<IMC::Abort, std_msgs::Empty> abort_server(imc_handle, ros_node, "abort");
    // 556
    imc_to_ros::BridgeServer<IMC::PlanDB, imc_ros_bridge::PlanDB> plandb_server(imc_handle, ros_node, "plan_db");
    imc_to_ros::BridgeServer<IMC::PlanControl, imc_ros_bridge::PlanControl> plancontrol_server(imc_handle, ros_node, "plan_control");

    auto announce_callback = [&](const ros::TimerEvent&) { imc_handle.announce(); };
    auto heartbeat_callback = [&](const ros::TimerEvent&) { imc_handle.publish_heartbeat(); };

    ros::Timer announce_timer = ros_node.createTimer(ros::Duration(10.), announce_callback);
    ros::Timer heartbeat_timer = ros_node.createTimer(ros::Duration(1.), heartbeat_callback);

    ros::spin();

    return 0;
}
