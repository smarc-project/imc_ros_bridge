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

#include <imc_tcp_link/imc_handle.h>
#include <IMC/Spec/Announce.hpp>
#include <IMC/Spec/Heartbeat.hpp>
#include <IMC/Spec/EntityInfo.hpp>

#include <functional>
#include <ros/ros.h>

void try_callback(const IMC::Message* imc_msg)
{
		ROS_INFO("Got callback!");
		std::cout << "Got callback with id: " << imc_msg->getId() << std::endl;
}

IMCHandle::IMCHandle(const std::string& bridge_tcp_addr, 
					 const std::string& bridge_tcp_port,
                     const std::string& neptus_addr,
                     const std::string& sys_name, 
					 int imc_id, 
					 int imc_src)
    : udp_link(std::bind(&IMCHandle::tcp_callback, this, std::placeholders::_1),
               bridge_tcp_addr, bridge_tcp_port, imc_id, imc_src),
      neptus_addr(neptus_addr),
      bridge_tcp_addr(bridge_tcp_addr), bridge_tcp_port(bridge_tcp_port),
      sys_name(sys_name), imc_id(imc_id), imc_src(imc_src)
{
    lat = 0.0;
    announce();
}

IMCHandle::~IMCHandle()
{

}

void IMCHandle::tcp_subscribe(uint16_t uid, std::function<void(const IMC::Message*)> callback)
{
    callbacks[uid] = callback;
}

void IMCHandle::tcp_callback(const IMC::Message* msg)
{
    uint16_t uid = msg->getId();
    if (callbacks.count(uid) > 0) {
		// 150 is a heartbeat and we dont really care about it. just debug it.
		// 556 is PlanDB, i _think_ its the planDB succss, meaning "i understood that you got my plan"
		// neptus basically spams this so im excluding it!
		if(uid == 150 || uid == 556){
			ROS_DEBUG("Got callback with id: %u", uid);
		}else{
			ROS_INFO("Got callback with id: %u", uid);
		}
		callbacks.at(uid)(msg);
    }
    else {
        ROS_INFO("Got tcp message with no configure callback, msgid: %u!", uid);
	// lets just print the whole message in json format if we can't parse it yet.
	std::cout << "Message name: " << msg->getName() << std::endl << "Message JSON:" << std::endl;
	// (ostream, indent)
	msg->fieldsToJSON(std::cout, 4);
	std::cout << std::endl;
    std::cout << "---------------------" << std::endl;
    }
}

void IMCHandle::announce()
{
    //std::string announce_addr = "224.0.75.69";
    //std::string announce_addr = "192.168.1.160";

    IMC::Announce msg;
    msg.sys_name = sys_name;
    // 0=CCU, 1=HUMANSENSOR, 2 = UUV, 3 = ASV, 4=UAV, 5=UGV, 6=STATICSENSOR
    msg.sys_type = 2; // UUV = Unmanned underwater veh.
    msg.owner = 0;
    // dont put location info here, this is only updated once
    // use EstimatedState for continous updates of location.
    //lat +=0.01;
    //msg.lat = lat;
    //msg.lon = 0.7;
    //msg.height = -1.;
    //msg.services = "imc+info://0.0.0.0/version/5.4.11/;imc+udp://127.0.0.1:6002/;";
    msg.services = "imc+udp://" + bridge_tcp_addr + ":" + bridge_tcp_port + "/;";
    udp_link.publish_multicast(msg, neptus_addr);

    //TEST Publish EntityInfo
    IMC::EntityInfo info_msg;
    //info_msg.id = udp_link.imc_src; //What is this used for?
    info_msg.label = sys_name;
    udp_link.publish(info_msg, neptus_addr);
}

void IMCHandle::publish_heartbeat()
{
    IMC::Heartbeat msg;
    udp_link.publish(msg, neptus_addr);
}
