#include <imc_tcp_link/imc_handle.h>
#include <IMC/Spec/Announce.hpp>
#include <IMC/Spec/Heartbeat.hpp>

#include <functional>
#include <ros/ros.h>

void try_callback(const IMC::Message* imc_msg)
{
    ROS_INFO("Got callback!");
    std::cout << "Got callback with id: " << imc_msg->getId() << std::endl;
}

IMCHandle::IMCHandle(const std::string& tcp_addr, const std::string& tcp_port,
                     const std::string& sys_name, int imc_id)
    : udp_link(std::bind(&IMCHandle::tcp_callback, this, std::placeholders::_1),
               tcp_addr, tcp_port),
      tcp_addr(tcp_addr), tcp_port(tcp_port),
      sys_name(sys_name), imc_id(imc_id)
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
        ROS_INFO("Got callback with id: %u", uid);
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
    std::string announce_addr = "224.0.75.69";

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
    msg.services = "imc+udp://" + tcp_addr + ":" + tcp_port + "/;";
    udp_link.publish_multicast(msg, announce_addr);
}

void IMCHandle::publish_heartbeat()
{
    IMC::Heartbeat msg;
    udp_link.publish(msg);
}
