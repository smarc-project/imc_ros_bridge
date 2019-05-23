#include <imc_tcp_link/imc_handle.h>

#include <functional>

void try_callback(const IMC::Message* imc_msg)
{
    ROS_INFO("Got callback!");
    std::cout << "Got callback with id: " << imc_msg->getId() << std::endl;
}

IMCHandle::IMCHandle(const std::string& tcp_addr, const std::string& tcp_port) //: tcp_client_(std::bind(&IMCHandle::tcp_callback, this, std::placeholders::_1))
{
    start(tcp_addr, tcp_port);
}

void IMCHandle::start(const std::string& tcp_addr, const std::string& tcp_port)
{
    tcp_client_ = new ros_imc_broker::TcpLink(std::bind(&IMCHandle::tcp_callback, this, std::placeholders::_1));
    //tcp_client_ = new ros_imc_broker::TcpLink(&try_callback);
    tcp_client_->setServer(tcp_addr, tcp_port);

    //! TCP client thread.
    tcp_client_thread_ = new boost::thread(boost::ref(*tcp_client_));
}

IMCHandle::~IMCHandle()
{
    tcp_client_thread_->interrupt();
    tcp_client_thread_->join();
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
    }
}
