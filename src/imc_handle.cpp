#include <imc_tcp_link/imc_handle.h>

#include <functional>

IMCHandle::IMCHandle(const std::string& tcp_addr, const std::string& tcp_port) : tcp_client_(std::bind(&IMCHandle::tcp_callback, this, std::placeholders::_1))
{
    tcp_client_.setServer(tcp_addr, tcp_port);

    //! TCP client thread.
    tcp_client_thread_ = boost::thread(boost::ref(tcp_client_));
}

IMCHandle::~IMCHandle()
{
    tcp_client_thread_.interrupt();
    tcp_client_thread_.join();
}

void IMCHandle::tcp_subscribe(uint16_t uid, std::function<void(const IMC::Message*)> callback)
{
    callbacks[uid] = callback;
}

void IMCHandle::tcp_callback(const IMC::Message* msg)
{
    callbacks.at(msg->getId())(msg);
}
