#ifndef IMC_HANDLE_H
#define IMC_HANDLE_H

#include <imc_tcp_link/TcpLink.hpp>

class IMCHandle {

private:

    ros_imc_broker::TcpLink* tcp_client_;
    boost::thread* tcp_client_thread_;
    // we might need multiple for every message in the future, let's start here
    std::map<uint16_t, std::function<void(const IMC::Message*)> > callbacks;

public:

    IMCHandle(const std::string& tcp_addr, const std::string& tcp_port);
    void start(const std::string& tcp_addr, const std::string& tcp_port);

    ~IMCHandle();

    void tcp_subscribe(uint16_t uid, std::function<void(const IMC::Message*)> callback);

    void tcp_callback(const IMC::Message* msg);

    template <typename IMC_MSG>
    void write(const IMC_MSG& imc_msg)
    {
        //tcp_client_.write(imc_msg);
        tcp_client_->write(&imc_msg);
    }

};

#endif // IMC_HANDLE_H
