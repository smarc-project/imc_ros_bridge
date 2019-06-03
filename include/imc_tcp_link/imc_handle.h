#ifndef IMC_HANDLE_H
#define IMC_HANDLE_H

//#include <imc_tcp_link/TcpLink.hpp>
#include <imc_udp_link/udp_link.h>

class IMCHandle {

private:

    std::string sys_name;
    int imc_id;
    std::string tcp_addr;
    std::string tcp_port;

    //ros_imc_broker::TcpLink* tcp_client_;
    //boost::thread* tcp_client_thread_;
    UDPLink udp_link;

    // we might need multiple for every message in the future, let's start here
    std::map<uint16_t, std::function<void(const IMC::Message*)> > callbacks;

public:

    IMCHandle(const std::string& tcp_addr, const std::string& tcp_port,
              const std::string& sys_name, int imc_id);

    ~IMCHandle();

    void announce();

    void publish_heartbeat();

    void tcp_subscribe(uint16_t uid, std::function<void(const IMC::Message*)> callback);

    void tcp_callback(const IMC::Message* msg);

    template <typename IMC_MSG>
    void write(IMC_MSG& imc_msg)
    {
        //tcp_client_.write(imc_msg);
        //tcp_client_->write(&imc_msg);
        udp_link.publish(imc_msg);
    }

};

#endif // IMC_HANDLE_H
