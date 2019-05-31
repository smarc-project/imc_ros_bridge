#ifndef UDP_LINK_H
#define UDP_LINK_H

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <iostream>

// IMC headers.
#include <IMC/Base/Parser.hpp>

//#define IPADDRESS "127.0.0.1" // "192.168.1.64"
//#define UDP_PORT 30101 //6001

using boost::asio::ip::udp;
using boost::asio::ip::address;

class UDPLink {

private:

    boost::asio::io_service io_service;
    udp::socket socket{io_service};
    udp::socket out_socket{io_service};
    boost::array<char, 1024> recv_buffer;
    udp::endpoint remote_endpoint;
    std::function<void (IMC::Message*)> recv_handler_;
    IMC::Parser parser_;

    std::vector<int> announce_ports{30100, 30101, 30102, 30103, 30104};

public:

    UDPLink(std::function<void (IMC::Message*)> recv_handler,
            const std::string& addr, const std::string& port);

    ~UDPLink();

    void wait();

    void announce(const std::string& addr);

    void handle_receive(const boost::system::error_code& error, size_t bytes_transferred);

};

#endif // UDP_LINK_H
