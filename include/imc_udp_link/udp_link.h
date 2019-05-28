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
#define UDP_PORT 13251

using boost::asio::ip::udp;
using boost::asio::ip::address;

class UDPLink {

private:

    boost::asio::io_service io_service;
    udp::socket socket{io_service};
    boost::array<char, 1024> recv_buffer;
    udp::endpoint remote_endpoint;
    boost::function<void (IMC::Message*)> recv_handler_;

public:

    UDPLink(boost::function<void (IMC::Message*)> recv_handler,
            const std::string& addr, const std::string& port)
        : recv_handler_(recv_handler)
    {
        socket.open(udp::v4());
        socket.bind(udp::endpoint(address::from_string(addr), UDP_PORT));

        wait();

        std::cout << "Receiving\n";
        io_service.run();
        std::cout << "Receiver exit\n";
    }

    ~UDPLink() {}

    void wait()
    {
        socket.async_receive_from(boost::asio::buffer(recv_buffer),
                                  remote_endpoint,
                                  boost::bind(&Client::handle_receive,
                                              this, boost::asio::placeholders::error,
                                              boost::asio::placeholders::bytes_transferred));
    }

    void handle_receive(const boost::system::error_code& error, size_t bytes_transferred)
    {
        if (error) {
            std::cout << "Receive failed: " << error.message() << "\n";
            return;
        }

        std::cout << "Received: '" << std::string(recv_buffer.begin(), recv_buffer.begin()+bytes_transferred) << "' (" << error.message() << ")\n";

        for (size_t i = 0; i < bytes_transferred; ++i) {
            IMC::Message* m = parser_.parse((uint8_t)in_buffer_[i]);
            if (m) {
                recv_handler_(m);
                delete m;
            }
        }

        if (--count > 0) {
            std::cout << "Count: " << count << "\n";
            wait();
        }
    }

};

#endif // UDP_LINK_H
