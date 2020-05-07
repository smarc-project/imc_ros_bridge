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

#ifndef UDP_LINK_H
#define UDP_LINK_H

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
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

    std::string bridge_addr;
    std::string bridge_port;

    boost::asio::io_service io_service;
    udp::socket socket{io_service};
    udp::socket multicast_socket{io_service};
    // 32KiB in bits. Max UDP packet size.
    boost::array<char, 262144> recv_buffer;
    udp::endpoint remote_endpoint;
    std::function<void (IMC::Message*)> recv_handler_;
    IMC::Parser parser_;
    boost::thread run_thread;

    std::vector<int> announce_ports{30100, 30101, 30102, 30103, 30104};

    bool should_shutdown;

    int imc_src = 4;
    int imc_src_ent = 32;
	int imc_id;

public:

    UDPLink(std::function<void (IMC::Message*)> recv_handler,
            const std::string& bridge_addr, const std::string& bridge_port,
            int imc_id, int imc_src);

    ~UDPLink();

    void wait();

    void publish(IMC::Message& msg, const std::string& address);

    void publish_multicast(IMC::Message& msg, const std::string& multicast_addr);

    void handle_receive(const boost::system::error_code& error, size_t bytes_transferred);

};

#endif // UDP_LINK_H
