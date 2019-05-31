#include <imc_udp_link/udp_link.h>
#include <IMC/Spec/Announce.hpp>

#include <boost/lexical_cast.hpp>

using namespace std;

UDPLink::UDPLink(std::function<void (IMC::Message*)> recv_handler,
        const std::string& addr, const std::string& port)
    : recv_handler_(recv_handler)
{

    cout << __FILE__ << ", " << __LINE__ << endl;
    socket.open(udp::v4());
    socket.set_option(udp::socket::reuse_address(true));
    cout << __FILE__ << ", " << __LINE__ << endl;
    //socket.bind(udp::endpoint(boost::asio::ip::address_v4::any(), UDP_PORT));
    socket.bind(udp::endpoint(address::from_string(addr), boost::lexical_cast<int>(port)));
    //socket.bind(udp::endpoint(address::from_string("127.0.0.1"), UDP_PORT));
    cout << __FILE__ << ", " << __LINE__ << endl;

    wait();
    cout << __FILE__ << ", " << __LINE__ << endl;

    //auto work = std::make_shared<boost::asio::io_service::work>(io_service);
    announce("224.0.75.69");

    std::cout << "Receiving\n";
    io_service.run();
    std::cout << "Receiver exit\n";
}

UDPLink::~UDPLink() {}

void handler(const boost::system::error_code& error, size_t bytes_transferred)
{
    std::cout << "Got some successful bytes transferred: " << bytes_transferred << std::endl;
}

void UDPLink::announce(const string& addr)
{
    IMC::Announce msg;
    msg.sys_name = "lauv-xplore-1";
    msg.sys_type = 30; // lauv-xplore-1
    msg.owner = 0; // do not know what this is
    msg.lat = 5.;
    msg.lon = 10.;
    msg.height = -1.;
    msg.services = "imc+udp://127.0.0.1:6002/;";

    char out_buffer_[1024];
    uint16_t rv = IMC::Packet::serialize(&msg, (uint8_t*)out_buffer_, sizeof(out_buffer_));

    //boost::asio::io_service service;
    out_socket.open(boost::asio::ip::udp::v4());

    // Allow other processes to reuse the address, permitting other processes on
//  // the same machine to use the multicast address.
    out_socket.set_option(udp::socket::reuse_address(true));
    // Guarantee the loopback is enabled so that multiple processes on the same
    // machine can receive data that originates from the same socket.
    out_socket.set_option(boost::asio::ip::multicast::enable_loopback(true));

    //out_socket.bind(udp::endpoint(address::from_string("127.0.0.1"), 6002));
    //socket.bind(udp::endpoint(address::from_string(addr), 30101));

    std::string message;
    for (int port : announce_ports)
    {
        std::cout << "Writing to port: " << port << std::endl;
        udp::endpoint destination(address::from_string(addr), port);
        //udp::endpoint destination(boost::asio::ip::address_v4::broadcast(), port);
        out_socket.async_send_to(boost::asio::buffer(out_buffer_, rv), destination, handler);
    }

}

void UDPLink::wait()
{
    cout << __FILE__ << ", " << __LINE__ << endl;
    socket.async_receive_from(boost::asio::buffer(recv_buffer),
                              remote_endpoint,
                              boost::bind(&UDPLink::handle_receive,
                                          this, boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));
    cout << __FILE__ << ", " << __LINE__ << endl;
}

void UDPLink::handle_receive(const boost::system::error_code& error, size_t bytes_transferred)
{
    if (error) {
        std::cout << "Receive failed: " << error.message() << "\n";
        return;
    }

    std::cout << "Received by udp: '" << std::string(recv_buffer.begin(), recv_buffer.begin()+bytes_transferred) << "' (" << error.message() << ")\n";

    for (size_t i = 0; i < bytes_transferred; ++i) {
        IMC::Message* m = parser_.parse(recv_buffer[i]);
        if (m) {
            recv_handler_(m);
            delete m;
        }
    }

    wait();
}
