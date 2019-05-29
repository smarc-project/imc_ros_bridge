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

    std::cout << "Receiving\n";
    io_service.run();
    std::cout << "Receiver exit\n";
}

UDPLink::~UDPLink() {}

void UDPLink::announce(const string& addr)
{
    IMC::Announce msg;

    boost::asio::io_service service;
    udp::socket socket(service);
    socket.open(boost::asio::ip::udp::v4());

    // Allow other processes to reuse the address, permitting other processes on
//  // the same machine to use the multicast address.
    socket.set_option(udp::socket::reuse_address(true));
    // Guarantee the loopback is enabled so that multiple processes on the same
    // machine can receive data that originates from the same socket.
    socket.set_option(boost::asio::ip::multicast::enable_loopback(true));
    socket.bind(udp::endpoint(address::from_string("127.0.0.1"),
                              receiver ? port /* same as multicast port */
                              : 0 /* any */));

    udp::endpoint destination(address, port);
    
    std::string message;
    for (unsigned int i=0; i < 3; ++i)
    {
        std::ostringstream stream;
        stream << i;
        message = stream.str();
        socket.send_to(boost::asio::buffer(message), destination);
        std::cout << "Sent message: " << message << std::endl;
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
