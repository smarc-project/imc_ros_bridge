#include <imc_udp_link/udp_link.h>

#include <boost/lexical_cast.hpp>
#include <ros/ros.h>

using namespace std;

UDPLink::UDPLink(std::function<void (IMC::Message*)> recv_handler,
        const std::string& addr, const std::string& port)
    : recv_handler_(recv_handler), addr(addr), port(port)
{
    socket.open(udp::v4());
    socket.set_option(udp::socket::reuse_address(true));
    socket.bind(udp::endpoint(address::from_string(addr), boost::lexical_cast<int>(port)));

    multicast_socket.open(boost::asio::ip::udp::v4());

    wait();

    //auto work = std::make_shared<boost::asio::io_service::work>(io_service);

    run_thread = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service)); 
}

UDPLink::~UDPLink()
{
    //multicast_socket.shutdown();
    multicast_socket.close();
    run_thread.join();
}

void handler(const boost::system::error_code& error, size_t bytes_transferred)
{
}


void UDPLink::publish(IMC::Message& msg)
{
    msg.setSource(30);
    msg.setDestination(0);
    msg.setTimeStamp(ros::Time::now().toSec());

    char out_buffer_[1024];
    uint16_t rv = IMC::Packet::serialize(&msg, (uint8_t*)out_buffer_, sizeof(out_buffer_));

    udp::endpoint destination(address::from_string("127.0.0.1"), 6001);
    socket.async_send_to(boost::asio::buffer(out_buffer_, rv), destination, handler);
}

void UDPLink::publish_multicast(IMC::Message& msg, const string& multicast_addr)
{
    msg.setSource(30);
    msg.setDestination(0);
    msg.setTimeStamp(ros::Time::now().toSec());

    char out_buffer_[1024];
    uint16_t rv = IMC::Packet::serialize(&msg, (uint8_t*)out_buffer_, sizeof(out_buffer_));

    std::string message;
    for (int multicast_port : announce_ports)
    {
        std::cout << "Writing to port: " << multicast_port << std::endl;
        udp::endpoint destination(address::from_string(multicast_addr), multicast_port);
        multicast_socket.async_send_to(boost::asio::buffer(out_buffer_, rv), destination, handler);
    }

}

void UDPLink::wait()
{
    socket.async_receive_from(boost::asio::buffer(recv_buffer),
                              remote_endpoint,
                              boost::bind(&UDPLink::handle_receive,
                                          this, boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));
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
