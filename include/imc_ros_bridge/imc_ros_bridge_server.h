#include <imc_tcp_link/TcpLink.hpp>

namespace ros_to_imc {

template <typename ROS_MSG, typename IMC_MSG>
bool convert(const ROS_MSG& ros_msg, IMC_MSG& imc_msg)
{
    return false;
}

template <typename ROS_MSG, typename IMC_MSG>
class BridgeServer {

    ros::Subscriber ros_sub;
    ros_imc_broker::TcpLink& tcp_client_;

    ConversionServer(ros::NodeHandle& ros_node, ros_imc_broker::TcpLink& tcp_client_, const std::string& ros_topic, unsigned char uid=0) : tcp_client_(tcp_client_)
    {
        ros_sub = ros_node.subscribe(ros_topic, 10, &ConversionServer::conversion_callback, this);
    }

    void conversion_callback(const ROSMSG& ros_msg)
    {
        IMCMSG imc_msg;
        bool success = convert(ros_msg, imc_msg, uid);
        if (success) {
            tcp_client_.write(&imc_msg);
        }
        else {
            ROS_WARN("There was an error trying to convert imc type %s", imc_msg.getName());
        }
    }

};

} // namespace ros_to_imc

namespace imc_to_ros {

template <typename IMC_MSG, typename ROS_MSG>
bool convert(const IMC_MSG& imc_msg, ROS_MSG& ros_msg)
{
    return false;
}

template <typename IMC_MSG, typename ROS_MSG>
class BridgeServer {

    ros::Publisher ros_pub;
    unsigned char uid;

    ConversionServer(UavNode& uav_node, ros::NodeHandle& ros_node, const std::string& ros_topic, unsigned char uid=255) : uav_sub(uav_node), uid(uid)
    {
        ros_pub = ros_node.advertise<ROSMSG>(ros_topic, 10);
    }

    void conversion_callback(const ReceivedDataStructure& uav_msg) const
    {
        ROSMSG ros_msg;
        bool success = convert(uav_msg, ros_msg, uid);
        if (success) {
            ros_pub.publish(ros_msg);
        }
    }

}

} // namespace imc_to_ros
