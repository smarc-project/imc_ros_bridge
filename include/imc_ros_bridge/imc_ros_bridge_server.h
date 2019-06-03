#ifndef IMC_ROS_BRIDGE_SERVER_H
#define IMC_ROS_BRIDGE_SERVER_H

#include <ros/ros.h>
#include <imc_tcp_link/imc_handle.h>

namespace ros_to_imc {

template <typename ROS_MSG, typename IMC_MSG>
bool convert(const ROS_MSG& ros_msg, IMC_MSG& imc_msg)
{
    return false;
}

template <typename ROS_MSG, typename IMC_MSG>
class BridgeServer {

private:

    ros::Subscriber ros_sub;
    //ros_imc_broker::TcpLink& tcp_client_;
    IMCHandle& imc_handle;

public:

    BridgeServer(ros::NodeHandle& ros_node, IMCHandle& imc_handle, const std::string& ros_topic) : imc_handle(imc_handle)
    {
        ros_sub = ros_node.subscribe(ros_topic, 10, &BridgeServer::conversion_callback, this);
    }

    void conversion_callback(const ROS_MSG& ros_msg)
    {
        IMC_MSG imc_msg;
        bool success = convert(ros_msg, imc_msg);
        if (success) {
            //tcp_client_.write(&imc_msg);
            imc_handle.write(imc_msg);
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
    std::cout << "Default convert !" << std::endl;
    return false;
}

template <typename IMC_MSG, typename ROS_MSG>
class BridgeServer {

private:

    ros::Publisher ros_pub;

public:

    BridgeServer(IMCHandle& imc_handle, ros::NodeHandle& ros_node, const std::string& ros_topic)
    {
        ros_pub = ros_node.advertise<ROS_MSG>(ros_topic, 10);
        imc_handle.tcp_subscribe(IMC_MSG::getIdStatic(), std::bind(&BridgeServer::conversion_callback, this, std::placeholders::_1));
    }

    void conversion_callback(const IMC::Message* imc_msg) const
    {
        ROS_MSG ros_msg;
        bool success = convert(static_cast<const IMC_MSG&>(*imc_msg), ros_msg);
        if (success) {
            ros_pub.publish(ros_msg);
        }
        else {
            ROS_WARN("There was an error trying to convert imc type %s", imc_msg->getName());
        }
    }

};

} // namespace imc_to_ros

#endif // IMC_ROS_BRIDGE_SERVER_H
