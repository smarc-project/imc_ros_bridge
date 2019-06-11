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

#ifndef IMC_ROS_BRIDGE_SERVER_H
#define IMC_ROS_BRIDGE_SERVER_H

#include <ros/ros.h>
#include <imc_tcp_link/imc_handle.h>

namespace ros_to_imc {

template <typename ROS_MSG, typename IMC_MSG>
bool convert(const ROS_MSG& ros_msg, IMC_MSG& imc_msg)
{
    static_assert(sizeof(IMC_MSG) == -1 || sizeof(ROS_MSG) == -1, "ERROR: You need to supply a convert specialization for the ROS -> IMC msg types provided");
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
    static_assert(sizeof(IMC_MSG) == -1 || sizeof(ROS_MSG) == -1, "ERROR: You need to supply a convert specialization for the IMC -> ROS msg types provided");
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
