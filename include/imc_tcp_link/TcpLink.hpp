//***************************************************************************
// Copyright 2016 OceanScan - Marine Systems & Technology, Lda.             *
//***************************************************************************
// Licensed under the Apache License, Version 2.0 (the "License");          *
// you may not use this file except in compliance with the License.         *
// You may obtain a copy of the License at                                  *
//                                                                          *
// http://www.apache.org/licenses/LICENSE-2.0                               *
//                                                                          *
// Unless required by applicable law or agreed to in writing, software      *
// distributed under the License is distributed on an "AS IS" BASIS,        *
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
// See the License for the specific language governing permissions and      *
// limitations under the License.                                           *
//***************************************************************************
// Author: Ricardo Martins                                                  *
//***************************************************************************

#ifndef ROS_IMC_BROKER_TCP_LINK_HPP_INCLUDED_
#define ROS_IMC_BROKER_TCP_LINK_HPP_INCLUDED_

// ISO C++ headers
#include <cstdlib>
#include <cstring>
#include <iostream>

// Boost headers.
#include <boost/asio.hpp>

// IMC headers.
#include <IMC/Base/Parser.hpp>

// ROS headers.
#include <ros/ros.h>

namespace ros_imc_broker
{
  //! TCP link to DUNE.
  //
  //! This class implements a TCP client that connects to DUNE's backseat
  //! TCP server.
  //!
  //! @author Ricardo Martins <rasm@oceanscan-mst.com>
  class TcpLink
  {
  public:
    //! Constructor.
    //! @param[in] recv_handler handler function for received messages.
    TcpLink(boost::function<void (IMC::Message*)> recv_handler):
      socket_(io_service_),
      recv_handler_(recv_handler),
      connected_(false)
    { }

    ~TcpLink(void)
    { }

    void
    setServer(const std::string& addr, const std::string& port)
    {
      addr_ = addr;
      port_ = port;
    }

    void
    read(void)
    {
      size_t rv = socket_.read_some(boost::asio::buffer(in_buffer_, sizeof(in_buffer_)));
      for (size_t i = 0; i < rv; ++i)
      {
        IMC::Message* m = parser_.parse((uint8_t)in_buffer_[i]);
        if (m)
        {
          recv_handler_(m);
          delete m;
        }
      }
    }

    void
    write(const IMC::Message* message)
    {
      uint16_t rv = IMC::Packet::serialize(message, (uint8_t*)out_buffer_, sizeof(out_buffer_));

      try
      {
        boost::asio::write(socket_, boost::asio::buffer(out_buffer_, rv));
      }
      catch (std::exception& e)
      {
        connected_ = false;
        ROS_WARN("link closed: %s", e.what());
      }
    }

    void
    operator()()
    {
      ROS_INFO("IMC listener started");

      while (true)
      {
        if (isConnected())
        {
          try
          {
            read();
          }
          catch (std::exception& e)
          {
            connected_ = false;
            ROS_WARN("link closed: %s", e.what());
          }
        }
        else
        {
          try
          {
            connect();
          }
          catch (std::exception& e)
          {
            ROS_INFO("%s", e.what());
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
          }
        }

        boost::this_thread::interruption_point();
      }
    }

  private:
    //! ASIO I/O service.
    boost::asio::io_service io_service_;
    //! Callback for received IMC messages.
    boost::function<void (IMC::Message*)> recv_handler_;
    //! TCP socket.
    boost::asio::ip::tcp::socket socket_;
    //! Incoming data buffer.
    char in_buffer_[1024];
    //! Outgoing data buffer.
    char out_buffer_[1024];
    //! IMC message parser.
    IMC::Parser parser_;
    std::string addr_;
    std::string port_;
    bool connected_;

    bool
    isConnected(void)
    {
      return connected_;
    }

    void
    connect(void)
    {
      ROS_INFO("connecting to %s:%s", addr_.c_str(), port_.c_str());

      boost::asio::ip::tcp::resolver resolver(io_service_);
      boost::asio::ip::tcp::resolver::query query(boost::asio::ip::tcp::v4(), addr_.c_str(), port_.c_str());
      boost::asio::ip::tcp::resolver::iterator iterator = resolver.resolve(query);
      boost::asio::connect(socket_, iterator);
      connected_ = true;

      ROS_INFO("connected to %s:%s", addr_.c_str(), port_.c_str());
    }

  };
}

#endif
