//***************************************************************************
// Copyright 2017 OceanScan - Marine Systems & Technology, Lda.             *
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
// Automatically generated.                                                 *
//***************************************************************************
// IMC XML MD5: 522ff971d12877ebe15aff467ba253d4                            *
//***************************************************************************

#ifndef IMC_IRIDIUMMSGTXEXTENDED_HPP_INCLUDED_
#define IMC_IRIDIUMMSGTXEXTENDED_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <ostream>
#include <string>
#include <vector>

// IMC headers.
#include <IMC/Base/Config.hpp>
#include <IMC/Base/Message.hpp>
#include <IMC/Base/InlineMessage.hpp>
#include <IMC/Base/MessageList.hpp>
#include <IMC/Base/JSON.hpp>
#include <IMC/Base/Serialization.hpp>
#include <IMC/Spec/Enumerations.hpp>
#include <IMC/Spec/Bitfields.hpp>

namespace IMC
{
  //! Transmit Iridium Message (Extended).
  class IridiumMsgTxExtended: public Message
  {
  public:
    //! Request Identifier.
    uint16_t req_id;
    //! Time to live.
    uint16_t ttl;
    //! Expiration Time.
    uint32_t expiration;
    //! Destination Identifier.
    std::string destination;
    //! Data.
    std::vector<char> data;

    static uint16_t
    getIdStatic(void)
    {
      return 2005;
    }

    static IridiumMsgTxExtended*
    cast(Message* msg__)
    {
      return (IridiumMsgTxExtended*)msg__;
    }

    IridiumMsgTxExtended(void)
    {
      m_header.mgid = IridiumMsgTxExtended::getIdStatic();
      clear();
    }

    IridiumMsgTxExtended*
    clone(void) const
    {
      return new IridiumMsgTxExtended(*this);
    }

    void
    clear(void)
    {
      req_id = 0;
      ttl = 0;
      expiration = 0;
      destination.clear();
      data.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::IridiumMsgTxExtended& other__ = static_cast<const IridiumMsgTxExtended&>(msg__);
      if (req_id != other__.req_id) return false;
      if (ttl != other__.ttl) return false;
      if (expiration != other__.expiration) return false;
      if (destination != other__.destination) return false;
      if (data != other__.data) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(req_id, ptr__);
      ptr__ += IMC::serialize(ttl, ptr__);
      ptr__ += IMC::serialize(expiration, ptr__);
      ptr__ += IMC::serialize(destination, ptr__);
      ptr__ += IMC::serialize(data, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(req_id, bfr__, size__);
      bfr__ += IMC::deserialize(ttl, bfr__, size__);
      bfr__ += IMC::deserialize(expiration, bfr__, size__);
      bfr__ += IMC::deserialize(destination, bfr__, size__);
      bfr__ += IMC::deserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(req_id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ttl, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(expiration, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(destination, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return IridiumMsgTxExtended::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "IridiumMsgTxExtended";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 8;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(destination) + IMC::getSerializationSize(data);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "req_id", req_id, nindent__);
      IMC::toJSON(os__, "ttl", ttl, nindent__);
      IMC::toJSON(os__, "expiration", expiration, nindent__);
      IMC::toJSON(os__, "destination", destination, nindent__);
      IMC::toJSON(os__, "data", data, nindent__);
    }
  };
}

#endif
