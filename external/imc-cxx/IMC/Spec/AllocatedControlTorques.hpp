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

#ifndef IMC_ALLOCATEDCONTROLTORQUES_HPP_INCLUDED_
#define IMC_ALLOCATEDCONTROLTORQUES_HPP_INCLUDED_

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
  //! Allocated Control Torques.
  class AllocatedControlTorques: public Message
  {
  public:
    //! Torque about the x axis.
    double k;
    //! Torque about the y axis.
    double m;
    //! Torque about the x axis.
    double n;

    static uint16_t
    getIdStatic(void)
    {
      return 411;
    }

    static AllocatedControlTorques*
    cast(Message* msg__)
    {
      return (AllocatedControlTorques*)msg__;
    }

    AllocatedControlTorques(void)
    {
      m_header.mgid = AllocatedControlTorques::getIdStatic();
      clear();
    }

    AllocatedControlTorques*
    clone(void) const
    {
      return new AllocatedControlTorques(*this);
    }

    void
    clear(void)
    {
      k = 0;
      m = 0;
      n = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::AllocatedControlTorques& other__ = static_cast<const AllocatedControlTorques&>(msg__);
      if (k != other__.k) return false;
      if (m != other__.m) return false;
      if (n != other__.n) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(k, ptr__);
      ptr__ += IMC::serialize(m, ptr__);
      ptr__ += IMC::serialize(n, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(k, bfr__, size__);
      bfr__ += IMC::deserialize(m, bfr__, size__);
      bfr__ += IMC::deserialize(n, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(k, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(m, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(n, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return AllocatedControlTorques::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "AllocatedControlTorques";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 24;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "k", k, nindent__);
      IMC::toJSON(os__, "m", m, nindent__);
      IMC::toJSON(os__, "n", n, nindent__);
    }
  };
}

#endif
