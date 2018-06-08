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

#ifndef IMC_COMMAND_HPP_INCLUDED_
#define IMC_COMMAND_HPP_INCLUDED_

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
  //! Command To Follow.
  class Command: public Message
  {
  public:
    //! Flags.
    enum FlagsBits
    {
      //! Use Speed Reference in meters per second.
      FLAG_SPEED_METERS_PS = 0x01,
      //! Use Speed Reference in revolutions per minute.
      FLAG_SPEED_RPM = 0x02,
      //! Use Z Reference as depth.
      FLAG_DEPTH = 0x04,
      //! Use Z Reference as altitude.
      FLAG_ALTITUDE = 0x08,
      //! Use Heading Reference.
      FLAG_HEADING = 0x10,
      //! Use Heading Rate Reference.
      FLAG_HEADING_RATE = 0x20,
      //! Flag Maneuver Completion.
      FLAG_MANDONE = 0x80
    };

    //! Flags.
    uint8_t flags;
    //! Speed Reference.
    float speed;
    //! Z Reference.
    float z;
    //! Heading Reference.
    float heading;

    static uint16_t
    getIdStatic(void)
    {
      return 497;
    }

    static Command*
    cast(Message* msg__)
    {
      return (Command*)msg__;
    }

    Command(void)
    {
      m_header.mgid = Command::getIdStatic();
      clear();
    }

    Command*
    clone(void) const
    {
      return new Command(*this);
    }

    void
    clear(void)
    {
      flags = 0;
      speed = 0;
      z = 0;
      heading = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::Command& other__ = static_cast<const Command&>(msg__);
      if (flags != other__.flags) return false;
      if (speed != other__.speed) return false;
      if (z != other__.z) return false;
      if (heading != other__.heading) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(flags, ptr__);
      ptr__ += IMC::serialize(speed, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      ptr__ += IMC::serialize(heading, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      bfr__ += IMC::deserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(heading, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(heading, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return Command::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "Command";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 13;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "flags", flags, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
      IMC::toJSON(os__, "heading", heading, nindent__);
    }
  };
}

#endif
