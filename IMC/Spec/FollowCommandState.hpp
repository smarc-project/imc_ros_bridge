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

#ifndef IMC_FOLLOWCOMMANDSTATE_HPP_INCLUDED_
#define IMC_FOLLOWCOMMANDSTATE_HPP_INCLUDED_

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
#include <IMC/Spec/Command.hpp>

namespace IMC
{
  //! Follow Command State.
  class FollowCommandState: public Message
  {
  public:
    //! State.
    enum StateEnum
    {
      //! Waiting for first command.
      FC_WAIT = 1,
      //! Moving towards received command.
      FC_MOVING = 2,
      //! Speed command is zero.
      FC_STOPPED = 3,
      //! Command is out of safe bounds.
      FC_BAD_COMMAND = 4,
      //! Controlling system timed out.
      FC_TIMEOUT = 5
    };

    //! Controlling Source.
    uint16_t control_src;
    //! Controlling Entity.
    uint8_t control_ent;
    //! Command.
    InlineMessage<Command> command;
    //! State.
    uint8_t state;

    static uint16_t
    getIdStatic(void)
    {
      return 498;
    }

    static FollowCommandState*
    cast(Message* msg__)
    {
      return (FollowCommandState*)msg__;
    }

    FollowCommandState(void)
    {
      m_header.mgid = FollowCommandState::getIdStatic();
      clear();
      command.setParent(this);
    }

    FollowCommandState*
    clone(void) const
    {
      return new FollowCommandState(*this);
    }

    void
    clear(void)
    {
      control_src = 0;
      control_ent = 0;
      command.clear();
      state = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::FollowCommandState& other__ = static_cast<const FollowCommandState&>(msg__);
      if (control_src != other__.control_src) return false;
      if (control_ent != other__.control_ent) return false;
      if (command != other__.command) return false;
      if (state != other__.state) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(control_src, ptr__);
      ptr__ += IMC::serialize(control_ent, ptr__);
      ptr__ += command.serialize(ptr__);
      ptr__ += IMC::serialize(state, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(control_src, bfr__, size__);
      bfr__ += IMC::deserialize(control_ent, bfr__, size__);
      bfr__ += command.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(state, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(control_src, bfr__, size__);
      bfr__ += IMC::deserialize(control_ent, bfr__, size__);
      bfr__ += command.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::deserialize(state, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return FollowCommandState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "FollowCommandState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 4;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return command.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "control_src", control_src, nindent__);
      IMC::toJSON(os__, "control_ent", control_ent, nindent__);
      command.toJSON(os__, "command", nindent__);
      IMC::toJSON(os__, "state", state, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!command.isNull())
      {
        command.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!command.isNull())
      {
        command.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!command.isNull())
      {
        command.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!command.isNull())
      {
        command.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!command.isNull())
      {
        command.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
