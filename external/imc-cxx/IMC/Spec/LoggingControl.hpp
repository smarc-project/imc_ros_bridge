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

#ifndef IMC_LOGGINGCONTROL_HPP_INCLUDED_
#define IMC_LOGGINGCONTROL_HPP_INCLUDED_

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
  //! Logging Control.
  class LoggingControl: public Message
  {
  public:
    //! Control Operation.
    enum ControlOperationEnum
    {
      //! Request Start of Logging.
      COP_REQUEST_START = 0,
      //! Logging Started.
      COP_STARTED = 1,
      //! Request Logging Stop.
      COP_REQUEST_STOP = 2,
      //! Logging Stopped.
      COP_STOPPED = 3,
      //! Request Current Log Name.
      COP_REQUEST_CURRENT_NAME = 4,
      //! Current Log Name.
      COP_CURRENT_NAME = 5
    };

    //! Control Operation.
    uint8_t op;
    //! Log Label / Path.
    std::string name;

    static uint16_t
    getIdStatic(void)
    {
      return 102;
    }

    static LoggingControl*
    cast(Message* msg__)
    {
      return (LoggingControl*)msg__;
    }

    LoggingControl(void)
    {
      m_header.mgid = LoggingControl::getIdStatic();
      clear();
    }

    LoggingControl*
    clone(void) const
    {
      return new LoggingControl(*this);
    }

    void
    clear(void)
    {
      op = 0;
      name.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::LoggingControl& other__ = static_cast<const LoggingControl&>(msg__);
      if (op != other__.op) return false;
      if (name != other__.name) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(name, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(name, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(name, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return LoggingControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "LoggingControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(name);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "name", name, nindent__);
    }
  };
}

#endif
