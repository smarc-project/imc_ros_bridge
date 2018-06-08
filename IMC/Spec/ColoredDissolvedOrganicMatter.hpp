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

#ifndef IMC_COLOREDDISSOLVEDORGANICMATTER_HPP_INCLUDED_
#define IMC_COLOREDDISSOLVEDORGANICMATTER_HPP_INCLUDED_

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
  //! Colored Dissolved Organic Matter.
  class ColoredDissolvedOrganicMatter: public Message
  {
  public:
    //! Value.
    float value;

    static uint16_t
    getIdStatic(void)
    {
      return 2003;
    }

    static ColoredDissolvedOrganicMatter*
    cast(Message* msg__)
    {
      return (ColoredDissolvedOrganicMatter*)msg__;
    }

    ColoredDissolvedOrganicMatter(void)
    {
      m_header.mgid = ColoredDissolvedOrganicMatter::getIdStatic();
      clear();
    }

    ColoredDissolvedOrganicMatter*
    clone(void) const
    {
      return new ColoredDissolvedOrganicMatter(*this);
    }

    void
    clear(void)
    {
      value = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ColoredDissolvedOrganicMatter& other__ = static_cast<const ColoredDissolvedOrganicMatter&>(msg__);
      if (value != other__.value) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(value, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(value, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(value, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ColoredDissolvedOrganicMatter::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ColoredDissolvedOrganicMatter";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 4;
    }

    double
    getValueFP(void) const
    {
      return static_cast<double>(value);
    }

    void
    setValueFP(double val)
    {
      value = static_cast<float>(val);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "value", value, nindent__);
    }
  };
}

#endif
