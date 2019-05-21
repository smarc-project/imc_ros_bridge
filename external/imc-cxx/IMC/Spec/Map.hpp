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

#ifndef IMC_MAP_HPP_INCLUDED_
#define IMC_MAP_HPP_INCLUDED_

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
#include <IMC/Spec/MapFeature.hpp>

namespace IMC
{
  //! Map.
  class Map: public Message
  {
  public:
    //! Identifier.
    std::string id;
    //! Features.
    MessageList<MapFeature> features;

    static uint16_t
    getIdStatic(void)
    {
      return 602;
    }

    static Map*
    cast(Message* msg__)
    {
      return (Map*)msg__;
    }

    Map(void)
    {
      m_header.mgid = Map::getIdStatic();
      clear();
      features.setParent(this);
    }

    Map*
    clone(void) const
    {
      return new Map(*this);
    }

    void
    clear(void)
    {
      id.clear();
      features.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::Map& other__ = static_cast<const Map&>(msg__);
      if (id != other__.id) return false;
      if (features != other__.features) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(id, ptr__);
      ptr__ += features.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(id, bfr__, size__);
      bfr__ += features.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(id, bfr__, size__);
      bfr__ += features.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return Map::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "Map";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(id) + features.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "id", id, nindent__);
      features.toJSON(os__, "features", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      features.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      features.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      features.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      features.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      features.setDestinationEntity(value__);
    }
  };
}

#endif
