/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright 2016 Davide Faconti
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#ifndef ROS_MESSAGE_PART_EXTRACT_H
#define ROS_MESSAGE_PART_EXTRACT_H

#include "ros_type_introspection/deserializer.hpp"


namespace RosIntrospection{


template <typename RM> inline
void extractSpecificROSMessagesImpl(const ROSTypeList& type_list,
                                ROSType type,
                                const SString& prefix,
                                const nonstd::VectorView<uint8_t>& buffer,
                                size_t& buffer_offset,
                                std::vector< std::pair<SString,RM> >& destination)
{
  static_assert( ros::message_traits::IsMessage<RM>::value,
                 "The template type must be a ROS message");

  int32_t array_size = type.arraySize();
  if( array_size == -1)
  {
    ReadFromBuffer( buffer, buffer_offset, array_size );
  }

  //---------------------------------------------------------------------------
  // we store in a function pointer the operation to be done later
  // This operation is different according to the typeID
  for (int v=0; v<array_size; v++)
  {
    if( strcmp( type.baseName().data(), ros::message_traits::DataType<RM>::value()) == 0)
    {
      RM msg;
      ros::serialization::IStream s( (uint8_t*)&buffer[buffer_offset], buffer.size() - buffer_offset );
      ros::serialization::deserialize(s, msg);
      buffer_offset += ros::serialization::serializationLength(msg);
      destination.push_back( std::make_pair(prefix, std::move(msg) ) );
    }
    else if( type.isBuiltin())
    {
      // must do this even if STORE_RESULT==false to increment buffer_offset
      Variant temp = type.deserializeFromBuffer(buffer, buffer_offset);
    }
    else if( type.typeID() == OTHER)
    {
      const ROSMessageDefinition* mg_definition = nullptr;

      for(const ROSMessageDefinition& msg: type_list) // find in the list
      {
        if( msg.type().msgName() == type.msgName() &&
            msg.type().pkgName() == type.pkgName()  )
        {
          mg_definition = &msg;
          break;
        }
      }
      if( !mg_definition )
      {
        std::string output( "can't deserialize this stuff: ");
        output +=  type.baseName().toStdString() + "\n\n";
        output +=  "Available types are: \n\n";
        for(const ROSMessageDefinition& msg: type_list) // find in the list
        {
          output += "   " +msg.type().baseName().toStdString() + "\n";
        }
        throw std::runtime_error( output );
      }

      for (const ROSField& field : mg_definition->fields() )
      {
        if(field.isConstant() == false)
        {
          SString new_prefix(prefix);
          new_prefix.append("/",1).append( field.name() ) ;
          extractSpecificROSMessagesImpl(type_list,  field.type(),
                                     new_prefix ,
                                     buffer, buffer_offset,
                                     destination);
        }
      }
    }
    else {
      throw std::runtime_error( "can't deserialize this stuff");
    }
  }
}

/**
 * This is a less generic version of buildRosFlatType which extract only part of the message
 * In particular those parts which match the typename RM
 *
 * Example usage:
 *
 *   std::vector< std::pair<SString,std_msgs::Header>> headers;
 *
 *   ExtractSpecificROSMessages(type_map,  // map obtained using buildROSTypeMapFromDefinition
 *                              main_type, // ROSType obtained from sensor_msgs::JointState
 *                              "JointState",
 *                              buffer,    //buffer with the raw version of a message of type sensor_msgs::JointState
 *                              headers);  // output vector
 *
 *  // headers will contain one element. All the other fields in sensor_msgs::JointState
 */
template <typename RM> inline
void ExtractSpecificROSMessages(const ROSTypeList& type_list,
                                ROSType type,
                                const SString& prefix,
                                const nonstd::VectorView<uint8_t>& buffer,
                                std::vector< std::pair<SString,RM> >& destination)
{
  bool found = false;
  for(const ROSMessageDefinition& msg: type_list) // find in the list
  {
    if( strcmp( msg.type().baseName().data(), ros::message_traits::DataType<RM>::value()) == 0)
    {
      found = true;
      break;
    }
  }
  if(!found){
    throw std::runtime_error("extractSpecificROSMessages: ROSTypeList does not contain the type you are trying to extract");
  }

  size_t offset = 0;
  extractSpecificROSMessagesImpl(type_list, type, prefix,
                                 buffer, offset, destination);
  if( offset != buffer.size() )
  {
    throw std::runtime_error("extractSpecificROSMessages: There was an error parsing the buffer" );
  }
}


} //end namespace

#endif
