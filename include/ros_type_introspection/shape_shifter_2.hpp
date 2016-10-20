/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef TOPIC_TOOLS_SHAPE_SHIFTER2_H
#define TOPIC_TOOLS_SHAPE_SHIFTER2_H

#include "ros/ros.h"
#include "ros/console.h"
#include "ros/assert.h"
#include <vector>
#include <boost/flyweight.hpp>
#include <ros/message_traits.h>
#include <ros_type_introspection/ros_introspection.hpp>
#include <topic_tools/shape_shifter.h>

namespace RosIntrospection
{


class ShapeShifter2
{
public:
  typedef boost::shared_ptr<ShapeShifter2> Ptr;
  typedef boost::shared_ptr<ShapeShifter2 const> ConstPtr;

  static bool uses_old_API_;

  // Constructor and destructor
  ShapeShifter2();
  virtual ~ShapeShifter2();

  // Helpers for inspecting ShapeShifter2
  std::string const& getDataType()          const;
  std::string const& getMD5Sum()            const;
  std::string const& getMessageDefinition() const;

  // Helper for advertising
  ros::Publisher advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size_,
                           bool latch=false,
                           const ros::SubscriberStatusCallback &connect_cb=ros::SubscriberStatusCallback()) const;

  //! Call to try instantiating as a particular type
  template<class M>
  boost::shared_ptr<M> instantiate() const;

  //! Write serialized message contents out to a stream
  template<typename Stream>
  void write(Stream& stream) const;

  const uint8_t* raw_data() const;

  template<typename Stream>
  void read(Stream& stream);

  // Directly serialize the content of a
  template<typename Message>
  void direct_read(const Message& msg,bool morph);

  //! Return the size of the serialized message
  uint32_t size() const;

  void morph(const std::string& md5sum, const std::string& datatype, const std::string& msg_def);

private:

  boost::flyweight<std::string> md5;
  boost::flyweight<std::string> datatype;
  boost::flyweight<std::string> msg_def;
  bool typed;

  std::vector<uint8_t> msgBuf;

};

}


// Message traits allow shape shifter to work with the new serialization API
namespace ros {
namespace message_traits {

template <> struct IsMessage<RosIntrospection::ShapeShifter2> : TrueType { };
template <> struct IsMessage<const RosIntrospection::ShapeShifter2> : TrueType { };

template<>
struct MD5Sum<RosIntrospection::ShapeShifter2>
{
  static const char* value(const RosIntrospection::ShapeShifter2& m) { return m.getMD5Sum().data(); }

  // Used statically, a ShapeShifter2 appears to be of any type
  static const char* value() { return "*"; }
};

template<>
struct DataType<RosIntrospection::ShapeShifter2>
{
  static const char* value(const RosIntrospection::ShapeShifter2& m) { return m.getDataType().data(); }

  // Used statically, a ShapeShifter2 appears to be of any type
  static const char* value() { return "*"; }
};

template<>
struct Definition<RosIntrospection::ShapeShifter2>
{
  static const char* value(const RosIntrospection::ShapeShifter2& m) { return m.getMessageDefinition().data(); }
};

} // namespace message_traits


namespace serialization
{

template<>
struct Serializer<RosIntrospection::ShapeShifter2>
{
  template<typename Stream>
  inline static void write(Stream& stream, const RosIntrospection::ShapeShifter2& m) {
    m.write(stream);
  }

  template<typename Stream>
  inline static void read(Stream& stream, RosIntrospection::ShapeShifter2& m)
  {
    m.read(stream);
  }

  inline static uint32_t serializedLength(const RosIntrospection::ShapeShifter2& m) {
    return m.size();
  }
};


template<>
struct PreDeserialize<RosIntrospection::ShapeShifter2>
{
  static void notify(const PreDeserializeParams<RosIntrospection::ShapeShifter2>& params)
  {
    std::string md5      = (*params.connection_header)["md5sum"];
    std::string datatype = (*params.connection_header)["type"];
    std::string msg_def  = (*params.connection_header)["message_definition"];

    params.message->morph(md5, datatype, msg_def);
  }
};

} // namespace serialization

} //namespace ros



// Template implementations:

namespace RosIntrospection
{

template<class M>
boost::shared_ptr<M> ShapeShifter2::instantiate() const
{
  if (!typed)
    throw topic_tools::ShapeShifterException("Tried to instantiate message from an untyped ShapeShifter2.");

  if (ros::message_traits::datatype<M>() != getDataType())
    throw topic_tools::ShapeShifterException("Tried to instantiate message without matching datatype.");

  if (ros::message_traits::md5sum<M>() != getMD5Sum())
    throw topic_tools::ShapeShifterException("Tried to instantiate message without matching md5sum.");

  boost::shared_ptr<M> p(boost::make_shared<M>());

  ros::serialization::IStream s(msgBuf.data(), msgBuf.size() );
  ros::serialization::deserialize(s, *p);

  return p;
}

template<typename Stream>
void ShapeShifter2::write(Stream& stream) const {
  if (msgBuf.size() > 0)
    memcpy(stream.advance(msgBuf.size()), msgBuf.data(), msgBuf.size());
}

const uint8_t* ShapeShifter2::raw_data() const {
  return msgBuf.data();
}

uint32_t ShapeShifter2::size() const
{
  return msgBuf.size();
}

template<typename Stream>
void ShapeShifter2::read(Stream& stream)
{
  //allocate enough space
  msgBuf.resize( stream.getLength() );
  //copy
  memcpy(msgBuf.data(), stream.getData(), stream.getLength());
}

template<typename Message>
void ShapeShifter2::direct_read(const Message& msg, bool do_morph)
{
  if(do_morph)
  {
    this->morph(
          ros::message_traits::MD5Sum<Message>::value(),
          ros::message_traits::DataType<Message>::value(),
          ros::message_traits::Definition<Message>::value());
  }

  auto length = ros::serialization::serializationLength(msg);

  //allocate enough space
  msgBuf.resize( length );
  //copy
  ros::serialization::OStream o_stream(msgBuf.data(), length);
  ros::serialization::serialize(o_stream, msg);
}

ShapeShifter2::ShapeShifter2()
  :  typed(false),
     msgBuf()
{
}


ShapeShifter2::~ShapeShifter2()
{

}


std::string const& ShapeShifter2::getDataType()          const { return datatype; }


std::string const& ShapeShifter2::getMD5Sum()            const { return md5;   }


std::string const& ShapeShifter2::getMessageDefinition() const { return msg_def;  }


void ShapeShifter2::morph(const std::string& _md5sum, const std::string& _datatype, const std::string& _msg_def)
{
  md5 = _md5sum;
  datatype = _datatype;
  msg_def = _msg_def;
  typed = md5 != "*";
}


ros::Publisher ShapeShifter2::advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size_, bool latch, const ros::SubscriberStatusCallback &connect_cb) const
{
  ros::AdvertiseOptions opts(topic, queue_size_, getMD5Sum(), getDataType(), getMessageDefinition(), connect_cb);
  opts.latch = latch;

  return nh.advertise(opts);
}

} // namespace topic_tools


#endif

