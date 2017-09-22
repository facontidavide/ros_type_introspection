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

#ifndef ROSTypePARSER_H
#define ROSTypePARSER_H

#include <vector>
#include <map>
#include <boost/function.hpp>
#include <boost/utility/string_ref.hpp>
#include "ros_type_introspection/utils/stringtree.hpp"
#include "ros_type_introspection/utils/variant.hpp"
#include "ros_type_introspection/utils/vector_view.hpp"

namespace RosIntrospection{

/**
 * @brief SetWarningsOutput global function to set where warnings are streamed. By default is std::err.
 * You can disable it as follows:
 *
 *     std::ostream null_stream(nullptr);
 *     SetWarningsOutput(&null_stream);
 *
 * @param stream
 */
void SetWarningsOutput(std::ostream *stream);

typedef details::TreeElement<SString> StringTreeNode;
typedef details::Tree<SString> StringTree;

/**
 * @brief Description of a ROS type.
 */
class ROSType {
public:

  ROSType(){}

  ROSType(const std::string& name);

  /// Concatenation of msg_name and pkg_name.
  /// ex.: geometry_msgs/Pose[40]"
  const SString& baseName() const;

  /// ex.: geometry_msgs/Pose[40] -> "Pose"
  const SString& msgName()  const;

  /// ex.: geometry_msgs/Pose[40] -> "geometry_msgs"
  const SString& pkgName()  const;

  void setPkgName(const SString& new_pkg);

  /// True if the type is an array
  bool isArray() const;

  /// True if the type is ROS builtin
  bool isBuiltin() const;

  /// 1 if !is_array, -1 if is_array and array is
  /// variable length, otherwise length in name
  int  arraySize() const;

  /// If builtin, size of builtin, -1 means variable or undefined
  int typeSize() const;

  /// If type is builtin, returns the id.  BuiltinType::OTHER otherwise.
  BuiltinType typeID() const;

  bool operator==(const ROSType& other) const  {
    return this->baseName() == other.baseName();
  }

  bool operator<(const ROSType& other) const {
    return this->baseName() < other.baseName();
  }

  Variant deserializeFromBuffer(const nonstd::VectorView<uint8_t>& buffer, size_t& offset) const
  {
      if(!_deserialize_impl){ return Variant(); }
      else{
          return _deserialize_impl(buffer, offset);
      }
  }

protected:

  BuiltinType _id;
  int     _array_size;
  SString _base_name;
  SString _msg_name;
  SString _pkg_name;
  boost::function<Variant(const nonstd::VectorView<uint8_t>& buffer, size_t& offset)> _deserialize_impl;

};

// helper function to deserialize raw memory
template <typename T> inline void ReadFromBuffer( const nonstd::VectorView<uint8_t>& buffer, size_t& offset, T& destination)
{
  if ( offset + sizeof(T) > buffer.size() )
  {
    throw std::runtime_error("Buffer overrun in RosIntrospection::ReadFromBuffer");
  }
  destination =  (*( reinterpret_cast<const T*>( &(buffer.data()[offset]) ) ) );
  offset += sizeof(T);
}

template <> inline void ReadFromBuffer( const nonstd::VectorView<uint8_t>& buffer, size_t& offset, SString& destination)
{
  uint32_t string_size = 0;
  ReadFromBuffer( buffer, offset, string_size );

  if( offset + string_size > buffer.size())
  {
    throw std::runtime_error("Buffer overrun in RosIntrospection::ReadFromBuffer");
  }

  const char* buffer_ptr = reinterpret_cast<const char*>( &buffer[offset] );
  offset += string_size;

  destination = SString( buffer_ptr, string_size );
}

template <typename T> inline Variant ReadFromBuffer( const nonstd::VectorView<uint8_t>& buffer, size_t& offset)
{
  T destination;
  ReadFromBuffer(buffer, offset, destination);
  return Variant(destination);
}

inline Variant ReadFromBuffer(BuiltinType id, const nonstd::VectorView<uint8_t>& buffer, size_t& offset)
{
  switch(id)
  {
  case BOOL: return ReadFromBuffer<bool>(buffer,offset);
  case BYTE:
  case UINT8:  return ReadFromBuffer<uint8_t>(buffer,offset);
  case UINT16: return ReadFromBuffer<uint16_t>(buffer,offset);
  case UINT32: return ReadFromBuffer<uint32_t>(buffer,offset);
  case UINT64: return ReadFromBuffer<uint64_t>(buffer,offset);

  case INT8:   return ReadFromBuffer<int8_t>(buffer,offset);
  case INT16:  return ReadFromBuffer<int16_t>(buffer,offset);
  case INT32:  return ReadFromBuffer<int32_t>(buffer,offset);
  case INT64:  return ReadFromBuffer<int64_t>(buffer,offset);

  case FLOAT32:  return ReadFromBuffer<float>(buffer,offset);
  case FLOAT64:  return ReadFromBuffer<double>(buffer,offset);

  case TIME: {
    ros::Time tmp;
    ReadFromBuffer( buffer, offset, tmp.sec );
    ReadFromBuffer( buffer, offset, tmp.nsec );
    return tmp;
  }
  case DURATION: {
    ros::Duration tmp;
    ReadFromBuffer( buffer, offset, tmp.sec );
    ReadFromBuffer( buffer, offset, tmp.nsec );
    return tmp;
  }

  case STRING: {
    uint32_t string_size = 0;
    ReadFromBuffer( buffer, offset, string_size );
    if( offset + string_size > buffer.size()) {
      throw std::runtime_error("Buffer overrun");
    }
    Variant var_string(reinterpret_cast<const char*>( &buffer[offset] ), string_size  );
    offset += string_size;
    return var_string;
  }
  case OTHER: return -1;
  }
  throw std::runtime_error( "unsupported builtin type value");
}



/**
 * @brief A ROSMessage will contain one or more ROSField(s). Each field is little more
 * than a name / type pair.
 */
class ROSField {
public:

  ROSField(const std::string& definition );

  const SString&  name() const { return _name; }

  const ROSType&  type() const { return _type; }

  /// True if field is a constant in message definition
  bool isConstant() const {
    return _value.size() != 0;
  }

  /// If constant, value of field, else undefined
  const SString& value() const   { return _value; }

  friend class ROSMessage;

protected:
  SString _name;
  ROSType _type;
  SString _value;
};


class ROSMessage{
public:

  /// This constructor does most of the work in terms of parsing.
  /// It uses the message definition to extract fields and types.
  ROSMessage(const std::string& msg_def );

  /**
   * @brief Get field by index.
   */
  const ROSField& field(size_t index) const { return _fields[index]; }

  /**
   * @brief Vector of fields.
   * @return
   */
  const std::vector<ROSField>& fields() const { return _fields; }

  const ROSType& type() const { return _type; }

  void mutateType(const ROSType& new_type ) { _type = new_type; }

  void updateMissingPkgNames(const std::vector<const ROSType *> &all_types);

private:

  ROSType _type;
  std::vector<ROSField> _fields;
};

struct ROSMessageInfo
{
  StringTree tree;
  std::vector<ROSMessage> type_list;
};

inline BuiltinType toBuiltinType(const SString& s) {
  static std::map<SString, BuiltinType> string_to_builtin_map {
    { "bool", BOOL },
    { "byte", BYTE },
    { "char", CHAR },
    { "uint8", UINT8 },
    { "uint16", UINT16 },
    { "uint32", UINT32 },
    { "uint64", UINT64 },
    { "int8", INT8 },
    { "int16", INT16 },
    { "int32", INT32 },
    { "int64", INT64 },
    { "float32", FLOAT32 },
    { "float64", FLOAT64 },
    { "time", TIME },
    { "duration", DURATION },
    { "string", STRING },
  };
  const auto it = string_to_builtin_map.find(s);
  return (it != string_to_builtin_map.cend()) ? it->second : OTHER;
}


} // end namespace

#endif // ROSTypePARSER_H
