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
#include <boost/utility/string_ref.hpp>
#include <ros_type_introspection/string.hpp>
#include <ros_type_introspection/stringtree.hpp>

namespace RosIntrospection{


#if 1
// Faster, but might need more testing
typedef ssoX::basic_string< 31, char> SString;
#else
// slightly slower but safer option. More convenient during debug
typedef std::string SString;
#endif

typedef details::TreeElement<SString> StringTreeNode;
typedef details::Tree<SString> StringTree;

enum BuiltinType {
  BOOL , BYTE, CHAR,
  UINT8, UINT16, UINT32, UINT64,
  INT8, INT16, INT32, INT64,
  FLOAT32, FLOAT64,
  TIME, DURATION,
  STRING, OTHER
};

inline std::ostream& operator<<(std::ostream& os, const BuiltinType& c)
{
  static const char* names[] =
  {
    "BOOL" , "BYTE", "CHAR",
    "UINT8", "UINT16", "UINT32", "UINT64",
    "INT8", "INT16", "INT32", "INT64",
    "FLOAT32", "FLOAT64",
    "TIME", "DURATION",
    "STRING", "OTHER"
  };

  os << names[ static_cast<int>(c) ];
  return os;
}



const int BuiltinTypeSize[OTHER] = {
  1, 1, 1,
  1, 2, 4, 8,
  1, 2, 4, 8,
  4, 8,
  8, 8,
  -1
};

/**
 * @brief Description of a ROS type.
 */
class ROSType {
public:

  ROSType(){}

  ROSType(const std::string& name);

  /// Concatenation of msg_name and pkg_name.
  /// ex.: geometry_msgs/Pose[40]"
  const std::string& baseName() const;

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

protected:

  BuiltinType _id;
  int         _array_size;
  std::string _base_name;
  SString _msg_name;
  SString _pkg_name;

};

/**
 * @brief A ROSMessage will contain one or more ROSField(s). Each field is little more
 * than a name / type pair.
 */
class ROSField {
public:
  ROSField(const std::string& name, const ROSType& type ):
    _name( name ), _type( type ) {}

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

class ROSMessage;
typedef std::vector<ROSMessage> ROSTypeList;



class ROSMessage{
public:

  /// This constructor does most of the work in terms of parsing.
  /// It uses the message definition to extract fields and types.
  ROSMessage(const std::string& msg_def );

  /**
   Sometimes the whole type information is incomplete, in particular
   ROSTYPE::pkgName(). This method helps the application to "fill the blancks".
   Used internally by buildROSTypeMapFromDefinition, the user should probably
   ignore it.
   */
  void updateTypes(const std::vector<ROSType> &all_types);

  /**
   * @brief Get field by name.
   * it uses linear search, so you should use it for debug only.
   */
  const ROSField* field(const SString& name) const;

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

private:
  ROSType _type;
  std::vector<ROSField> _fields;
};


inline const ROSField* ROSMessage::field(const SString &name) const
{
  for(int i=0; i<_fields.size(); i++ )  {
    if(  name ==_fields[i].name() ) {
      return &_fields[i];
    }
  }
  return nullptr;
}


//------------------------------

/**
 * @brief A single message definition will (most probably) generate myltiple ROSMessage(s).
 * In fact the "child" ROSTypes are parsed as well in a recursive and hierarchical way.
 * To make an example, given as input the [geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html)
 * the result will be a ROSTypeList containing Pose, Point and Quaternion.
 *
 * @param type_name name to give to the main type to be extracted.
 *
 * @param msg_definition text obtained by either:
 *                       - topic_tools::ShapeShifter::getMessageDefinition()
 *                       - rosbag::MessageInstance::getMessageDefinition()
 *                       - ros::message_traits::Definition< __your_type__ >::value()
 *
 * @return list od ROSMessages extracted by the main type its dependencies.
 */
ROSTypeList buildROSTypeMapFromDefinition(
    const std::string& type_name,
    const std::string& msg_definition);

std::ostream& operator<<(std::ostream& s, const ROSTypeList& c);


} // end namespace

#endif // ROSTypePARSER_H
