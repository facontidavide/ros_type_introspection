#ifndef ROS_BUILTIN_TYPES_HPP
#define ROS_BUILTIN_TYPES_HPP

#include <stdint.h>
#include <string>
#include <ros/ros.h>

namespace RosIntrospection{


enum BuiltinType {
  BOOL , BYTE, CHAR,
  UINT8, UINT16, UINT32, UINT64,
  INT8, INT16, INT32, INT64,
  FLOAT32, FLOAT64,
  TIME, DURATION,
  STRING, OTHER
};

inline const char* toStr(const BuiltinType& c)
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

  return names[ static_cast<int>(c) ];
}

inline std::ostream& operator<<(std::ostream& os, const BuiltinType& c)
{
  os << toStr(c);
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

template <typename T> BuiltinType getType()
{
    return OTHER;
}

template <> inline BuiltinType getType<bool>()  {  return BOOL; }

template <> inline BuiltinType getType<int8_t>()  {  return INT8; }
template <> inline BuiltinType getType<int16_t>() {  return INT16; }
template <> inline BuiltinType getType<int32_t>() {  return INT32; }
template <> inline BuiltinType getType<int64_t>() {  return INT64; }

template <> inline BuiltinType getType<uint8_t>()  {  return UINT8; }
template <> inline BuiltinType getType<uint16_t>() {  return UINT16; }
template <> inline BuiltinType getType<uint32_t>() {  return UINT32; }
template <> inline BuiltinType getType<uint64_t>() {  return UINT64; }

template <> inline BuiltinType getType<float>()  {  return FLOAT32; }
template <> inline BuiltinType getType<double>() {  return FLOAT64; }

template <> inline BuiltinType getType<std::string>() {  return STRING; }

template <> inline BuiltinType getType<ros::Time>()     {  return TIME; }
template <> inline BuiltinType getType<ros::Duration>() {  return DURATION; }

}

#endif // ROS_BUILTIN_TYPES_HPP
