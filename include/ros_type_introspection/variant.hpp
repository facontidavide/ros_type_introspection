#ifndef VARIANT_H
#define VARIANT_H

#include <type_traits>
#include <limits>
#include "ros_type_introspection/builtin_types.hpp"
#include "ros_type_introspection/details/exceptions.hpp"
#include "ros_type_introspection/details/conversion_impl.hpp"
#include <boost/any.hpp>

#if 1
// Faster, but might need more testing
typedef ssoX::basic_string<char> SString;
#else
// slightly slower but safer option. More convenient during debug
typedef std::string SString;
#endif

namespace RosIntrospection
{

class Variant
{

public:

  Variant() {
    _type = OTHER;
  }
   
// // 
  template<typename T> Variant(T value);

  BuiltinType getTypeID() const;
  
  template<typename T> T convert( ) const;
  
  boost::any auto_convert() const;

  template<typename T> T extract( ) const;

  template <typename T> void assign(const T& value);

private:
  std::vector<uint8_t> _raw_data;
  uint8_t  _type;

};

template <typename T> inline
bool operator ==(const Variant& var, const T& num)
{
  return var.convert<T>() == num;
}

template <typename T> inline
bool operator ==(const T& num, const Variant& var)
{
  return var.convert<T>() == num;
}

//----------------------- Implementation ----------------------------------------------


template<typename T>
inline Variant::Variant(T value) {
  static_assert (std::numeric_limits<T>::is_specialized ||
                 std::is_same<T, ros::Time>::value ||
                 std::is_same<T, ros::Duration>::value ||
                 std::is_same<T, SString>::value
                 , "not a valid type");
  assign(value);
}

inline BuiltinType Variant::getTypeID() const {
  return static_cast<BuiltinType>(_type);
}


template<typename T>
inline T Variant::extract( ) const
{
  static_assert (std::numeric_limits<T>::is_specialized ||
                 std::is_same<T, ros::Time>::value ||
                 std::is_same<T, ros::Duration>::value ||
                 std::is_same<T, SString>::value
                 , "not a valid type");
  if( getTypeID() != RosIntrospection::getType<T>() )
  {
    throw TypeException("Variant::extract -> wrong type");
  } 
  return * reinterpret_cast<const T*>( &_raw_data [0]);
}

template <>
inline void Variant::assign(const SString& value)
{
  //No assert necessary here as we know data is a SString
  _raw_data.resize(sizeof(value)/sizeof(uint8_t));
  *reinterpret_cast<SString *>( &_raw_data[0] ) =  value;
  _type = STRING ;
}

template <typename T>
inline void Variant::assign(const T& value)
{
  static_assert (std::numeric_limits<T>::is_specialized ||
                 std::is_same<T, ros::Time>::value ||
                 std::is_same<T, ros::Duration>::value ||
                 std::is_same<T, SString>::value
                 , "not a valid type");

  _raw_data.resize(sizeof(T));
  *reinterpret_cast<T *>( &_raw_data [0]) =  value;
  _type = RosIntrospection::getType<T>() ;
}

inline boost::any Variant::auto_convert() const
{
  using namespace RosIntrospection::details;
  boost::any target;
  //------
    switch( _type )
  {
    
  case CHAR:
  case INT8:   int64_t tmp1; convert_impl<int8_t,  int64_t>(*reinterpret_cast<const int8_t*>( &_raw_data [0]), tmp1 ); target = tmp1; break;

  case INT16:  int64_t tmp2; convert_impl<int16_t, int64_t>(*reinterpret_cast<const int16_t*>( &_raw_data [0]), tmp2  ); target = tmp2;break;
  case INT32:  int64_t tmp3; convert_impl<int32_t, int64_t>(*reinterpret_cast<const int32_t*>( &_raw_data [0]), tmp3  ); target = tmp3;break;
  case INT64:  int64_t tmp4; convert_impl<int64_t, int64_t>(*reinterpret_cast<const int64_t*>( &_raw_data [0]), tmp4  ); target = tmp4;break;

  case BOOL:
  case BYTE:
  case UINT8:   uint64_t tmp5; convert_impl<uint8_t,  uint64_t>(*reinterpret_cast<const uint8_t*>( &_raw_data [0]), tmp5  ); target = tmp5;break;

  case UINT16:  uint64_t tmp6; convert_impl<uint16_t, uint64_t>(*reinterpret_cast<const uint16_t*>( &_raw_data [0]), tmp6  ); target = tmp6;break;
  case UINT32:  uint64_t tmp7; convert_impl<uint32_t, uint64_t>(*reinterpret_cast<const uint32_t*>( &_raw_data [0]), tmp7  ); target = tmp7;break;
  case UINT64:  uint64_t tmp8; convert_impl<uint64_t, uint64_t>(*reinterpret_cast<const uint64_t*>( &_raw_data [0]), tmp8  ); target = tmp8;break;

  case FLOAT32:  double tmp9; convert_impl<float, double>(*reinterpret_cast<const float*>( &_raw_data [0]), tmp9  ); target = tmp9;break;
  case FLOAT64:  double tmp10; convert_impl<double, double>(*reinterpret_cast<const double*>( &_raw_data [0]), tmp10  ); target = tmp10;break;
  
  case DURATION: target = extract<ros::Duration>().toSec(); break;
  case TIME: target = extract<ros::Time>().toSec(); break;

  case STRING: target = extract<SString>(); break;
  
  default:  throw TypeException("Variant::convert -> cannot convert type" + std::to_string(_type)); break;

  }
  return  target;
  
}

template<typename DST> inline DST Variant::convert() const
{
  using namespace RosIntrospection::details;
  DST target;

  //----------
  switch( _type )
  {
    
  case CHAR:
  case INT8:   convert_impl<int8_t,  DST>(*reinterpret_cast<const int8_t*>( &_raw_data [0]), target  ); break;

  case INT16:  convert_impl<int16_t, DST>(*reinterpret_cast<const int16_t*>( &_raw_data [0]), target  ); break;
  case INT32:  convert_impl<int32_t, DST>(*reinterpret_cast<const int32_t*>( &_raw_data [0]), target  ); break;
  case INT64:  convert_impl<int64_t, DST>(*reinterpret_cast<const int64_t*>( &_raw_data [0]), target  ); break;

  case BOOL:
  case BYTE:
  case UINT8:   convert_impl<uint8_t,  DST>(*reinterpret_cast<const uint8_t*>( &_raw_data [0]), target  ); break;

  case UINT16:  convert_impl<uint16_t, DST>(*reinterpret_cast<const uint16_t*>( &_raw_data [0]), target  ); break;
  case UINT32:  convert_impl<uint32_t, DST>(*reinterpret_cast<const uint32_t*>( &_raw_data [0]), target  ); break;
  case UINT64:  convert_impl<uint64_t, DST>(*reinterpret_cast<const uint64_t*>( &_raw_data [0]), target  ); break;

  case FLOAT32:  convert_impl<float, DST>(*reinterpret_cast<const float*>( &_raw_data [0]), target  ); break;
  case FLOAT64:  convert_impl<double, DST>(*reinterpret_cast<const double*>( &_raw_data [0]), target  ); break;
  
  case DURATION:
  case TIME: {
     throw TypeException("ros::Duration and ros::Time can be converted only to double (will be seconds)");
  } break;

  default:  throw TypeException("Variant::convert -> cannot convert type" + std::to_string(_type)); break;

  }
  return  target;
}

template<> inline double Variant::convert() const
{
  using namespace RosIntrospection::details;
  double target;  
  //----------
  switch( _type )
  {
  case CHAR:
  case INT8:   convert_impl<int8_t,  double>(*reinterpret_cast<const int8_t*>( &_raw_data [0]), target  ); break;

  case INT16:  convert_impl<int16_t, double>(*reinterpret_cast<const int16_t*>( &_raw_data [0]), target  ); break;
  case INT32:  convert_impl<int32_t, double>(*reinterpret_cast<const int32_t*>( &_raw_data [0]), target  ); break;
  case INT64:  convert_impl<int64_t, double>(*reinterpret_cast<const int64_t*>( &_raw_data [0]), target  ); break;

  case BOOL:
  case BYTE:
  case UINT8:   convert_impl<uint8_t,  double>(*reinterpret_cast<const uint8_t*>( &_raw_data [0]), target  ); break;

  case UINT16:  convert_impl<uint16_t, double>(*reinterpret_cast<const uint16_t*>( &_raw_data [0]), target  ); break;
  case UINT32:  convert_impl<uint32_t, double>(*reinterpret_cast<const uint32_t*>( &_raw_data [0]), target  ); break;
  case UINT64:  convert_impl<uint64_t, double>(*reinterpret_cast<const uint64_t*>( &_raw_data [0]), target  ); break;

  case FLOAT32:  convert_impl<float, double>(*reinterpret_cast<const float*>( &_raw_data [0]), target  ); break;
  case FLOAT64:  convert_impl<double, double>(*reinterpret_cast<const double*>( &_raw_data [0]), target  ); break;

  case DURATION: {
    ros::Duration tmp =  extract<ros::Duration>();
    target = tmp.toSec();
  }break;
  case TIME: {
    ros::Time tmp =  extract<ros::Time>();
    target = tmp.toSec();
  }break;
    
  default:  throw TypeException("Variant::convert -> cannot convert type" + std::to_string(_type)); break;

  }
  return  target;
}

template<> inline SString Variant::convert() const
{
  if(  _type != STRING )
  {
     throw TypeException("Variant::convert -> cannot convert String");
  }
  return extract<SString>();
}

template<> inline ros::Time Variant::convert() const
{
  if(  _type != TIME )
  {
     throw TypeException("Variant::convert -> cannot convert ros::Time");
  }
  return extract<ros::Time>();
}

template<> inline ros::Duration Variant::convert() const
{
  if(  _type != TIME )
  {
     throw TypeException("Variant::convert -> cannot convert ros::Duration");
  }
  return extract<ros::Duration>();
}


} //end namespace





#endif // VARIANT_H
