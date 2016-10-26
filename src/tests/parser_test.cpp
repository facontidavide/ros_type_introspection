#include "config.h"
#include <gtest/gtest.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16MultiArray.h>
#include <ros_type_introspection/parser.hpp>

using namespace ros::message_traits;
using namespace RosIntrospection;

TEST(RosType, builtin_int32)
{
  ROSType f("int32");

  EXPECT_EQ(f.baseName(), "int32");
  EXPECT_EQ(f.msgName(),  "int32");
  EXPECT_EQ(f.pkgName().size(),  0);
  EXPECT_EQ(f.isArray(),  false);
  EXPECT_EQ(f.isBuiltin(),  true);
  EXPECT_EQ(f.arraySize(),  1);
  EXPECT_EQ(f.typeSize(),  4);
}

TEST(RosType,  builtin_string)
{
  ROSType f("string");
  EXPECT_EQ(f.baseName(),  "string");
  EXPECT_EQ(f.msgName() ,  "string");
  EXPECT_EQ(f.pkgName().size(),  0);
  EXPECT_EQ(f.isArray(),  false);
  EXPECT_EQ(f.isBuiltin(),  true);
  EXPECT_EQ(f.arraySize(),  1);
  EXPECT_EQ(f.typeSize(),  -1);
}


TEST(RosType, builtin_fixedlen_array)
{
  ROSType f("float64[32]");

  EXPECT_EQ(f.baseName(),  "float64[32]");
  EXPECT_EQ(f.msgName(),  "float64");
  EXPECT_EQ(f.pkgName().size(),  0 );
  EXPECT_EQ(f.isArray(),  true);
  EXPECT_EQ(f.isBuiltin(),  true);
  EXPECT_EQ(f.arraySize(),  32);
  EXPECT_EQ(f.typeSize(),  8);
}


TEST(RosType, no_builtin_array)
{
  ROSType f("geometry_msgs/Pose[]");

  EXPECT_EQ(f.baseName(),  "geometry_msgs/Pose[]" );
  EXPECT_EQ(f.msgName(),  "Pose" );
  EXPECT_EQ(f.pkgName(),  "geometry_msgs" );
  EXPECT_EQ(f.isArray(),  true);
  EXPECT_EQ(f.isBuiltin(),  false);
  EXPECT_EQ(f.arraySize(),  -1);
  EXPECT_EQ(f.typeSize(),  -1);
}

TEST(ROSMessageFields, ParseComments) {

  std::string
      def("MSG: geometry_msgs/Quaternion\n"
          "\n"
          "#just a comment"
          "          # I'm a comment after whitespace\n"
          "float64 x # I'm an end of line comment float64 y\n"
          "float64 z\n"
          );
  ROSMessage mt(def);
  EXPECT_EQ( mt.type().baseName(),  "geometry_msgs/Quaternion" );

  EXPECT_EQ( mt.fields().size(),  2);
  EXPECT_EQ(mt.field(0).type().msgName(),  "float64");
  EXPECT_EQ(mt.field(1).type().msgName(),  "float64");
  EXPECT_EQ(mt.field(0).name(),  "x");
  EXPECT_EQ(mt.field(1).name(),  "z");

  EXPECT_EQ( mt.field(0).isConstant(),  false);
  EXPECT_EQ( mt.field(1).isConstant(),  false);
}

TEST(ROSMessageFields, constant_uint8)
{
  ROSMessage msg("uint8 a = 66\n");

  EXPECT_EQ(msg.fields().size(),  1);
  EXPECT_EQ( msg.field(0).name(),  "a" );
  EXPECT_EQ( msg.field(0).type().baseName(),  "uint8" );
  EXPECT_EQ( msg.field(0).isConstant(),  true);
  EXPECT_EQ(msg.field(0).value(),  "66");
}

TEST(ROSMessageFields, ConstantNavstatus )
{
  ROSMessage msg( Definition<sensor_msgs::NavSatStatus >::value() );

  EXPECT_EQ( msg.fields().size(),  10);

  EXPECT_EQ( msg.field(0).name(), ("STATUS_NO_FIX") );
  EXPECT_EQ( msg.field(0).type().baseName(),  ("int8"));
  EXPECT_EQ( msg.field(0).value(), ("-1"));

  EXPECT_EQ( msg.field(1).name(),  ("STATUS_FIX") );
  EXPECT_EQ( msg.field(1).type().baseName(),  ("int8"));
  EXPECT_EQ( msg.field(1).value() ,  ("0"));

  EXPECT_EQ( msg.field(2).name(),  ("STATUS_SBAS_FIX") );
  EXPECT_EQ( msg.field(2).type().baseName(),  ("int8"));
  EXPECT_EQ( msg.field(2).value() ,  ("1"));

  EXPECT_EQ( msg.field(3).name(),  ("STATUS_GBAS_FIX") );
  EXPECT_EQ( msg.field(3).type().baseName(),  ("int8"));
  EXPECT_EQ( msg.field(3).value() ,  ("2"));

  EXPECT_EQ( msg.field(4).name(),  ("status") );
  EXPECT_EQ( msg.field(4).type().baseName(),  ("int8"));
  EXPECT_EQ( msg.field(4).isConstant() ,  false);

  EXPECT_EQ( msg.field(5).name(),  ("SERVICE_GPS") );
  EXPECT_EQ( msg.field(5).type().baseName(),  ("uint16"));
  EXPECT_EQ( msg.field(5).value() ,  ("1"));

  EXPECT_EQ( msg.field(6).name(),  ("SERVICE_GLONASS") );
  EXPECT_EQ( msg.field(6).type().baseName(),  ("uint16"));
  EXPECT_EQ( msg.field(6).value() ,  ("2"));

  EXPECT_EQ( msg.field(7).name(),  ("SERVICE_COMPASS") );
  EXPECT_EQ( msg.field(7).type().baseName(),  ("uint16"));
  EXPECT_EQ( msg.field(7).value() ,  ("4"));

  EXPECT_EQ( msg.field(8).name(),  ("SERVICE_GALILEO") );
  EXPECT_EQ( msg.field(8).type().baseName(),  ("uint16"));
  EXPECT_EQ( msg.field(8).value() ,  ("8"));

  EXPECT_EQ( msg.field(9).name(),  ("service") );
  EXPECT_EQ( msg.field(9).type().baseName(),  ("uint16"));
  EXPECT_EQ( msg.field(9).isConstant() ,  false);
}


TEST(ROSMessageFields, ConstantComments )
{
  ROSMessage msg(
        "string strA=  this string has a # comment in it  \n"
        "string strB = this string has \"quotes\" and \\slashes\\ in it\n"
        "float64 a=64.0 # numeric comment\n");


  EXPECT_EQ( msg.fields().size(),  3);
  EXPECT_EQ( ("strA"), msg.field(0).name());
  EXPECT_EQ( ("string"), msg.field(0).type().baseName() );
  EXPECT_EQ( msg.field(0).isConstant(),  true);
  EXPECT_EQ( ("this string has a # comment in it"),  msg.field(0).value());

  EXPECT_EQ( ("strB"),  msg.field(1).name());
  EXPECT_EQ( ("string"), msg.field(1).type().baseName() );
  EXPECT_EQ( msg.field(1).isConstant(),  true);
  EXPECT_EQ( ("this string has \"quotes\" and \\slashes\\ in it"),   msg.field(1).value());

  EXPECT_EQ( ("a"), msg.field(2).name());
  EXPECT_EQ( ("float64"), msg.field(2).type().baseName() );
  EXPECT_EQ( msg.field(2).isConstant(),  true);
  EXPECT_EQ( ("64.0"), msg.field(2).value());
}


TEST(buildROSTypeMapFromDefinition,  PoseParsing )
{
  RosIntrospection::ROSTypeList rmap;

  rmap = buildROSTypeMapFromDefinition(
        DataType<geometry_msgs::Pose >::value(),
        Definition<geometry_msgs::Pose >::value());

  if(VERBOSE_TEST){ std::cout << rmap << std::endl; }

  ROSMessage& msg = rmap.at(0);
  EXPECT_EQ( msg.type().baseName(),  "geometry_msgs/Pose" );
  EXPECT_EQ( msg.fields().size(),  2);
  EXPECT_EQ( msg.field(0).type().baseName(),  "geometry_msgs/Point" );
  EXPECT_EQ( msg.field(0).name(),  "position" );
  EXPECT_EQ( msg.field(1).type().baseName(),  "geometry_msgs/Quaternion" );
  EXPECT_EQ( msg.field(1).name(),  "orientation" );

  msg = rmap.at(1);
  EXPECT_EQ( ("geometry_msgs/Point" ),  msg.type().baseName() );
  EXPECT_EQ( msg.fields().size(),  3);
  EXPECT_EQ( msg.field(0).type().baseName(),  "float64" );
  EXPECT_EQ( msg.field(0).name(),  "x" );
  EXPECT_EQ( msg.field(1).type().baseName(),  "float64" );
  EXPECT_EQ( msg.field(1).name(),  "y" );
  EXPECT_EQ( msg.field(2).type().baseName(),  "float64" );
  EXPECT_EQ( msg.field(2).name(),  "z" );

  msg = rmap.at(2);
  EXPECT_EQ( ("geometry_msgs/Quaternion" ),  msg.type().baseName() );
  EXPECT_EQ( msg.fields().size(),  4);
  EXPECT_EQ( msg.field(0).type().baseName(),  "float64" );
  EXPECT_EQ( msg.field(0).name(),  "x" );
  EXPECT_EQ( msg.field(1).type().baseName() ,  "float64" );
  EXPECT_EQ( msg.field(1).name(),  "y" );
  EXPECT_EQ( msg.field(2).type().baseName() ,  "float64" );
  EXPECT_EQ( msg.field(2).name(),  "z" );
  EXPECT_EQ( msg.field(3).type().baseName() ,  "float64" );
  EXPECT_EQ( msg.field(3).name(),  "w" );
}

TEST(buildROSTypeMapFromDefinition,  IMUparsing )
{
  RosIntrospection::ROSTypeList rmap;

  rmap = buildROSTypeMapFromDefinition(
        DataType<sensor_msgs::Imu >::value(),
        Definition<sensor_msgs::Imu >::value());

  if(VERBOSE_TEST){ std::cout << rmap << std::endl; }

  ROSMessage& msg = rmap.at(0);
  EXPECT_EQ( ("sensor_msgs/Imu"),  msg.type().baseName() );
  EXPECT_EQ( msg.fields().size(),  7);
  EXPECT_EQ( ("std_msgs/Header" ),  msg.field(0).type().baseName() );
  EXPECT_EQ( ("header" )         ,  msg.field(0).name() );

  EXPECT_EQ( ("geometry_msgs/Quaternion" ),  msg.field(1).type().baseName() );
  EXPECT_EQ( ("orientation" )             ,  msg.field(1).name() );

  EXPECT_EQ( ("float64[9]" )             ,  msg.field(2).type().baseName() );
  EXPECT_EQ( ("orientation_covariance" ) ,  msg.field(2).name() );
  EXPECT_EQ( msg.field(2).type().arraySize(),  9);

  EXPECT_EQ( ("geometry_msgs/Vector3" ),  msg.field(3).type().baseName() );
  EXPECT_EQ( ("angular_velocity" )     ,  msg.field(3).name() );

  EXPECT_EQ( ("float64[9]" )                 ,  msg.field(4).type().baseName() );
  EXPECT_EQ( ("angular_velocity_covariance" ),  msg.field(4).name() );
  EXPECT_EQ( msg.field(4).type().arraySize(),  9);

  EXPECT_EQ( ("geometry_msgs/Vector3" ),  msg.field(5).type().baseName() );
  EXPECT_EQ( ("linear_acceleration" )  ,  msg.field(5).name() );

  EXPECT_EQ( ("float64[9]" )                    ,  msg.field(6).type().baseName() );
  EXPECT_EQ( ("linear_acceleration_covariance" ),  msg.field(6).name() );
  EXPECT_EQ( msg.field(6).type().arraySize(),  9);


  //---------------------------------
  msg = rmap.at(1);
  EXPECT_EQ( msg.type().baseName(),  "std_msgs/Header" );
  EXPECT_EQ( msg.fields().size(),  3);
  EXPECT_EQ( msg.field(0).type().baseName(),  ("uint32" ));
  EXPECT_EQ( msg.field(0).name(),  ("seq") );
  EXPECT_EQ( msg.field(1).type().baseName(),  ("time") );
  EXPECT_EQ( msg.field(1).name(),  "stamp" );
  EXPECT_EQ( msg.field(2).type().baseName(),  "string" );
  EXPECT_EQ( msg.field(2).name(),  "frame_id" );

  msg = rmap.at(2);
  EXPECT_EQ( msg.type().baseName(),  ("geometry_msgs/Quaternion") );
  EXPECT_EQ( msg.fields().size(),  4);
  EXPECT_EQ( msg.field(0).type().baseName(),  "float64" );
  EXPECT_EQ( msg.field(0).name(),  "x" );
  EXPECT_EQ( msg.field(1).type().baseName(),  "float64" );
  EXPECT_EQ( msg.field(1).name(),  "y" );
  EXPECT_EQ( msg.field(2).type().baseName(),  "float64" );
  EXPECT_EQ( msg.field(2).name(),  "z" );
  EXPECT_EQ( msg.field(3).type().baseName(),  "float64" );
  EXPECT_EQ( msg.field(3).name(),  "w" );

  msg = rmap.at(3);
  EXPECT_EQ( msg.type().baseName(),  ("geometry_msgs/Vector3") );
  EXPECT_EQ( msg.fields().size(),  3);
  EXPECT_EQ( msg.field(0).type().baseName(),  "float64" );
  EXPECT_EQ( msg.field(0).name(),  "x" );
  EXPECT_EQ( msg.field(1).type().baseName(),  "float64" );
  EXPECT_EQ( msg.field(1).name(),  "y" );
  EXPECT_EQ( msg.field(2).type().baseName(),  "float64" );
  EXPECT_EQ( msg.field(2).name(),  "z" );
}

TEST(buildROSTypeMapFromDefinition,  Int16MultiArrayParsing )
{
  // this test case was added because it previously failed to detect nested
  // arrays of custom types. In this case:
  //    std_msgs/MultiArrayDimension[]

  RosIntrospection::ROSTypeList rmap;

  rmap = buildROSTypeMapFromDefinition(
        DataType<std_msgs::Int16MultiArray >::value(),
        Definition<std_msgs::Int16MultiArray >::value());

  if(VERBOSE_TEST){
    std::cout << rmap << std::endl;
  }

  /*std_msgs/Int16MultiArray :
          layout : std_msgs/MultiArrayLayout
          data : int16[]

      std_msgs/MultiArrayLayout :
          dim : std_msgs/MultiArrayDimension[]
          data_offset : uint32

      std_msgs/MultiArrayDimension :
          label : string
          size : uint32
          stride : uint32*/

  ROSMessage& msg = rmap.at(0);

  EXPECT_EQ( ("std_msgs/Int16MultiArray"),  msg.type().baseName() );
  EXPECT_EQ( msg.fields().size(),  2);
  EXPECT_EQ( ("std_msgs/MultiArrayLayout" ),  msg.field(0).type().baseName() );
  EXPECT_EQ( ("layout" )                   ,  msg.field(0).name() );
  EXPECT_EQ( false,  msg.field(0).type().isArray() );

  EXPECT_EQ( ("int16[]" ),  msg.field(1).type().baseName() );
  EXPECT_EQ( ("data" ) ,  msg.field(1).name() );
  EXPECT_EQ( true,  msg.field(1).type().isArray() );

  msg = rmap.at(1);
  EXPECT_EQ( ("std_msgs/MultiArrayLayout"),  msg.type().baseName() );
  EXPECT_EQ( msg.fields().size(),  2);
  EXPECT_EQ( ("std_msgs/MultiArrayDimension[]" ),  msg.field(0).type().baseName() );
  EXPECT_EQ( ("dim" )                           ,  msg.field(0).name() );
  EXPECT_EQ( true,  msg.field(0).type().isArray() );

  EXPECT_EQ( ("uint32" )      ,  msg.field(1).type().baseName() );
  EXPECT_EQ( ("data_offset" ) ,  msg.field(1).name() );
  EXPECT_EQ( false,  msg.field(1).type().isArray() );


  msg = rmap.at(2);
  EXPECT_EQ( ("std_msgs/MultiArrayDimension"),  msg.type().baseName() );
  EXPECT_EQ( msg.fields().size(),  3);

  EXPECT_EQ( ("string" ),  msg.field(0).type().baseName() );
  EXPECT_EQ( ("label" )  ,  msg.field(0).name() );
  EXPECT_EQ( false,  msg.field(0).type().isArray() );

  EXPECT_EQ( ("uint32" ),  msg.field(1).type().baseName() );
  EXPECT_EQ( ("size" )  ,  msg.field(1).name() );
  EXPECT_EQ( false,  msg.field(1).type().isArray() );

  EXPECT_EQ( ("uint32" ) ,  msg.field(2).type().baseName() );
  EXPECT_EQ( ("stride" ) ,  msg.field(2).name() );
  EXPECT_EQ( false,  msg.field(2).type().isArray() );

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

