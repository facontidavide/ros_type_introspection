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

  EXPECT_TRUE(f.baseName() =="int32");
  EXPECT_TRUE(f.msgName() == "int32");
  EXPECT_TRUE(f.pkgName().size() == 0);
  EXPECT_TRUE(f.isArray() == false);
  EXPECT_TRUE(f.isBuiltin() == true);
  EXPECT_TRUE(f.arraySize() == 1);
  EXPECT_TRUE(f.typeSize() == 4);
}

TEST(RosType,  builtin_string)
{
  ROSType f("string");
  EXPECT_TRUE(f.baseName() == "string");
  EXPECT_TRUE(f.msgName()  == "string");
  EXPECT_TRUE(f.pkgName().size() == 0);
  EXPECT_TRUE(f.isArray() == false);
  EXPECT_TRUE(f.isBuiltin() == true);
  EXPECT_TRUE(f.arraySize() == 1);
  EXPECT_TRUE(f.typeSize() == -1);
}


TEST(RosType, builtin_fixedlen_array)
{
  ROSType f("float64[32]");

  EXPECT_TRUE(f.baseName() == "float64[32]");
  EXPECT_TRUE(f.msgName() == "float64");
  EXPECT_TRUE(f.pkgName().size() == 0 );
  EXPECT_TRUE(f.isArray() == true);
  EXPECT_TRUE(f.isBuiltin() == true);
  EXPECT_TRUE(f.arraySize() == 32);
  EXPECT_TRUE(f.typeSize() == 8);
}


TEST(RosType, no_builtin_array)
{
  ROSType f("geometry_msgs/Pose[]");

  EXPECT_TRUE(f.baseName() == "geometry_msgs/Pose[]" );
  EXPECT_TRUE(f.msgName() == "Pose" );
  EXPECT_TRUE(f.pkgName() == "geometry_msgs" );
  EXPECT_TRUE(f.isArray() == true);
  EXPECT_TRUE(f.isBuiltin() == false);
  EXPECT_TRUE(f.arraySize() == -1);
  EXPECT_TRUE(f.typeSize() == -1);
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
  EXPECT_TRUE( mt.type().baseName() == "geometry_msgs/Quaternion" );

  EXPECT_TRUE( mt.fields().size() == 2);
  EXPECT_TRUE(mt.field(0).type().msgName() == "float64");
  EXPECT_TRUE(mt.field(1).type().msgName() == "float64");
  EXPECT_TRUE(mt.field(0).name() == "x");
  EXPECT_TRUE(mt.field(1).name() == "z");

  EXPECT_TRUE( mt.field(0).isConstant() == false);
  EXPECT_TRUE( mt.field(1).isConstant() == false);
}

TEST(ROSMessageFields, constant_uint8)
{
  ROSMessage msg("uint8 a = 66\n");

  EXPECT_TRUE(msg.fields().size() == 1);
  EXPECT_TRUE( msg.field(0).name() == "a" );
  EXPECT_TRUE( msg.field(0).type().baseName() == "uint8" );
  EXPECT_TRUE( msg.field(0).isConstant() == true);
  EXPECT_TRUE(msg.field(0).value() == "66");
}

TEST(ROSMessageFields, ConstantNavstatus )
{
  ROSMessage msg( Definition<sensor_msgs::NavSatStatus >::value() );

  EXPECT_TRUE( msg.fields().size() == 10);

  EXPECT_TRUE( msg.field(0).name() == ("STATUS_NO_FIX") );
  EXPECT_TRUE( msg.field(0).type().baseName() == ("int8"));
  EXPECT_TRUE( msg.field(0).value()  == ("-1"));

  EXPECT_TRUE( msg.field(1).name() == ("STATUS_FIX") );
  EXPECT_TRUE( msg.field(1).type().baseName() == ("int8"));
  EXPECT_TRUE( msg.field(1).value()  == ("0"));

  EXPECT_TRUE( msg.field(2).name() == ("STATUS_SBAS_FIX") );
  EXPECT_TRUE( msg.field(2).type().baseName() == ("int8"));
  EXPECT_TRUE( msg.field(2).value()  == ("1"));

  EXPECT_TRUE( msg.field(3).name() == ("STATUS_GBAS_FIX") );
  EXPECT_TRUE( msg.field(3).type().baseName() == ("int8"));
  EXPECT_TRUE( msg.field(3).value()  == ("2"));

  EXPECT_TRUE( msg.field(4).name() == ("status") );
  EXPECT_TRUE( msg.field(4).type().baseName() == ("int8"));
  EXPECT_TRUE( msg.field(4).isConstant()  == false);

  EXPECT_TRUE( msg.field(5).name() == ("SERVICE_GPS") );
  EXPECT_TRUE( msg.field(5).type().baseName() == ("uint16"));
  EXPECT_TRUE( msg.field(5).value()  == ("1"));

  EXPECT_TRUE( msg.field(6).name() == ("SERVICE_GLONASS") );
  EXPECT_TRUE( msg.field(6).type().baseName() == ("uint16"));
  EXPECT_TRUE( msg.field(6).value()  == ("2"));

  EXPECT_TRUE( msg.field(7).name() == ("SERVICE_COMPASS") );
  EXPECT_TRUE( msg.field(7).type().baseName() == ("uint16"));
  EXPECT_TRUE( msg.field(7).value()  == ("4"));

  EXPECT_TRUE( msg.field(8).name() == ("SERVICE_GALILEO") );
  EXPECT_TRUE( msg.field(8).type().baseName() == ("uint16"));
  EXPECT_TRUE( msg.field(8).value()  == ("8"));

  EXPECT_TRUE( msg.field(9).name() == ("service") );
  EXPECT_TRUE( msg.field(9).type().baseName() == ("uint16"));
  EXPECT_TRUE( msg.field(9).isConstant()  == false);
}


TEST(ROSMessageFields, ConstantComments )
{
  ROSMessage msg(
        "string strA=  this string has a # comment in it  \n"
        "string strB = this string has \"quotes\" and \\slashes\\ in it\n"
        "float64 a=64.0 # numeric comment\n");


  EXPECT_TRUE( msg.fields().size() == 3);
  EXPECT_TRUE( ("strA")== msg.field(0).name());
  EXPECT_TRUE( ("string")== msg.field(0).type().baseName() );
  EXPECT_TRUE( msg.field(0).isConstant() == true);
  EXPECT_TRUE( ("this string has a # comment in it") == msg.field(0).value());

  EXPECT_TRUE( ("strB") == msg.field(1).name());
  EXPECT_TRUE( ("string")== msg.field(1).type().baseName() );
  EXPECT_TRUE( msg.field(1).isConstant() == true);
  EXPECT_TRUE( ("this string has \"quotes\" and \\slashes\\ in it") ==  msg.field(1).value());

  EXPECT_TRUE( ("a")== msg.field(2).name());
  EXPECT_TRUE( ("float64")== msg.field(2).type().baseName() );
  EXPECT_TRUE( msg.field(2).isConstant() == true);
  EXPECT_TRUE( ("64.0")== msg.field(2).value());
}


TEST(buildROSTypeMapFromDefinition,  PoseParsing )
{
  RosIntrospection::ROSTypeList rmap;

  rmap = buildROSTypeMapFromDefinition(
        DataType<geometry_msgs::Pose >::value(),
        Definition<geometry_msgs::Pose >::value());

  if(VERBOSE_TEST){ std::cout << rmap << std::endl; }

  ROSMessage& msg = rmap.at(0);
  EXPECT_TRUE( msg.type().baseName() == "geometry_msgs/Pose" );
  EXPECT_TRUE( msg.fields().size() == 2);
  EXPECT_TRUE( msg.field(0).type().baseName() == "geometry_msgs/Point" );
  EXPECT_TRUE( msg.field(0).name() == "position" );
  EXPECT_TRUE( msg.field(1).type().baseName() == "geometry_msgs/Quaternion" );
  EXPECT_TRUE( msg.field(1).name() == "orientation" );

  msg = rmap.at(1);
  EXPECT_TRUE( ("geometry_msgs/Point" ) == msg.type().baseName() );
  EXPECT_TRUE( msg.fields().size() == 3);
  EXPECT_TRUE( msg.field(0).type().baseName() == "float64" );
  EXPECT_TRUE( msg.field(0).name() == "x" );
  EXPECT_TRUE( msg.field(1).type().baseName() == "float64" );
  EXPECT_TRUE( msg.field(1).name() == "y" );
  EXPECT_TRUE( msg.field(2).type().baseName() == "float64" );
  EXPECT_TRUE( msg.field(2).name() == "z" );

  msg = rmap.at(2);
  EXPECT_TRUE( ("geometry_msgs/Quaternion" ) == msg.type().baseName() );
  EXPECT_TRUE( msg.fields().size() == 4);
  EXPECT_TRUE( msg.field(0).type().baseName() == "float64" );
  EXPECT_TRUE( msg.field(0).name() == "x" );
  EXPECT_TRUE( msg.field(1).type().baseName()  == "float64" );
  EXPECT_TRUE( msg.field(1).name() == "y" );
  EXPECT_TRUE( msg.field(2).type().baseName()  == "float64" );
  EXPECT_TRUE( msg.field(2).name() == "z" );
  EXPECT_TRUE( msg.field(3).type().baseName()  == "float64" );
  EXPECT_TRUE( msg.field(3).name() == "w" );
}

TEST(buildROSTypeMapFromDefinition,  IMUparsing )
{
  RosIntrospection::ROSTypeList rmap;

  rmap = buildROSTypeMapFromDefinition(
        DataType<sensor_msgs::Imu >::value(),
        Definition<sensor_msgs::Imu >::value());

  if(VERBOSE_TEST){ std::cout << rmap << std::endl; }

  ROSMessage& msg = rmap.at(0);
  EXPECT_TRUE( ("sensor_msgs/Imu") == msg.type().baseName() );
  EXPECT_TRUE( msg.fields().size() == 7);
  EXPECT_TRUE( ("std_msgs/Header" ) == msg.field(0).type().baseName() );
  EXPECT_TRUE( ("header" )          == msg.field(0).name() );

  EXPECT_TRUE( ("geometry_msgs/Quaternion" ) == msg.field(1).type().baseName() );
  EXPECT_TRUE( ("orientation" )              == msg.field(1).name() );

  EXPECT_TRUE( ("float64[9]" )              == msg.field(2).type().baseName() );
  EXPECT_TRUE( ("orientation_covariance" )  == msg.field(2).name() );
  EXPECT_TRUE( msg.field(2).type().arraySize() == 9);

  EXPECT_TRUE( ("geometry_msgs/Vector3" ) == msg.field(3).type().baseName() );
  EXPECT_TRUE( ("angular_velocity" )      == msg.field(3).name() );

  EXPECT_TRUE( ("float64[9]" )                  == msg.field(4).type().baseName() );
  EXPECT_TRUE( ("angular_velocity_covariance" ) == msg.field(4).name() );
  EXPECT_TRUE( msg.field(4).type().arraySize() == 9);

  EXPECT_TRUE( ("geometry_msgs/Vector3" ) == msg.field(5).type().baseName() );
  EXPECT_TRUE( ("linear_acceleration" )   == msg.field(5).name() );

  EXPECT_TRUE( ("float64[9]" )                     == msg.field(6).type().baseName() );
  EXPECT_TRUE( ("linear_acceleration_covariance" ) == msg.field(6).name() );
  EXPECT_TRUE( msg.field(6).type().arraySize() == 9);


  //---------------------------------
  msg = rmap.at(1);
  EXPECT_TRUE( msg.type().baseName() == "std_msgs/Header" );
  EXPECT_TRUE( msg.fields().size() == 3);
  EXPECT_TRUE( msg.field(0).type().baseName() == ("uint32" ));
  EXPECT_TRUE( msg.field(0).name() == ("seq") );
  EXPECT_TRUE( msg.field(1).type().baseName() == ("time") );
  EXPECT_TRUE( msg.field(1).name() == "stamp" );
  EXPECT_TRUE( msg.field(2).type().baseName() == "string" );
  EXPECT_TRUE( msg.field(2).name() == "frame_id" );

  msg = rmap.at(2);
  EXPECT_TRUE( msg.type().baseName() == ("geometry_msgs/Quaternion") );
  EXPECT_TRUE( msg.fields().size() == 4);
  EXPECT_TRUE( msg.field(0).type().baseName() == "float64" );
  EXPECT_TRUE( msg.field(0).name() == "x" );
  EXPECT_TRUE( msg.field(1).type().baseName() == "float64" );
  EXPECT_TRUE( msg.field(1).name() == "y" );
  EXPECT_TRUE( msg.field(2).type().baseName() == "float64" );
  EXPECT_TRUE( msg.field(2).name() == "z" );
  EXPECT_TRUE( msg.field(3).type().baseName() == "float64" );
  EXPECT_TRUE( msg.field(3).name() == "w" );

  msg = rmap.at(3);
  EXPECT_TRUE( msg.type().baseName() == ("geometry_msgs/Vector3") );
  EXPECT_TRUE( msg.fields().size() == 3);
  EXPECT_TRUE( msg.field(0).type().baseName() == "float64" );
  EXPECT_TRUE( msg.field(0).name() == "x" );
  EXPECT_TRUE( msg.field(1).type().baseName() == "float64" );
  EXPECT_TRUE( msg.field(1).name() == "y" );
  EXPECT_TRUE( msg.field(2).type().baseName() == "float64" );
  EXPECT_TRUE( msg.field(2).name() == "z" );
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

  EXPECT_TRUE( ("std_msgs/Int16MultiArray") == msg.type().baseName() );
  EXPECT_TRUE( msg.fields().size() == 2);
  EXPECT_TRUE( ("std_msgs/MultiArrayLayout" ) == msg.field(0).type().baseName() );
  EXPECT_TRUE( ("layout" )                    == msg.field(0).name() );
  EXPECT_TRUE( false == msg.field(0).type().isArray() );

  EXPECT_TRUE( ("int16[]" ) == msg.field(1).type().baseName() );
  EXPECT_TRUE( ("data" )  == msg.field(1).name() );
  EXPECT_TRUE( true == msg.field(1).type().isArray() );

  msg = rmap.at(1);
  EXPECT_TRUE( ("std_msgs/MultiArrayLayout") == msg.type().baseName() );
  EXPECT_TRUE( msg.fields().size() == 2);
  EXPECT_TRUE( ("std_msgs/MultiArrayDimension[]" ) == msg.field(0).type().baseName() );
  EXPECT_TRUE( ("dim" )                            == msg.field(0).name() );
  EXPECT_TRUE( true == msg.field(0).type().isArray() );

  EXPECT_TRUE( ("uint32" )       == msg.field(1).type().baseName() );
  EXPECT_TRUE( ("data_offset" )  == msg.field(1).name() );
  EXPECT_TRUE( false == msg.field(1).type().isArray() );


  msg = rmap.at(2);
  EXPECT_TRUE( ("std_msgs/MultiArrayDimension") == msg.type().baseName() );
  EXPECT_TRUE( msg.fields().size() == 3);

  EXPECT_TRUE( ("string" ) == msg.field(0).type().baseName() );
  EXPECT_TRUE( ("label" )   == msg.field(0).name() );
  EXPECT_TRUE( false == msg.field(0).type().isArray() );

  EXPECT_TRUE( ("uint32" ) == msg.field(1).type().baseName() );
  EXPECT_TRUE( ("size" )   == msg.field(1).name() );
  EXPECT_TRUE( false == msg.field(1).type().isArray() );

  EXPECT_TRUE( ("uint32" )  == msg.field(2).type().baseName() );
  EXPECT_TRUE( ("stride" )  == msg.field(2).name() );
  EXPECT_TRUE( false == msg.field(2).type().isArray() );

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

