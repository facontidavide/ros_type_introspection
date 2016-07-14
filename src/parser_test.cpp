#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch.h>

#include <topic_tools/shape_shifter.h>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/utility/string_ref.hpp>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <tf/tfMessage.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Imu.h>

#include <sstream>
#include <iostream>
#include <chrono>
#include <ros_type_introspection/ros_type_introspection.hpp>

using namespace ros::message_traits;
using namespace ROSTypeParser;


void compare( const ROSTypeMap& mapA, const ROSTypeMap& mapB )
{
    REQUIRE( mapA.size() == mapB.size() );
    for (auto itA = mapA.begin() ; itA != mapA.end(); itA++)
    {
        auto itB = mapB.find( itA->first );
        REQUIRE( itA->first == itB->first );

        const ROSMessage& msgA = itA->second;
        const ROSMessage& msgB = itB->second;

        REQUIRE( msgA.type.msgName() == msgB.type.msgName() );

        auto fieldsA = msgA.fields;
        auto fieldsB = msgB.fields;
        REQUIRE( fieldsA.size() == fieldsB.size() );

        for (int i=0; i<fieldsA.size(); i++) {
            REQUIRE( fieldsA[i].type().msgName()  == fieldsB[i].type().msgName() );
            REQUIRE( fieldsA[i].name() == fieldsB[i].name() );
        }
    }
}

TEST_CASE("builtin_int32", "ROSType")
{
    ROSType f("int32");

    REQUIRE(std::string("int32") == f.baseName());
    REQUIRE(std::string("int32") == f.msgName());
    REQUIRE(std::string("") == f.pkgName());
    REQUIRE(f.isArray() == false);
    REQUIRE(f.isBuiltin() == true);
    REQUIRE(f.arraySize() == 1);
    REQUIRE(f.typeSize() == 4);
}

TEST_CASE( "builtin_string", "ROSType")
{
    ROSType f("string");
    REQUIRE(std::string("string") == f.baseName());
    REQUIRE(std::string("string") == f.msgName());
    REQUIRE(std::string("") == f.pkgName());
    REQUIRE(f.isArray() == false);
    REQUIRE(f.isBuiltin() == true);
    REQUIRE(f.arraySize() == 1);
    REQUIRE(f.typeSize() == -1);
}


TEST_CASE("builtin_fixedlen_array", "ROSType")
{
    ROSType f("float64[32]");

    REQUIRE(std::string("float64[32]") == f.baseName());
    REQUIRE(std::string("float64") == f.msgName());
    REQUIRE(std::string("") == f.pkgName());
    REQUIRE(f.isArray() == true);
    REQUIRE(f.isBuiltin() == true);
    REQUIRE(f.arraySize() == 32);
    REQUIRE(f.typeSize() == 8);
}


TEST_CASE("no_builtin_array", "ROSType")
{
    ROSType f("geometry_msgs/Pose[]");

    REQUIRE(std::string("geometry_msgs/Pose[]") == f.baseName());
    REQUIRE(std::string("Pose") == f.msgName());
    REQUIRE(std::string("geometry_msgs") == f.pkgName());
    REQUIRE(f.isArray() == true);
    REQUIRE(f.isBuiltin() == false);
    REQUIRE(f.arraySize() == -1);
    REQUIRE(f.typeSize() == -1);
}

TEST_CASE("parse_comments", "ROSMessageFields") {

  std::string
    def("MSG: geometry_msgs/Quaternion\n"
        "\n"
        "#just a comment"
        "          # I'm a comment after whitespace\n"
        "float64 x # I'm an end of line comment float64 y\n"
        "float64 z\n"
        );
  ROSMessage mt(def);
  REQUIRE(std::string("geometry_msgs/Quaternion") == mt.type.baseName() );

  REQUIRE( mt.fields.size() == 2);
  REQUIRE(std::string("float64") ==  mt.fields[0].type().msgName());
  REQUIRE(std::string("float64") ==  mt.fields[1].type().msgName());
  REQUIRE(std::string("x") == mt.fields[0].name());
  REQUIRE(std::string("z") == mt.fields[1].name());

  REQUIRE( mt.fields[0].isConstant() == false);
  REQUIRE( mt.fields[1].isConstant() == false);
}

TEST_CASE("constant_uint8", "ROSMessageFields")
{
  ROSMessage msg("uint8 a = 66\n");

  REQUIRE(msg.fields.size() == 1);
  REQUIRE( std::string("a") == msg.fields[0].name() );
  REQUIRE( std::string("uint8") == msg.fields[0].type().baseName() );
  REQUIRE(msg.fields[0].isConstant() == true);
  REQUIRE( std::string("66") == msg.fields[0].value());
}

TEST_CASE("constant_example_navstatus", "ROSMessageFields")
{
    ROSMessage msg( Definition<sensor_msgs::NavSatStatus >::value() );

    REQUIRE( msg.fields.size() == 10);

    REQUIRE( msg.fields[0].name() == std::string("STATUS_NO_FIX") );
    REQUIRE( msg.fields[0].type().baseName() == std::string("int8"));
    REQUIRE( msg.fields[0].value()  == std::string("-1"));

    REQUIRE( msg.fields[1].name() == std::string("STATUS_FIX") );
    REQUIRE( msg.fields[1].type().baseName() == std::string("int8"));
    REQUIRE( msg.fields[1].value()  == std::string("0"));

    REQUIRE( msg.fields[2].name() == std::string("STATUS_SBAS_FIX") );
    REQUIRE( msg.fields[2].type().baseName() == std::string("int8"));
    REQUIRE( msg.fields[2].value()  == std::string("1"));

    REQUIRE( msg.fields[3].name() == std::string("STATUS_GBAS_FIX") );
    REQUIRE( msg.fields[3].type().baseName() == std::string("int8"));
    REQUIRE( msg.fields[3].value()  == std::string("2"));

    REQUIRE( msg.fields[4].name() == std::string("status") );
    REQUIRE( msg.fields[4].type().baseName() == std::string("int8"));
    REQUIRE( msg.fields[4].isConstant()  == false);

    REQUIRE( msg.fields[5].name() == std::string("SERVICE_GPS") );
    REQUIRE( msg.fields[5].type().baseName() == std::string("uint16"));
    REQUIRE( msg.fields[5].value()  == std::string("1"));

    REQUIRE( msg.fields[6].name() == std::string("SERVICE_GLONASS") );
    REQUIRE( msg.fields[6].type().baseName() == std::string("uint16"));
    REQUIRE( msg.fields[6].value()  == std::string("2"));

    REQUIRE( msg.fields[7].name() == std::string("SERVICE_COMPASS") );
    REQUIRE( msg.fields[7].type().baseName() == std::string("uint16"));
    REQUIRE( msg.fields[7].value()  == std::string("4"));

    REQUIRE( msg.fields[8].name() == std::string("SERVICE_GALILEO") );
    REQUIRE( msg.fields[8].type().baseName() == std::string("uint16"));
    REQUIRE( msg.fields[8].value()  == std::string("8"));

    REQUIRE( msg.fields[9].name() == std::string("service") );
    REQUIRE( msg.fields[9].type().baseName() == std::string("uint16"));
    REQUIRE( msg.fields[9].isConstant()  == false);
}


TEST_CASE("constant_comments", "ROSMessageFields")
{
  ROSMessage msg(
    "string strA=  this string has a # comment in it  \n"
    "string strB = this string has \"quotes\" and \\slashes\\ in it\n"
    "float64 a=64.0 # numeric comment\n");


  REQUIRE( msg.fields.size() == 3);
  REQUIRE( std::string("strA")== msg.fields[0].name());
  REQUIRE( std::string("string")== msg.fields[0].type().baseName() );
  REQUIRE( msg.fields[0].isConstant() == true);
  REQUIRE( std::string("this string has a # comment in it") == msg.fields[0].value());

  REQUIRE( std::string("strB") == msg.fields[1].name());
  REQUIRE( std::string("string")== msg.fields[1].type().baseName() );
  REQUIRE( msg.fields[1].isConstant() == true);
  REQUIRE( std::string("this string has \"quotes\" and \\slashes\\ in it") ==  msg.fields[1].value());

  REQUIRE( std::string("a")== msg.fields[2].name());
  REQUIRE( std::string("float64")== msg.fields[2].type().baseName() );
  REQUIRE( msg.fields[2].isConstant() == true);
  REQUIRE( std::string("64.0")== msg.fields[2].value());
}


TEST_CASE( "Test Pose parsing", "buildROSTypeMapFromDefinition" )
{
    ROSTypeParser::ROSTypeMap rmap;

    rmap = buildROSTypeMapFromDefinition(
                DataType<geometry_msgs::Pose >::value(),
                Definition<geometry_msgs::Pose >::value());

    ROSMessage& msg = rmap["Pose"];
    REQUIRE( std::string("geometry_msgs/Pose" ) == msg.type.baseName() );
    REQUIRE( msg.fields.size() == 2);
    REQUIRE( std::string("geometry_msgs/Point" )      == msg.fields[0].type().baseName() );
    REQUIRE( std::string("position" )                 == msg.fields[0].name() );
    REQUIRE( std::string("geometry_msgs/Quaternion" ) == msg.fields[1].type().baseName() );
    REQUIRE( std::string("orientation")               == msg.fields[1].name() );

    msg = rmap["Point"];
    REQUIRE( std::string("geometry_msgs/Point" ) == msg.type.baseName() );
    REQUIRE( msg.fields.size() == 3);
    REQUIRE( std::string("float64" ) == msg.fields[0].type().baseName() );
    REQUIRE( std::string("x" )       == msg.fields[0].name() );
    REQUIRE( std::string("float64" ) == msg.fields[1].type().baseName() );
    REQUIRE( std::string("y")        == msg.fields[1].name() );
    REQUIRE( std::string("float64" ) == msg.fields[2].type().baseName() );
    REQUIRE( std::string("z")        == msg.fields[2].name() );

    msg = rmap["Quaternion"];
    REQUIRE( std::string("geometry_msgs/Quaternion" ) == msg.type.baseName() );
    REQUIRE( msg.fields.size() == 4);
    REQUIRE( std::string("float64" ) == msg.fields[0].type().baseName() );
    REQUIRE( std::string("x" )       == msg.fields[0].name() );
    REQUIRE( std::string("float64" ) == msg.fields[1].type().baseName() );
    REQUIRE( std::string("y")        == msg.fields[1].name() );
    REQUIRE( std::string("float64" ) == msg.fields[2].type().baseName() );
    REQUIRE( std::string("z")        == msg.fields[2].name() );
    REQUIRE( std::string("float64" ) == msg.fields[3].type().baseName() );
    REQUIRE( std::string("w")        == msg.fields[3].name() );
}

TEST_CASE( "Test IMU parsing", "buildROSTypeMapFromDefinition" )
{
    ROSTypeParser::ROSTypeMap rmap;

    rmap = buildROSTypeMapFromDefinition(
                DataType<sensor_msgs::Imu >::value(),
                Definition<sensor_msgs::Imu >::value());

    ROSMessage& msg = rmap["Imu"];
    REQUIRE( std::string("sensor_msgs/Imu") == msg.type.baseName() );
    REQUIRE( msg.fields.size() == 7);
    REQUIRE( std::string("std_msgs/Header" ) == msg.fields[0].type().baseName() );
    REQUIRE( std::string("header" )          == msg.fields[0].name() );

    REQUIRE( std::string("geometry_msgs/Quaternion" ) == msg.fields[1].type().baseName() );
    REQUIRE( std::string("orientation" )              == msg.fields[1].name() );

    REQUIRE( std::string("float64[9]" )              == msg.fields[2].type().baseName() );
    REQUIRE( std::string("orientation_covariance" )  == msg.fields[2].name() );
    REQUIRE( msg.fields[2].type().arraySize() == 9);

    REQUIRE( std::string("geometry_msgs/Vector3" ) == msg.fields[3].type().baseName() );
    REQUIRE( std::string("angular_velocity" )      == msg.fields[3].name() );

    REQUIRE( std::string("float64[9]" )                  == msg.fields[4].type().baseName() );
    REQUIRE( std::string("angular_velocity_covariance" ) == msg.fields[4].name() );
    REQUIRE( msg.fields[4].type().arraySize() == 9);

    REQUIRE( std::string("geometry_msgs/Vector3" ) == msg.fields[5].type().baseName() );
    REQUIRE( std::string("linear_acceleration" )   == msg.fields[5].name() );

    REQUIRE( std::string("float64[9]" )                     == msg.fields[6].type().baseName() );
    REQUIRE( std::string("linear_acceleration_covariance" ) == msg.fields[6].name() );
    REQUIRE( msg.fields[6].type().arraySize() == 9);

    //---------------------------------
    msg = rmap["Header"];
    REQUIRE( std::string("std_msgs/Header") == msg.type.baseName() );
    REQUIRE( msg.fields.size() == 3);
    REQUIRE( std::string("uint32" )  == msg.fields[0].type().baseName() );
    REQUIRE( std::string("seq" )     == msg.fields[0].name() );
    REQUIRE( std::string("time" )    == msg.fields[1].type().baseName() );
    REQUIRE( std::string("stamp" )   == msg.fields[1].name() );
    REQUIRE( std::string("string")   == msg.fields[2].type().baseName() );
    REQUIRE( std::string("frame_id") == msg.fields[2].name() );

    msg = rmap["Quaternion"];
    REQUIRE( std::string("geometry_msgs/Quaternion") == msg.type.baseName() );
    REQUIRE( msg.fields.size() == 4);
    REQUIRE( std::string("float64" ) == msg.fields[0].type().baseName() );
    REQUIRE( std::string("x" )       == msg.fields[0].name() );
    REQUIRE( std::string("float64" ) == msg.fields[1].type().baseName() );
    REQUIRE( std::string("y")        == msg.fields[1].name() );
    REQUIRE( std::string("float64" ) == msg.fields[2].type().baseName() );
    REQUIRE( std::string("z")        == msg.fields[2].name() );
    REQUIRE( std::string("float64" ) == msg.fields[3].type().baseName() );
    REQUIRE( std::string("w")        == msg.fields[3].name() );

    msg = rmap["Vector3"];
    REQUIRE( std::string("geometry_msgs/Vector3") == msg.type.baseName() );
    REQUIRE( msg.fields.size() == 3);
    REQUIRE( std::string("float64" ) == msg.fields[0].type().baseName() );
    REQUIRE( std::string("x" )       == msg.fields[0].name() );
    REQUIRE( std::string("float64" ) == msg.fields[1].type().baseName() );
    REQUIRE( std::string("y")        == msg.fields[1].name() );
    REQUIRE( std::string("float64" ) == msg.fields[2].type().baseName() );
    REQUIRE( std::string("z")        == msg.fields[2].name() );
}

