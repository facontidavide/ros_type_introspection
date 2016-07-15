#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Imu.h>

#include <ros_type_introspection/parser.hpp>

using namespace ros::message_traits;
using namespace ROSIntrospection;

TEST_CASE("builtin_int32", "ROSType")
{
    ROSType f("int32");

    REQUIRE(f.baseName() =="int32");
    REQUIRE(f.msgName() == "int32");
    REQUIRE(f.pkgName().size() == 0);
    REQUIRE(f.isArray() == false);
    REQUIRE(f.isBuiltin() == true);
    REQUIRE(f.arraySize() == 1);
    REQUIRE(f.typeSize() == 4);
}

TEST_CASE( "builtin_string", "ROSType")
{
    ROSType f("string");
    REQUIRE(f.baseName() == "string");
    REQUIRE(f.msgName()  == "string");
    REQUIRE(f.pkgName().size() == 0);
    REQUIRE(f.isArray() == false);
    REQUIRE(f.isBuiltin() == true);
    REQUIRE(f.arraySize() == 1);
    REQUIRE(f.typeSize() == -1);
}


TEST_CASE("builtin_fixedlen_array", "ROSType")
{
    ROSType f("float64[32]");

    REQUIRE(f.baseName() == "float64[32]");
    REQUIRE(f.msgName() == "float64");
    REQUIRE(f.pkgName().size() == 0 );
    REQUIRE(f.isArray() == true);
    REQUIRE(f.isBuiltin() == true);
    REQUIRE(f.arraySize() == 32);
    REQUIRE(f.typeSize() == 8);
}


TEST_CASE("no_builtin_array", "ROSType")
{
    ROSType f("geometry_msgs/Pose[]");

    REQUIRE(f.baseName() == "geometry_msgs/Pose[]" );
    REQUIRE(f.msgName() == "Pose" );
    REQUIRE(f.pkgName() == "geometry_msgs" );
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
  REQUIRE( mt.type.baseName() == "geometry_msgs/Quaternion" );

  REQUIRE( mt.fields.size() == 2);
  REQUIRE(mt.fields[0].type().msgName() == "float64");
  REQUIRE(mt.fields[1].type().msgName() == "float64");
  REQUIRE(mt.fields[0].name() == "x");
  REQUIRE(mt.fields[1].name() == "z");

  REQUIRE( mt.fields[0].isConstant() == false);
  REQUIRE( mt.fields[1].isConstant() == false);
}

TEST_CASE("constant_uint8", "ROSMessageFields")
{
  ROSMessage msg("uint8 a = 66\n");

  REQUIRE(msg.fields.size() == 1);
  REQUIRE( msg.fields[0].name() == "a" );
  REQUIRE( msg.fields[0].type().baseName() == "uint8" );
  REQUIRE( msg.fields[0].isConstant() == true);
  REQUIRE(msg.fields[0].value() == "66");
}

TEST_CASE("constant_example_navstatus", "ROSMessageFields")
{
    ROSMessage msg( Definition<sensor_msgs::NavSatStatus >::value() );

    REQUIRE( msg.fields.size() == 10);

    REQUIRE( msg.fields[0].name() == ("STATUS_NO_FIX") );
    REQUIRE( msg.fields[0].type().baseName() == ("int8"));
    REQUIRE( msg.fields[0].value()  == ("-1"));

    REQUIRE( msg.fields[1].name() == ("STATUS_FIX") );
    REQUIRE( msg.fields[1].type().baseName() == ("int8"));
    REQUIRE( msg.fields[1].value()  == ("0"));

    REQUIRE( msg.fields[2].name() == ("STATUS_SBAS_FIX") );
    REQUIRE( msg.fields[2].type().baseName() == ("int8"));
    REQUIRE( msg.fields[2].value()  == ("1"));

    REQUIRE( msg.fields[3].name() == ("STATUS_GBAS_FIX") );
    REQUIRE( msg.fields[3].type().baseName() == ("int8"));
    REQUIRE( msg.fields[3].value()  == ("2"));

    REQUIRE( msg.fields[4].name() == ("status") );
    REQUIRE( msg.fields[4].type().baseName() == ("int8"));
    REQUIRE( msg.fields[4].isConstant()  == false);

    REQUIRE( msg.fields[5].name() == ("SERVICE_GPS") );
    REQUIRE( msg.fields[5].type().baseName() == ("uint16"));
    REQUIRE( msg.fields[5].value()  == ("1"));

    REQUIRE( msg.fields[6].name() == ("SERVICE_GLONASS") );
    REQUIRE( msg.fields[6].type().baseName() == ("uint16"));
    REQUIRE( msg.fields[6].value()  == ("2"));

    REQUIRE( msg.fields[7].name() == ("SERVICE_COMPASS") );
    REQUIRE( msg.fields[7].type().baseName() == ("uint16"));
    REQUIRE( msg.fields[7].value()  == ("4"));

    REQUIRE( msg.fields[8].name() == ("SERVICE_GALILEO") );
    REQUIRE( msg.fields[8].type().baseName() == ("uint16"));
    REQUIRE( msg.fields[8].value()  == ("8"));

    REQUIRE( msg.fields[9].name() == ("service") );
    REQUIRE( msg.fields[9].type().baseName() == ("uint16"));
    REQUIRE( msg.fields[9].isConstant()  == false);
}


TEST_CASE("constant_comments", "ROSMessageFields")
{
  ROSMessage msg(
    "string strA=  this string has a # comment in it  \n"
    "string strB = this string has \"quotes\" and \\slashes\\ in it\n"
    "float64 a=64.0 # numeric comment\n");


  REQUIRE( msg.fields.size() == 3);
  REQUIRE( ("strA")== msg.fields[0].name());
  REQUIRE( ("string")== msg.fields[0].type().baseName() );
  REQUIRE( msg.fields[0].isConstant() == true);
  REQUIRE( ("this string has a # comment in it") == msg.fields[0].value());

  REQUIRE( ("strB") == msg.fields[1].name());
  REQUIRE( ("string")== msg.fields[1].type().baseName() );
  REQUIRE( msg.fields[1].isConstant() == true);
  REQUIRE( ("this string has \"quotes\" and \\slashes\\ in it") ==  msg.fields[1].value());

  REQUIRE( ("a")== msg.fields[2].name());
  REQUIRE( ("float64")== msg.fields[2].type().baseName() );
  REQUIRE( msg.fields[2].isConstant() == true);
  REQUIRE( ("64.0")== msg.fields[2].value());
}


TEST_CASE( "Test Pose parsing", "buildROSTypeMapFromDefinition" )
{
    ROSIntrospection::ROSTypeList rmap;

    rmap = buildROSTypeMapFromDefinition(
                DataType<geometry_msgs::Pose >::value(),
                Definition<geometry_msgs::Pose >::value());

    ROSMessage& msg = rmap.at(0);
    REQUIRE( msg.type.baseName() == "geometry_msgs/Pose" );
    REQUIRE( msg.fields.size() == 2);
    REQUIRE( msg.fields[0].type().baseName() == "geometry_msgs/Point" );
    REQUIRE( msg.fields[0].name() == "position" );
    REQUIRE( msg.fields[1].type().baseName() == "geometry_msgs/Quaternion" );
    REQUIRE( msg.fields[1].name() == "orientation" );

    msg = rmap.at(1);
    REQUIRE( ("geometry_msgs/Point" ) == msg.type.baseName() );
    REQUIRE( msg.fields.size() == 3);
    REQUIRE( msg.fields[0].type().baseName() == "float64" );
    REQUIRE( msg.fields[0].name() == "x" );
    REQUIRE( msg.fields[1].type().baseName() == "float64" );
    REQUIRE( msg.fields[1].name() == "y" );
    REQUIRE( msg.fields[2].type().baseName() == "float64" );
    REQUIRE( msg.fields[2].name() == "z" );

    msg = rmap.at(2);
    REQUIRE( ("geometry_msgs/Quaternion" ) == msg.type.baseName() );
    REQUIRE( msg.fields.size() == 4);
    REQUIRE( msg.fields[0].type().baseName() == "float64" );
    REQUIRE( msg.fields[0].name() == "x" );
    REQUIRE( msg.fields[1].type().baseName()  == "float64" );
    REQUIRE( msg.fields[1].name() == "y" );
    REQUIRE( msg.fields[2].type().baseName()  == "float64" );
    REQUIRE( msg.fields[2].name() == "z" );
    REQUIRE( msg.fields[3].type().baseName()  == "float64" );
    REQUIRE( msg.fields[3].name() == "w" );
}

TEST_CASE( "Test IMU parsing", "buildROSTypeMapFromDefinition" )
{
    ROSIntrospection::ROSTypeList rmap;

    rmap = buildROSTypeMapFromDefinition(
                DataType<sensor_msgs::Imu >::value(),
                Definition<sensor_msgs::Imu >::value());

    ROSMessage& msg = rmap.at(0);
    REQUIRE( ("sensor_msgs/Imu") == msg.type.baseName() );
    REQUIRE( msg.fields.size() == 7);
    REQUIRE( ("std_msgs/Header" ) == msg.fields[0].type().baseName() );
    REQUIRE( ("header" )          == msg.fields[0].name() );

    REQUIRE( ("geometry_msgs/Quaternion" ) == msg.fields[1].type().baseName() );
    REQUIRE( ("orientation" )              == msg.fields[1].name() );

    REQUIRE( ("float64[9]" )              == msg.fields[2].type().baseName() );
    REQUIRE( ("orientation_covariance" )  == msg.fields[2].name() );
    REQUIRE( msg.fields[2].type().arraySize() == 9);

    REQUIRE( ("geometry_msgs/Vector3" ) == msg.fields[3].type().baseName() );
    REQUIRE( ("angular_velocity" )      == msg.fields[3].name() );

    REQUIRE( ("float64[9]" )                  == msg.fields[4].type().baseName() );
    REQUIRE( ("angular_velocity_covariance" ) == msg.fields[4].name() );
    REQUIRE( msg.fields[4].type().arraySize() == 9);

    REQUIRE( ("geometry_msgs/Vector3" ) == msg.fields[5].type().baseName() );
    REQUIRE( ("linear_acceleration" )   == msg.fields[5].name() );

    REQUIRE( ("float64[9]" )                     == msg.fields[6].type().baseName() );
    REQUIRE( ("linear_acceleration_covariance" ) == msg.fields[6].name() );
    REQUIRE( msg.fields[6].type().arraySize() == 9);

    //---------------------------------
    msg = rmap.at(1);
    REQUIRE( msg.type.baseName() == "std_msgs/Header" );
    REQUIRE( msg.fields.size() == 3);
    REQUIRE( msg.fields[0].type().baseName() == ("uint32" ));
    REQUIRE( msg.fields[0].name() == ("seq") );
    REQUIRE( msg.fields[1].type().baseName() == ("time") );
    REQUIRE( msg.fields[1].name() == "stamp" );
    REQUIRE( msg.fields[2].type().baseName() == "string" );
    REQUIRE( msg.fields[2].name() == "frame_id" );

    msg = rmap.at(2);
    REQUIRE( msg.type.baseName() == ("geometry_msgs/Quaternion") );
    REQUIRE( msg.fields.size() == 4);
    REQUIRE( msg.fields[0].type().baseName() == "float64" );
    REQUIRE( msg.fields[0].name() == "x" );
    REQUIRE( msg.fields[1].type().baseName() == "float64" );
    REQUIRE( msg.fields[1].name() == "y" );
    REQUIRE( msg.fields[2].type().baseName() == "float64" );
    REQUIRE( msg.fields[2].name() == "z" );
    REQUIRE( msg.fields[3].type().baseName() == "float64" );
    REQUIRE( msg.fields[3].name() == "w" );

    msg = rmap.at(3);
    REQUIRE( msg.type.baseName() == ("geometry_msgs/Vector3") );
    REQUIRE( msg.fields.size() == 3);
    REQUIRE( msg.fields[0].type().baseName() == "float64" );
    REQUIRE( msg.fields[0].name() == "x" );
    REQUIRE( msg.fields[1].type().baseName() == "float64" );
    REQUIRE( msg.fields[1].name() == "y" );
    REQUIRE( msg.fields[2].type().baseName() == "float64" );
    REQUIRE( msg.fields[2].name() == "z" );
}

