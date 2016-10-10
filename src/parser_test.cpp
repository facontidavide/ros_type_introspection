#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch.h>

    #include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16MultiArray.h>
#include <ros_type_introspection/parser.hpp>

using namespace ros::message_traits;
using namespace RosIntrospection;

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
  REQUIRE( mt.type().baseName() == "geometry_msgs/Quaternion" );

  REQUIRE( mt.fields().size() == 2);
  REQUIRE(mt.field(0).type().msgName() == "float64");
  REQUIRE(mt.field(1).type().msgName() == "float64");
  REQUIRE(mt.field(0).name() == "x");
  REQUIRE(mt.field(1).name() == "z");

  REQUIRE( mt.field(0).isConstant() == false);
  REQUIRE( mt.field(1).isConstant() == false);
}

TEST_CASE("constant_uint8", "ROSMessageFields")
{
  ROSMessage msg("uint8 a = 66\n");

  REQUIRE(msg.fields().size() == 1);
  REQUIRE( msg.field(0).name() == "a" );
  REQUIRE( msg.field(0).type().baseName() == "uint8" );
  REQUIRE( msg.field(0).isConstant() == true);
  REQUIRE(msg.field(0).value() == "66");
}

TEST_CASE("constant_example_navstatus", "ROSMessageFields")
{
    ROSMessage msg( Definition<sensor_msgs::NavSatStatus >::value() );

    REQUIRE( msg.fields().size() == 10);

    REQUIRE( msg.field(0).name() == ("STATUS_NO_FIX") );
    REQUIRE( msg.field(0).type().baseName() == ("int8"));
    REQUIRE( msg.field(0).value()  == ("-1"));

    REQUIRE( msg.field(1).name() == ("STATUS_FIX") );
    REQUIRE( msg.field(1).type().baseName() == ("int8"));
    REQUIRE( msg.field(1).value()  == ("0"));

    REQUIRE( msg.field(2).name() == ("STATUS_SBAS_FIX") );
    REQUIRE( msg.field(2).type().baseName() == ("int8"));
    REQUIRE( msg.field(2).value()  == ("1"));

    REQUIRE( msg.field(3).name() == ("STATUS_GBAS_FIX") );
    REQUIRE( msg.field(3).type().baseName() == ("int8"));
    REQUIRE( msg.field(3).value()  == ("2"));

    REQUIRE( msg.field(4).name() == ("status") );
    REQUIRE( msg.field(4).type().baseName() == ("int8"));
    REQUIRE( msg.field(4).isConstant()  == false);

    REQUIRE( msg.field(5).name() == ("SERVICE_GPS") );
    REQUIRE( msg.field(5).type().baseName() == ("uint16"));
    REQUIRE( msg.field(5).value()  == ("1"));

    REQUIRE( msg.field(6).name() == ("SERVICE_GLONASS") );
    REQUIRE( msg.field(6).type().baseName() == ("uint16"));
    REQUIRE( msg.field(6).value()  == ("2"));

    REQUIRE( msg.field(7).name() == ("SERVICE_COMPASS") );
    REQUIRE( msg.field(7).type().baseName() == ("uint16"));
    REQUIRE( msg.field(7).value()  == ("4"));

    REQUIRE( msg.field(8).name() == ("SERVICE_GALILEO") );
    REQUIRE( msg.field(8).type().baseName() == ("uint16"));
    REQUIRE( msg.field(8).value()  == ("8"));

    REQUIRE( msg.field(9).name() == ("service") );
    REQUIRE( msg.field(9).type().baseName() == ("uint16"));
    REQUIRE( msg.field(9).isConstant()  == false);
}


TEST_CASE("constant_comments", "ROSMessageFields")
{
  ROSMessage msg(
    "string strA=  this string has a # comment in it  \n"
    "string strB = this string has \"quotes\" and \\slashes\\ in it\n"
    "float64 a=64.0 # numeric comment\n");


  REQUIRE( msg.fields().size() == 3);
  REQUIRE( ("strA")== msg.field(0).name());
  REQUIRE( ("string")== msg.field(0).type().baseName() );
  REQUIRE( msg.field(0).isConstant() == true);
  REQUIRE( ("this string has a # comment in it") == msg.field(0).value());

  REQUIRE( ("strB") == msg.field(1).name());
  REQUIRE( ("string")== msg.field(1).type().baseName() );
  REQUIRE( msg.field(1).isConstant() == true);
  REQUIRE( ("this string has \"quotes\" and \\slashes\\ in it") ==  msg.field(1).value());

  REQUIRE( ("a")== msg.field(2).name());
  REQUIRE( ("float64")== msg.field(2).type().baseName() );
  REQUIRE( msg.field(2).isConstant() == true);
  REQUIRE( ("64.0")== msg.field(2).value());
}


TEST_CASE( "Test Pose parsing", "buildROSTypeMapFromDefinition" )
{
    RosIntrospection::ROSTypeList rmap;

    rmap = buildROSTypeMapFromDefinition(
                DataType<geometry_msgs::Pose >::value(),
                Definition<geometry_msgs::Pose >::value());

    std::cout << rmap << std::endl;

    ROSMessage& msg = rmap.at(0);
    REQUIRE( msg.type().baseName() == "geometry_msgs/Pose" );
    REQUIRE( msg.fields().size() == 2);
    REQUIRE( msg.field(0).type().baseName() == "geometry_msgs/Point" );
    REQUIRE( msg.field(0).name() == "position" );
    REQUIRE( msg.field(1).type().baseName() == "geometry_msgs/Quaternion" );
    REQUIRE( msg.field(1).name() == "orientation" );

    msg = rmap.at(1);
    REQUIRE( ("geometry_msgs/Point" ) == msg.type().baseName() );
    REQUIRE( msg.fields().size() == 3);
    REQUIRE( msg.field(0).type().baseName() == "float64" );
    REQUIRE( msg.field(0).name() == "x" );
    REQUIRE( msg.field(1).type().baseName() == "float64" );
    REQUIRE( msg.field(1).name() == "y" );
    REQUIRE( msg.field(2).type().baseName() == "float64" );
    REQUIRE( msg.field(2).name() == "z" );

    msg = rmap.at(2);
    REQUIRE( ("geometry_msgs/Quaternion" ) == msg.type().baseName() );
    REQUIRE( msg.fields().size() == 4);
    REQUIRE( msg.field(0).type().baseName() == "float64" );
    REQUIRE( msg.field(0).name() == "x" );
    REQUIRE( msg.field(1).type().baseName()  == "float64" );
    REQUIRE( msg.field(1).name() == "y" );
    REQUIRE( msg.field(2).type().baseName()  == "float64" );
    REQUIRE( msg.field(2).name() == "z" );
    REQUIRE( msg.field(3).type().baseName()  == "float64" );
    REQUIRE( msg.field(3).name() == "w" );
}

TEST_CASE( "Test IMU parsing", "buildROSTypeMapFromDefinition" )
{
    RosIntrospection::ROSTypeList rmap;

    rmap = buildROSTypeMapFromDefinition(
                DataType<sensor_msgs::Imu >::value(),
                Definition<sensor_msgs::Imu >::value());

    std::cout << rmap << std::endl;

    ROSMessage& msg = rmap.at(0);
    REQUIRE( ("sensor_msgs/Imu") == msg.type().baseName() );
    REQUIRE( msg.fields().size() == 7);
    REQUIRE( ("std_msgs/Header" ) == msg.field(0).type().baseName() );
    REQUIRE( ("header" )          == msg.field(0).name() );

    REQUIRE( ("geometry_msgs/Quaternion" ) == msg.field(1).type().baseName() );
    REQUIRE( ("orientation" )              == msg.field(1).name() );

    REQUIRE( ("float64[9]" )              == msg.field(2).type().baseName() );
    REQUIRE( ("orientation_covariance" )  == msg.field(2).name() );
    REQUIRE( msg.field(2).type().arraySize() == 9);

    REQUIRE( ("geometry_msgs/Vector3" ) == msg.field(3).type().baseName() );
    REQUIRE( ("angular_velocity" )      == msg.field(3).name() );

    REQUIRE( ("float64[9]" )                  == msg.field(4).type().baseName() );
    REQUIRE( ("angular_velocity_covariance" ) == msg.field(4).name() );
    REQUIRE( msg.field(4).type().arraySize() == 9);

    REQUIRE( ("geometry_msgs/Vector3" ) == msg.field(5).type().baseName() );
    REQUIRE( ("linear_acceleration" )   == msg.field(5).name() );

    REQUIRE( ("float64[9]" )                     == msg.field(6).type().baseName() );
    REQUIRE( ("linear_acceleration_covariance" ) == msg.field(6).name() );
    REQUIRE( msg.field(6).type().arraySize() == 9);


    //---------------------------------
    msg = rmap.at(1);
    REQUIRE( msg.type().baseName() == "std_msgs/Header" );
    REQUIRE( msg.fields().size() == 3);
    REQUIRE( msg.field(0).type().baseName() == ("uint32" ));
    REQUIRE( msg.field(0).name() == ("seq") );
    REQUIRE( msg.field(1).type().baseName() == ("time") );
    REQUIRE( msg.field(1).name() == "stamp" );
    REQUIRE( msg.field(2).type().baseName() == "string" );
    REQUIRE( msg.field(2).name() == "frame_id" );

    msg = rmap.at(2);
    REQUIRE( msg.type().baseName() == ("geometry_msgs/Quaternion") );
    REQUIRE( msg.fields().size() == 4);
    REQUIRE( msg.field(0).type().baseName() == "float64" );
    REQUIRE( msg.field(0).name() == "x" );
    REQUIRE( msg.field(1).type().baseName() == "float64" );
    REQUIRE( msg.field(1).name() == "y" );
    REQUIRE( msg.field(2).type().baseName() == "float64" );
    REQUIRE( msg.field(2).name() == "z" );
    REQUIRE( msg.field(3).type().baseName() == "float64" );
    REQUIRE( msg.field(3).name() == "w" );

    msg = rmap.at(3);
    REQUIRE( msg.type().baseName() == ("geometry_msgs/Vector3") );
    REQUIRE( msg.fields().size() == 3);
    REQUIRE( msg.field(0).type().baseName() == "float64" );
    REQUIRE( msg.field(0).name() == "x" );
    REQUIRE( msg.field(1).type().baseName() == "float64" );
    REQUIRE( msg.field(1).name() == "y" );
    REQUIRE( msg.field(2).type().baseName() == "float64" );
    REQUIRE( msg.field(2).name() == "z" );
}

TEST_CASE( "Test Int16MultiArray parsing", "buildROSTypeMapFromDefinition" )
{
    // this test case was added because it previously failed to detect nested
    // arrays of custom types. In this case:
    //    std_msgs/MultiArrayDimension[]

    RosIntrospection::ROSTypeList rmap;

    rmap = buildROSTypeMapFromDefinition(
                DataType<std_msgs::Int16MultiArray >::value(),
                Definition<std_msgs::Int16MultiArray >::value());

    std::cout << rmap << std::endl;

    /*
    std_msgs/Int16MultiArray :
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

    REQUIRE( ("std_msgs/Int16MultiArray") == msg.type().baseName() );
    REQUIRE( msg.fields().size() == 2);
    REQUIRE( ("std_msgs/MultiArrayLayout" ) == msg.field(0).type().baseName() );
    REQUIRE( ("layout" )                    == msg.field(0).name() );
    REQUIRE( false == msg.field(0).type().isArray() );

    REQUIRE( ("int16[]" ) == msg.field(1).type().baseName() );
    REQUIRE( ("data" )  == msg.field(1).name() );
    REQUIRE( true == msg.field(1).type().isArray() );

    msg = rmap.at(1);
    REQUIRE( ("std_msgs/MultiArrayLayout") == msg.type().baseName() );
    REQUIRE( msg.fields().size() == 2);
    REQUIRE( ("std_msgs/MultiArrayDimension[]" ) == msg.field(0).type().baseName() );
    REQUIRE( ("dim" )                            == msg.field(0).name() );
    REQUIRE( true == msg.field(0).type().isArray() );

    REQUIRE( ("uint32" )       == msg.field(1).type().baseName() );
    REQUIRE( ("data_offset" )  == msg.field(1).name() );
    REQUIRE( false == msg.field(1).type().isArray() );


    msg = rmap.at(2);
    REQUIRE( ("std_msgs/MultiArrayDimension") == msg.type().baseName() );
    REQUIRE( msg.fields().size() == 3);

    REQUIRE( ("string" ) == msg.field(0).type().baseName() );
    REQUIRE( ("label" )   == msg.field(0).name() );
    REQUIRE( false == msg.field(0).type().isArray() );

    REQUIRE( ("uint32" ) == msg.field(1).type().baseName() );
    REQUIRE( ("size" )   == msg.field(1).name() );
    REQUIRE( false == msg.field(1).type().isArray() );

    REQUIRE( ("uint32" )  == msg.field(2).type().baseName() );
    REQUIRE( ("stride" )  == msg.field(2).name() );
    REQUIRE( false == msg.field(2).type().isArray() );

}

