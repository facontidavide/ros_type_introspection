#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.h"

#include "topic_tools/shape_shifter.h"
#include <boost/algorithm/string.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/utility/string_ref.hpp>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <tf/tfMessage.h>
#include <sstream>
#include <iostream>
#include <chrono>
#include "ros-type-parser.h"

using namespace ros::message_traits;
using namespace RosTypeParser;

std::vector<SubstitutionRule> Rules()
{
    std::vector<SubstitutionRule> rules;
    rules.push_back( SubstitutionRule(".position[#]",
                                      ".name[#]",
                                      ".#.position") );

    rules.push_back( SubstitutionRule(".velocity[#]",
                                      ".name[#]",
                                      ".#.velocity") );

    rules.push_back( SubstitutionRule(".effort[#]",
                                      ".name[#]",
                                      ".#.effort") );

    rules.push_back( SubstitutionRule(".transforms[#].transform",
                                      ".transforms[#].header.frame_id",
                                      ".transform.#") );

    rules.push_back( SubstitutionRule(".transforms[#].header",
                                      ".transforms[#].header.frame_id",
                                      ".transform.#.header") );
    return rules;
}

void compare( const RosTypeMap& mapA, const RosTypeMap& mapB )
{
    REQUIRE( mapA.size() == mapB.size() );
    for (auto itA = mapA.begin() ; itA != mapA.end(); itA++)
    {
        auto itB = mapB.find( itA->first );
        REQUIRE( itA->first == itB->first );
        REQUIRE( itA->second.full_name == itB->second.full_name );

        auto fieldsA = itA->second.fields;
        auto fieldsB = itB->second.fields;
        REQUIRE( fieldsA.size() == fieldsB.size() );

        for (int i=0; i<fieldsA.size(); i++) {
            REQUIRE( fieldsA[i].type_name  == fieldsB[i].type_name );
            REQUIRE( fieldsA[i].field_name == fieldsB[i].field_name );
        }
    }
}

void compare( const std::map< std::string, double>& flatA, const std::map< std::string, double>& flatB )
{
    REQUIRE( flatA.size() == flatB.size() );

    for (auto itA = flatA.begin() ; itA != flatA.end(); itA++)
    {
        auto itB = flatB.find( itA->first );
        std::cout <<  itA->first << " : " << itA->second << std::endl;

        REQUIRE( itB != flatB.end() );
        REQUIRE( itA->first  == itB->first );
        REQUIRE( itA->second == itB->second );
    }
}


TEST_CASE( "Test Pose parsing", "parseRosTypeDescription" )
{

    RosTypeParser::RosTypeMap type_map;

    parseRosTypeDescription(
                DataType<geometry_msgs::Pose >::value(),
                Definition<geometry_msgs::Pose >::value(),
                &type_map );

    RosTypeParser::RosTypeMap expected_map;

    RosTypeParser::RosType type_pose("geometry_msgs/Pose");
    type_pose.fields.push_back( { "Point", "position" } );
    type_pose.fields.push_back( { "Quaternion", "orientation" } );

    RosTypeParser::RosType type_point("geometry_msgs/Point");
    type_point.fields.push_back( { "float64", "x" } );
    type_point.fields.push_back( { "float64", "y" } );
    type_point.fields.push_back( { "float64", "z" } );

    RosTypeParser::RosType type_rot("geometry_msgs/Quaternion");
    type_rot.fields.push_back( { "float64", "x" } );
    type_rot.fields.push_back( { "float64", "y" } );
    type_rot.fields.push_back( { "float64", "z" } );
    type_rot.fields.push_back( { "float64", "w" } );

    expected_map[ "Pose" ]       = type_pose;
    expected_map[ "Point" ]      = type_point;
    expected_map[ "Quaternion" ] = type_rot;

    std::cout << "------------------------------"  << std::endl;
    compare( type_map, expected_map );
}


TEST_CASE( "Test JointState parsing", "parseRosTypeDescription" )
//int func()
{
    RosTypeParser::RosTypeMap type_map;

    parseRosTypeDescription(
                DataType<std_msgs::Header >::value(),
                Definition<std_msgs::Header >::value(),
                &type_map );

    parseRosTypeDescription(
                DataType<geometry_msgs::Pose >::value(),
                Definition<geometry_msgs::Pose >::value(),
                &type_map );

    parseRosTypeDescription(
                DataType<sensor_msgs::JointState >::value(),
                Definition<sensor_msgs::JointState >::value(),
                &type_map );

    RosTypeParser::RosTypeMap expected_map;

    RosTypeParser::RosType type_pose("geometry_msgs/Pose");
    type_pose.fields.push_back( { "Point", "position" } );
    type_pose.fields.push_back( { "Quaternion", "orientation" } );

    RosTypeParser::RosType type_point("geometry_msgs/Point");
    type_point.fields.push_back( { "float64", "x" } );
    type_point.fields.push_back( { "float64", "y" } );
    type_point.fields.push_back( { "float64", "z" } );

    RosTypeParser::RosType type_rot("geometry_msgs/Quaternion");
    type_rot.fields.push_back( { "float64", "x" } );
    type_rot.fields.push_back( { "float64", "y" } );
    type_rot.fields.push_back( { "float64", "z" } );
    type_rot.fields.push_back( { "float64", "w" } );

    RosTypeParser::RosType type_header("std_msgs/Header");
    type_header.fields.push_back( { "uint32", "seq" } );
    type_header.fields.push_back( { "time",   "stamp" } );
    type_header.fields.push_back( { "string", "frame_id" } );

    RosTypeParser::RosType type_joint("sensor_msgs/JointState");
    type_joint.fields.push_back( { "Header",   "header" } );
    type_joint.fields.push_back( { "string[]",  "name" } );
    type_joint.fields.push_back( { "float64[]", "position" } );
    type_joint.fields.push_back( { "float64[]", "velocity" } );
    type_joint.fields.push_back( { "float64[]", "effort" } );

    expected_map[ "Pose" ]        = type_pose;
    expected_map[ "Point" ]       = type_point;
    expected_map[ "Quaternion" ]  = type_rot;
    expected_map[ "Header" ]      = type_header;
    expected_map[ "JointState" ]  = type_joint;

    std::cout << "------------------------------"  << std::endl;
    compare( type_map, expected_map );
}


TEST_CASE( "Deserialize Pose", "RosType deserialization" )
//int func()
{
    RosTypeParser::RosTypeMap type_map;

    parseRosTypeDescription(
                DataType<geometry_msgs::Pose >::value(),
                Definition<geometry_msgs::Pose >::value(),
                &type_map );

    geometry_msgs::Pose pose;

    pose.position.x = 1;
    pose.position.y = 2;
    pose.position.z = 3;
    pose.orientation.x = 4;
    pose.orientation.y = 5;
    pose.orientation.z = 6;
    pose.orientation.w = 7;

    std::vector<uint8_t> buffer(64*1024);
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    ros::serialization::Serializer< geometry_msgs::Pose>::write(stream, pose);

    RosTypeFlat flat_container;
    uint8_t* buffer_ptr = buffer.data();

    buildRosFlatType(type_map, "Pose", "Pose", &buffer_ptr,  &flat_container);

    std::map< std::string, double > expected_result;
    expected_result["Pose.position.x"] = 1;
    expected_result["Pose.position.y"] = 2;
    expected_result["Pose.position.z"] = 3;
    expected_result["Pose.orientation.x"] = 4;
    expected_result["Pose.orientation.y"] = 5;
    expected_result["Pose.orientation.z"] = 6;
    expected_result["Pose.orientation.w"] = 7;

    applyNameTransform( Rules() , &flat_container );

    for(auto&it: flat_container.value_renamed) {
        std::cout << it.first << " >> " << it.second << std::endl;
    }

    for(auto&it: expected_result) {
        std::cout << it.first << " >>>>> " << it.second << std::endl;
    }

    compare( flat_container.value,         expected_result );
    compare( flat_container.value_renamed, expected_result );


}


TEST_CASE( "Deserialize JointState", "RosType deserialization" )
//int func()
{
    RosTypeParser::RosTypeMap type_map;

    parseRosTypeDescription(
                DataType<sensor_msgs::JointState >::value(),
                Definition<sensor_msgs::JointState >::value(),
                &type_map );


    sensor_msgs::JointState joint_state;

    joint_state.header.seq = 2016;
    joint_state.header.stamp = { 1234, 567*1000*1000 };
    joint_state.header.frame_id = "pippo";

    joint_state.name.resize( 3 );
    joint_state.position.resize( 3 );
    joint_state.velocity.resize( 3 );
    joint_state.effort.resize( 3 );

    std::string names[3];
    names[0] = std::string("hola");
    names[1] = std::string("ciao");
    names[2] = std::string("bye");

    for (int i=0; i<3; i++)
    {
        joint_state.name[i] = names[i];
        joint_state.position[i]= 11+i;
        joint_state.velocity[i]= 21+i;
        joint_state.effort[i]= 31+i;
    }

    std::map< std::string, double > expected_result;
    expected_result["JointState.header.seq"] = 2016;
    expected_result["JointState.header.stamp"] = 1234.567;

    expected_result["JointState.hola.position"] = 11;
    expected_result["JointState.hola.velocity"] = 21;
    expected_result["JointState.hola.effort"]   = 31;

    expected_result["JointState.ciao.position"] = 12;
    expected_result["JointState.ciao.velocity"] = 22;
    expected_result["JointState.ciao.effort"]   = 32;

    expected_result["JointState.bye.position"] = 13;
    expected_result["JointState.bye.velocity"] = 23;
    expected_result["JointState.bye.effort"]   = 33;

    std::vector<uint8_t> buffer(64*1024);
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    ros::serialization::Serializer<sensor_msgs::JointState>::write(stream, joint_state);

    RosTypeFlat flat_container;
    uint8_t* buffer_ptr = buffer.data();

    buildRosFlatType(type_map, "JointState", "JointState", &buffer_ptr,  &flat_container);
    applyNameTransform( Rules(), &flat_container );


    for(auto&it: flat_container.value_renamed) {
        std::cout << it.first << " >> " << it.second << std::endl;
    }

    for(auto&it: expected_result) {
        std::cout << it.first << " >>>>> " << it.second << std::endl;
    }

    //---------------------------------------------------
    {
        auto& flatA =  flat_container.value_renamed;
        auto& flatB = expected_result;
        REQUIRE( flatA.size() == flatB.size() );

        for (auto itA = flatA.begin() ; itA != flatA.end(); itA++)
        {
            auto itB = flatB.find( itA->first );
            std::cout <<  itA->first << " : " << itA->second << std::endl;

            REQUIRE( itB != flatB.end() );
            REQUIRE( itA->first  == itB->first );
            REQUIRE( itA->second == itB->second );
        }
    }
    //---------------------------------------------------

    // repeat. Nothing should change
    buffer_ptr = buffer.data();
    buildRosFlatType(type_map, "JointState", "JointState", &buffer_ptr,  &flat_container);
    applyNameTransform( Rules() , &flat_container );


    //---------------------------------------------------
    {
        auto& flatA =  flat_container.value_renamed;
        auto& flatB = expected_result;
        REQUIRE( flatA.size() == flatB.size() );

        for (auto itA = flatA.begin() ; itA != flatA.end(); itA++)
        {
            auto itB = flatB.find( itA->first );
            std::cout <<  itA->first << " : " << itA->second << std::endl;

            REQUIRE( itB != flatB.end() );
            REQUIRE( itA->first  == itB->first );
            REQUIRE( itA->second == itB->second );
        }
    }
    //---------------------------------------------------
}


TEST_CASE( "Deserialize Transform", "RosType deserialization" )
//int func()
{
    RosTypeParser::RosTypeMap type_map;

    parseRosTypeDescription(
                DataType<tf::tfMessage >::value(),
                Definition<tf::tfMessage>::value(),
                &type_map );

    std::cout << "------------------------------"  << std::endl;
    printRosTypeMap( type_map );

    tf::tfMessage tf_msg;

    tf_msg.transforms.resize(1);

    const char* suffix[3] = { "_A", "_B", "_C" };

    std::map< std::string, double > expected_result;

    for (int i=0; i< tf_msg.transforms.size() ; i++)
    {
        tf_msg.transforms[i].header.seq = 100+i;
        tf_msg.transforms[i].header.stamp = {1234 + i, 0 };
        tf_msg.transforms[i].header.frame_id = std::string("frame").append(suffix[i]);

        tf_msg.transforms[i].child_frame_id = std::string("child").append(suffix[i]);

        std::string prefix = std::string( "msgTransform.transform.frame" ).append(suffix[i]);

        expected_result[ prefix + (".header.seq")   ] = tf_msg.transforms[i].header.seq;
        expected_result[ prefix + (".header.stamp") ] = 1234 + i;

        tf_msg.transforms[i].transform.translation.x = 10 +i;
        tf_msg.transforms[i].transform.translation.y = 20 +i;
        tf_msg.transforms[i].transform.translation.z = 30 +i;

        tf_msg.transforms[i].transform.rotation.x = 40 +i;
        tf_msg.transforms[i].transform.rotation.y = 50 +i;
        tf_msg.transforms[i].transform.rotation.z = 60 +i;
        tf_msg.transforms[i].transform.rotation.w = 70 +i;

        expected_result[ prefix + (".translation.x") ] = tf_msg.transforms[i].transform.translation.x;
        expected_result[ prefix + (".translation.y") ] = tf_msg.transforms[i].transform.translation.y;
        expected_result[ prefix + (".translation.z") ] = tf_msg.transforms[i].transform.translation.z;

        expected_result[ prefix + (".rotation.x") ] = tf_msg.transforms[i].transform.rotation.x;
        expected_result[ prefix + (".rotation.y") ] = tf_msg.transforms[i].transform.rotation.y;
        expected_result[ prefix + (".rotation.z") ] = tf_msg.transforms[i].transform.rotation.z;
        expected_result[ prefix + (".rotation.w") ] = tf_msg.transforms[i].transform.rotation.w;

    }

    std::vector<uint8_t> buffer(64*1024);
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    ros::serialization::Serializer<tf::tfMessage>::write(stream, tf_msg);

    RosTypeFlat flat_container;
    uint8_t* buffer_ptr = buffer.data();

    buildRosFlatType(type_map, "tfMessage", "msgTransform", &buffer_ptr,  &flat_container);
    applyNameTransform( Rules(), &flat_container );

//    std::cout <<  "---------------------------" << std::endl;
//    for(auto&it: flat_container.value_renamed) {
//        std::cout << it.first << " >> " << it.second << std::endl;
//    }
//    std::cout <<  "---------------------------" << std::endl;
//    for(auto&it: expected_result) {
//        std::cout << it.first << " >>>>> " << it.second << std::endl;
//    }

    { //---------------------------------------------------
        auto& flatA =  flat_container.value_renamed;
        auto& flatB = expected_result;
        REQUIRE( flatA.size() == flatB.size() );

        for (auto itA = flatA.begin() ; itA != flatA.end(); itA++)
        {
            auto itB = flatB.find( itA->first );
            std::cout <<  itA->first << " : " << itA->second << std::endl;

            REQUIRE( itB != flatB.end() );
            REQUIRE( itA->first  == itB->first );
            REQUIRE( itA->second == itB->second );
        }
    }//---------------------------------------------------

}


