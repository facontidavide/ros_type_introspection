#include "topic_tools/shape_shifter.h"
#include <boost/algorithm/string.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/utility/string_ref.hpp>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <tf/tfMessage.h>
#include <sstream>
#include <iostream>

#include "ros-type-parser.h"



int main(int argc, char **argv)
{
    using namespace ros::message_traits;
    using namespace RosTypeParser;

    RosTypeMap type_map;

    parseRosTypeDescription(
                DataType<geometry_msgs::Pose >::value(),
                Definition<geometry_msgs::Pose >::value(),
                &type_map );

    parseRosTypeDescription(
                DataType<sensor_msgs::JointState >::value(),
                Definition<sensor_msgs::JointState >::value(),
                &type_map );

    parseRosTypeDescription(
                DataType<tf::tfMessage >::value(),
                Definition<tf::tfMessage >::value(),
                &type_map );

    std::cout << "------------------------------"  << std::endl;
    printRosTypeMap( type_map );

    std::cout << "------------------------------"  << std::endl;
    //   printRosType( type_map, "Pose");

    std::cout << "------------------------------"  << std::endl;
    //  printRosType( type_map, "JointState");

    std::cout << "------------------------------"  << std::endl;
    printRosType( type_map, "tfMessage");

    /* geometry_msgs::Pose pose;

    pose.position.x = 1;
    pose.position.y = 2;
    pose.position.z = 3;

    pose.orientation.x = 4;
    pose.orientation.y = 5;
    pose.orientation.z = 6;
    pose.orientation.w = 7;

    {
        std::vector<uint8_t> buffer(64*1024);
        ros::serialization::OStream stream(buffer.data(), buffer.size());
        ros::serialization::Serializer< geometry_msgs::Pose>::write(stream, pose);

        RosTypeFlat flat_container;
        uint8_t* buffer_ptr = buffer.data();

        buildOffsetTable(type_map, "Pose", "Pose", &buffer_ptr,  &flat_container);

                 std::cout<< flat_container << std::endl;
    }

    std::cout << "------------------------------"  << std::endl;

*/
    sensor_msgs::JointState joint_state;

    joint_state.header.seq = 2016;
    joint_state.header.stamp = { 1234, 5678};
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

    {
        std::vector<uint8_t> buffer(64*1024);
        ros::serialization::OStream stream(buffer.data(), buffer.size());
        ros::serialization::Serializer<sensor_msgs::JointState>::write(stream, joint_state);

        RosTypeFlat flat_container;
        uint8_t* buffer_ptr = buffer.data();

        buildOffsetTable(type_map, "JointState", "JointState", &buffer_ptr,  &flat_container);

        std::cout << "------------------------------"  << std::endl;
        std::cout<< flat_container << std::endl;

        //-------------------------------------------------------
        std::vector< std::pair<const char*, const char*> > rules;
        rules.push_back( std::make_pair( "JointState.position[#]", "JointState.name[#]") );
        rules.push_back( std::make_pair( "JointState.effort[#]", "JointState.name[#]") );
        rules.push_back( std::make_pair( "JointState.velocity[#]", "JointState.name[#]") );
        rules.push_back( std::make_pair( "TransformStamped[#]", "TransformStamped[#].Header.frame_id") );

        RosTypeFlat renamed_container;
        applyNameTransform( rules, flat_container, &renamed_container );

        std::cout << "------------------------------"  << std::endl;
         std::cout<< renamed_container << std::endl;

    }


    return 0;
}
