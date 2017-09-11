
#include <topic_tools/shape_shifter.h>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/utility/string_ref.hpp>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <iostream>
#include <chrono>
#include <ros_type_introspection/renamer.hpp>

using namespace ros::message_traits;
using namespace RosIntrospection;


std::vector<SubstitutionRule> Rules()
{
    std::vector<SubstitutionRule> rules;

/*
    rules.push_back( SubstitutionRule( "transforms.#.transform",
                                       "transforms.#.header.frame_id",
                                       "transforms.#" ));

    rules.push_back( SubstitutionRule( "transforms.#.header",
                                       "transforms.#.header.frame_id",
                                       "transforms.#.header" ));*/

    rules.push_back( SubstitutionRule( "position.#", "name.#", "@.position" ));
    rules.push_back( SubstitutionRule( "velocity.#", "name.#", "@.velocity" ));
    rules.push_back( SubstitutionRule( "effort.#",   "name.#", "@.effort"   ));
    return rules;
}


int main( int argc, char** argv)
{

    ROSTypeList type_map =  buildROSTypeMapFromDefinition(
                DataType<sensor_msgs::JointState >::value(),
                Definition<sensor_msgs::JointState>::value() );

    std::cout << "------------------------------"  << std::endl;
    std::cout << type_map << std::endl;

    sensor_msgs::JointState js_msg;

    js_msg.name.resize(6);
    js_msg.position.resize(6);
    js_msg.velocity.resize(6);
    js_msg.effort.resize(6);

    const char* suffix[6] = { "_A", "_B", "_C", "_D" , "_E", "_F"};

    for (int i=0; i< js_msg.name.size() ; i++)
    {
        js_msg.header.seq = 100+i;
        js_msg.header.stamp.sec = 1234;
        js_msg.header.frame_id = std::string("frame").append(suffix[i]);

        js_msg.name[i] = std::string("child").append(suffix[i]);
        js_msg.position[i]  = 10 +i;
        js_msg.velocity[i]  = 20 +i;
        js_msg.effort[i]    = 30 +i;
    }

    std::vector<uint8_t> buffer(64*1024);

    auto start = std::chrono::high_resolution_clock::now();

    ros::serialization::OStream stream(buffer.data(), buffer.size());
    ros::serialization::Serializer<sensor_msgs::JointState>::write(stream, js_msg);
    ROSType main_type (DataType<sensor_msgs::JointState>::value());

    auto rules = Rules();
    RenamedValues renamed_values;
    ROSTypeFlat flat_container;
    for (int i=0; i<100*1000;i++)
    {
        buildRosFlatType(type_map,main_type, "joint_state", buffer.data(), &flat_container, 100);
        applyNameTransform( rules , flat_container , renamed_values);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = end - start;

    std::cout << "time elapsed: " << std::chrono::duration_cast< std::chrono::milliseconds>( elapsed ).count()  <<   std::endl;

    return 0;
}

