
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

int main( int argc, char** argv)
{
 /*   String test("Hello");

    String longname("Hello0123456789_0123456789_0123456789");
    std::cout << longname << " is sso "<< longname.isSso()<< std::endl;

    std::cout << test << " is sso "<< test.isSso()<< std::endl;

    test.append("0123456789", 10);
    std::cout << test << " is sso "<< test.isSso()<< std::endl;

    test.append("0123456789", 10);
    std::cout << test << " is sso "<< test.isSso()<< std::endl;


    test.append("0123456789", 10);
    std::cout << test << " is sso "<< test.isSso()<< std::endl;

    test.resize(25);
    std::cout << test << " is sso "<< test.isSso()<< std::endl;

    test.resize(20);
    std::cout << test << " is sso "<< test.isSso()<< std::endl;

    test.resize(50);
    std::cout << test << " is sso "<< test.isSso()<< std::endl;
*/
    RosTypeParser::RosTypeMap type_map;

    buildRosTypeMapFromDefinition(
                DataType<tf::tfMessage >::value(),
                Definition<tf::tfMessage>::value(),
                &type_map );

    std::cout << "------------------------------"  << std::endl;
    std::cout << type_map << std::endl;

    tf::tfMessage tf_msg;

    tf_msg.transforms.resize(6);

    const char* suffix[6] = { "_A", "_B", "_C", "_D" , "_E", "_F"};

    for (int i=0; i< tf_msg.transforms.size() ; i++)
    {
        tf_msg.transforms[i].header.seq = 100+i;
        tf_msg.transforms[i].header.stamp = {1234 + i, 0 };
        tf_msg.transforms[i].header.frame_id = std::string("frame").append(suffix[i]);

        tf_msg.transforms[i].child_frame_id = std::string("child").append(suffix[i]);
        tf_msg.transforms[i].transform.translation.x = 10 +i;
        tf_msg.transforms[i].transform.translation.y = 20 +i;
        tf_msg.transforms[i].transform.translation.z = 30 +i;

        tf_msg.transforms[i].transform.rotation.x = 40 +i;
        tf_msg.transforms[i].transform.rotation.y = 50 +i;
        tf_msg.transforms[i].transform.rotation.z = 60 +i;
        tf_msg.transforms[i].transform.rotation.w = 70 +i;
    }

    std::vector<uint8_t> buffer(64*1024);

    auto start = std::chrono::high_resolution_clock::now();

    for (int i=0; i<50000;i++)
    {
        ros::serialization::OStream stream(buffer.data(), buffer.size());
        ros::serialization::Serializer<tf::tfMessage>::write(stream, tf_msg);

        uint8_t* buffer_ptr = buffer.data();

        RosTypeFlat flat_container = buildRosFlatType(type_map, "tfMessage", "msgTransform", &buffer_ptr);
        applyNameTransform( Rules(), &flat_container );
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = end - start;

    std::cout << "time elapsed: " << std::chrono::duration_cast< std::chrono::milliseconds>( elapsed ).count()  <<   std::endl;

    return 0;
}

