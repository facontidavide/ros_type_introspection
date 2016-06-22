#include "topic_tools/shape_shifter.h"
#include <boost/algorithm/string.hpp>
#include <boost/serialization/serialization.hpp>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

#include <sstream>
#include <iostream>

#include "ros-type-parser.h"



int main(int argc, char **argv)
{
    using namespace ros::message_traits;
    using namespace RosTypeParser;

    std::cout << "------------------------------"  << std::endl;

    std::vector<Type> type_hierarchy;
    std::vector<Field> flat;

    type_hierarchy = parseRosTypeDescription(
                DataType<geometry_msgs::Pose >::value(),
                Definition<geometry_msgs::Pose >::value()   );

    flat = buildFlatTypeHierarchy( "pose", type_hierarchy,
                                   DataType<geometry_msgs::Pose >::value());

    for (int i=0; i<flat.size(); i++ )
    {
        std::cout << ">> "<< flat[i].field_name <<  " / " << flat[i].type_name  << std::endl;
    }


    std::cout << "------------------------------"  << std::endl;

    type_hierarchy = parseRosTypeDescription(
                DataType<sensor_msgs::JointState >::value(),
                Definition<sensor_msgs::JointState >::value()   );

    flat = buildFlatTypeHierarchy( "joint_state", type_hierarchy,
                                   DataType<sensor_msgs::JointState >::value());

    for (int i=0; i<flat.size(); i++ )
    {
        std::cout << ">> "<< flat[i].field_name <<  "\t/ " << flat[i].type_name  << std::endl;
    }

    geometry_msgs::Pose pose;

    pose.position.x = 1;
    pose.position.y = 2;
    pose.position.z = 3;

    pose.orientation.x = 4;
    pose.orientation.y = 5;
    pose.orientation.z = 6;
    pose.orientation.w = 7;
    /*
    std::vector<uint8_t> buffer;

    ros::serialization::OStream stream(buffer.data(), buffer.size());

    ros::serialization::Serializer< geometry_msgs::Pose>(stream, pose);


*/
    return 0;
}
