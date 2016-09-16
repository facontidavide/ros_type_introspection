
#include <topic_tools/shape_shifter.h>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/utility/string_ref.hpp>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <tf/tfMessage.h>
#include <sstream>
#include <iostream>
#include <chrono>
#include <ros_type_introspection/renamer.hpp>

using namespace ros::message_traits;
using namespace RosIntrospection;





std::vector<SubstitutionRule> Rules()
{
    std::vector<SubstitutionRule> rules;

    SubstitutionRule rule;
    rule.pattern = { "transforms", "#", "transform"};
    rule.location = { "transforms", "#", "header", "frame_id"};
    rule.substitution = { "transforms", "#"};

    rules.push_back( rule );

    rule.pattern = { "transforms", "#", "header"};
    rule.location = { "transforms", "#", "header", "frame_id"};
    rule.substitution = { "transforms", "#", "header"};

    rules.push_back( rule );

  /*  rules.push_back( SubstitutionRule(".position[#]",
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
                                      ".transform.#.header") );*/
    return rules;
}

int main( int argc, char** argv)
{

  ROSTypeList type_map =  buildROSTypeMapFromDefinition(
        DataType<tf::tfMessage >::value(),
        Definition<tf::tfMessage>::value() );

  std::cout << "------------------------------"  << std::endl;
  std::cout << type_map << std::endl;

  tf::tfMessage tf_msg;

  tf_msg.transforms.resize(6);

  const char* suffix[6] = { "_A", "_B", "_C", "_D" , "_E", "_F"};

  for (size_t i=0; i< tf_msg.transforms.size() ; i++)
  {
    tf_msg.transforms[i].header.seq = 100+i;
    tf_msg.transforms[i].header.stamp.sec = 1234;
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

  ros::serialization::OStream stream(buffer.data(), buffer.size());
  ros::serialization::Serializer<tf::tfMessage>::write(stream, tf_msg);

  ROSType main_type (DataType<tf::tfMessage >::value());

  ROSTypeFlat flat_container;
  for (long i=0; i<100*1000;i++)
  {
    uint8_t* buffer_ptr = buffer.data();

    flat_container = buildRosFlatType(type_map,main_type, "msgTransform", &buffer_ptr);
    applyNameTransform( Rules(), &flat_container );
  }

  for(auto& value_leaf: flat_container.renamed_value)
  {
     std::cout << value_leaf.first << " >> " << value_leaf.second << std::endl;
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = end - start;
  std::cout << "time elapsed: " << std::chrono::duration_cast< std::chrono::milliseconds>( elapsed ).count()  <<   std::endl;


  return 0;
}

