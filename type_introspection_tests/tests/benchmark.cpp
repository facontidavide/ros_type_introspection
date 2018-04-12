
#include <topic_tools/shape_shifter.h>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/utility/string_ref.hpp>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <iostream>
#include <chrono>
#include <ros_type_introspection/ros_introspection.hpp>


#include <benchmark/benchmark.h>


using namespace ros::message_traits;
using namespace RosIntrospection;


static std::vector<SubstitutionRule> Rules()
{
  std::vector<SubstitutionRule> rules;


    rules.push_back( SubstitutionRule( "transforms.#.transform",
                                       "transforms.#.header.frame_id",
                                       "transforms.#" ));

    rules.push_back( SubstitutionRule( "transforms.#.header",
                                       "transforms.#.header.frame_id",
                                       "transforms.#.header" ));

  rules.push_back( SubstitutionRule( "position.#", "name.#", "@.position" ));
  rules.push_back( SubstitutionRule( "velocity.#", "name.#", "@.velocity" ));
  rules.push_back( SubstitutionRule( "effort.#",   "name.#", "@.effort"   ));
  return rules;
}


static void BM_Joints(benchmark::State& state)
{
  RosIntrospection::Parser parser;

  ROSType main_type(DataType<sensor_msgs::JointState>::value());

  parser.registerMessageDefinition(
        "joint_state",
        main_type,
        Definition<sensor_msgs::JointState>::value());

  parser.registerRenamingRules( main_type, Rules() );

  sensor_msgs::JointState js_msg;

  js_msg.name.resize(6);
  js_msg.position.resize(6);
  js_msg.velocity.resize(6);
  js_msg.effort.resize(6);

  const char* suffix[6] = { "_A", "_B", "_C", "_D" , "_E", "_F"};

  for (size_t i=0; i< js_msg.name.size() ; i++)
  {
    js_msg.header.seq = 100+i;
    js_msg.header.stamp.sec = 1234;
    js_msg.header.frame_id = std::string("frame").append(suffix[i]);

    js_msg.name[i] = std::string("child").append(suffix[i]);
    js_msg.position[i]  = 10 +i;
    js_msg.velocity[i]  = 20 +i;
    js_msg.effort[i]    = 30 +i;
  }

  std::vector<uint8_t> buffer( ros::serialization::serializationLength(js_msg) );
  ros::serialization::OStream stream(buffer.data(), buffer.size());
  ros::serialization::Serializer<sensor_msgs::JointState>::write(stream, js_msg);

  FlatMessage flat_container;
  RenamedValues renamed_values;

  while (state.KeepRunning())
  {
    parser.deserializeIntoFlatContainer("joint_state",  absl::Span<uint8_t>(buffer),  &flat_container, 100);
    parser.applyNameTransform("joint_state", flat_container, &renamed_values );
  }
}

BENCHMARK(BM_Joints);

BENCHMARK_MAIN();

