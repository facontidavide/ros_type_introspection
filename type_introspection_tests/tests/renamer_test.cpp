#include "config.h"
#include <gtest/gtest.h>

#include <sensor_msgs/JointState.h>
#include "ros_type_introspection/ros_introspection.hpp"

using namespace ros::message_traits;
using namespace RosIntrospection;

/*
TEST(Renamer, DeserializeJointStateAndRename)
{
  std::vector<SubstitutionRule> rules;
  rules.push_back( SubstitutionRule("position.#", "name.#", "@/pos") );
  rules.push_back( SubstitutionRule("velocity.#", "name.#", "@/vel") );
  rules.push_back( SubstitutionRule("effort.#",   "name.#", "@/eff") );

  ROSTypeList type_map = buildROSTypeMapFromDefinition(
        DataType<sensor_msgs::JointState >::value(),
        Definition<sensor_msgs::JointState >::value() );

  sensor_msgs::JointState joint_state;

  joint_state.header.seq = 2016;
  joint_state.header.stamp.sec  = 1234;
  joint_state.header.stamp.nsec = 567*1000*1000;
  joint_state.header.frame_id = "pippo";

  joint_state.name.resize( 3 );
  joint_state.position.resize( 3 );
  joint_state.velocity.resize( 3 );
  joint_state.effort.resize( 3 );

  std::string names[3];
  names[0] = ("hola");
  names[1] = ("ciao");
  names[2] = ("bye");

  for (int i=0; i<3; i++)
  {
    joint_state.name[i] = names[i];
    joint_state.position[i]= 11+i;
    joint_state.velocity[i]= 21+i;
    joint_state.effort[i]= 31+i;
  }

  std::vector<uint8_t> buffer(64*1024);
  ros::serialization::OStream stream(buffer.data(), buffer.size());
  ros::serialization::Serializer<sensor_msgs::JointState>::write(stream, joint_state);

  ROSType main_type( DataType<sensor_msgs::JointState >::value() );

  FlatMessage flat_container;

  buildRosFlatType(type_map, main_type, "JointState", buffer.data(), &flat_container);
  applyNameTransform( rules, &flat_container );

  if(VERBOSE_TEST){
    for(auto&it: renamed_value) {
      std::cout << it.first << " >> " << it.second << std::endl;
    }
  }

  int i = 0;

  EXPECT_EQ( renamed_value[i].first , SString("JointState/hola/pos"));
  EXPECT_EQ( renamed_value[i++].second, 11 );

  EXPECT_EQ( renamed_value[i].first , SString("JointState/ciao/pos"));
  EXPECT_EQ( renamed_value[i++].second, 12 );

  EXPECT_EQ( renamed_value[i].first , SString("JointState/bye/pos"));
  EXPECT_EQ( renamed_value[i++].second, 13 );

  EXPECT_EQ( renamed_value[i].first , SString("JointState/hola/vel"));
  EXPECT_EQ( renamed_value[i++].second, 21 );

  EXPECT_EQ( renamed_value[i].first , SString("JointState/ciao/vel"));
  EXPECT_EQ( renamed_value[i++].second, 22 );

  EXPECT_EQ( renamed_value[i].first , SString("JointState/bye/vel"));
  EXPECT_EQ( renamed_value[i++].second, 23 );

  EXPECT_EQ( renamed_value[i].first , SString("JointState/hola/eff"));
  EXPECT_EQ( renamed_value[i++].second, 31 );

  EXPECT_EQ( renamed_value[i].first , SString("JointState/ciao/eff"));
  EXPECT_EQ( renamed_value[i++].second, 32 );

  EXPECT_EQ( renamed_value[i].first , SString("JointState/bye/eff"));
  EXPECT_EQ( renamed_value[i++].second, 33 );

  EXPECT_EQ( renamed_value[i].first , SString("JointState/header/seq"));
  EXPECT_EQ( renamed_value[i++].second, 2016 );

  EXPECT_EQ( renamed_value[i].first , SString("JointState/header/stamp"));
  EXPECT_EQ( renamed_value[i++].second, 1234.567 );

}
*/

TEST(Renamer2, DeserializeJointStateAndRename)
{
  RosIntrospection::Parser parser;

  std::vector<SubstitutionRule> rules;
  rules.push_back( SubstitutionRule("position.#", "name.#", "@/pos") );
  rules.push_back( SubstitutionRule("velocity.#", "name.#", "@/vel") );
  rules.push_back( SubstitutionRule("effort.#",   "name.#", "@/eff") );

  ROSType main_type( DataType<sensor_msgs::JointState>::value() );

  parser.registerMessageDefinition(
        "JointState", main_type,
        Definition<sensor_msgs::JointState>::value());

  parser.registerRenamingRules( main_type, rules);

  sensor_msgs::JointState joint_state;

  joint_state.header.seq = 2016;
  joint_state.header.stamp.sec  = 1234;
  joint_state.header.stamp.nsec = 567*1000*1000;
  joint_state.header.frame_id = "pippo";

  joint_state.name.resize( 3 );
  joint_state.position.resize( 3 );
  joint_state.velocity.resize( 3 );
  joint_state.effort.resize( 3 );

  std::string names[3];
  names[0] = ("hola");
  names[1] = ("ciao");
  names[2] = ("bye");

  for (int i=0; i<3; i++)
  {
    joint_state.name[i] = names[i];
    joint_state.position[i]= 11+i;
    joint_state.velocity[i]= 21+i;
    joint_state.effort[i]= 31+i;
  }

  std::vector<uint8_t> buffer( ros::serialization::serializationLength(joint_state) );
  ros::serialization::OStream stream(buffer.data(), buffer.size());
  ros::serialization::Serializer<sensor_msgs::JointState>::write(stream, joint_state);

  FlatMessage flat_container;
  RenamedValues renamed_value;

  parser.deserializeIntoFlatContainer("JointState",  absl::Span<uint8_t>(buffer),  &flat_container,100);
  parser.applyNameTransform("JointState",  flat_container, &renamed_value);

  if(VERBOSE_TEST)
  {
    std::cout << "----------------\n";
    for(auto&it: flat_container.value) {
      std::cout << it.first.toStdString() << " >> " << it.second.convert<double>() << std::endl;
    }
    std::cout << "----------------\n";
    for(auto&it: renamed_value) {
      std::cout << it.first << " >> " << it.second.convert<double>() << std::endl;
    }
    std::cout << "----------------\n";
  }

  int i = 0;

  EXPECT_EQ( renamed_value[i].first , ("JointState/header/seq"));
  EXPECT_EQ( renamed_value[i++].second.convert<double>(), 2016 );

  EXPECT_EQ( renamed_value[i].first , ("JointState/header/stamp"));
  EXPECT_EQ( renamed_value[i++].second.convert<double>(), 1234.567 );

  EXPECT_EQ( renamed_value[i].first , ("JointState/hola/pos"));
  EXPECT_EQ( renamed_value[i++].second.convert<double>(), 11 );

  EXPECT_EQ( renamed_value[i].first , ("JointState/ciao/pos"));
  EXPECT_EQ( renamed_value[i++].second.convert<double>(), 12 );

  EXPECT_EQ( renamed_value[i].first , ("JointState/bye/pos"));
  EXPECT_EQ( renamed_value[i++].second.convert<double>(), 13 );

  EXPECT_EQ( renamed_value[i].first , ("JointState/hola/vel"));
  EXPECT_EQ( renamed_value[i++].second.convert<double>(), 21 );

  EXPECT_EQ( renamed_value[i].first , ("JointState/ciao/vel"));
  EXPECT_EQ( renamed_value[i++].second.convert<double>(), 22 );

  EXPECT_EQ( renamed_value[i].first , ("JointState/bye/vel"));
  EXPECT_EQ( renamed_value[i++].second.convert<double>(), 23 );

  EXPECT_EQ( renamed_value[i].first , ("JointState/hola/eff"));
  EXPECT_EQ( renamed_value[i++].second.convert<double>(), 31 );

  EXPECT_EQ( renamed_value[i].first , ("JointState/ciao/eff"));
  EXPECT_EQ( renamed_value[i++].second.convert<double>(), 32 );

  EXPECT_EQ( renamed_value[i].first , ("JointState/bye/eff"));
  EXPECT_EQ( renamed_value[i++].second.convert<double>(), 33 );

}

