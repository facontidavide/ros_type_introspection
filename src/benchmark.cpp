
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

const SString* FindSubstitutionName(const StringElement* pattern_head,
                             const ROSTypeFlat& flat_container,
                             const StringTreeLeaf& leaf )
{
  auto node_ptr = leaf.node_ptr;

  for(auto& it: flat_container.name_id)
  {
    const StringTreeLeaf& name_leaf = it.first;
    bool arrays_eq = true;
    if( name_leaf.array_size != leaf.array_size) arrays_eq = false;
    for (int i=0; i <name_leaf.array_size && arrays_eq; i++)
    {
      arrays_eq = ( name_leaf.index_array[i] == leaf.index_array[i] );
    }

    if( arrays_eq ) // array part is ok, check the pattern
    {
      if( PatternMatch( pattern_head, name_leaf.node_ptr) ) {
        std::cout << " substritution: " << it.second;
        return &(it.second);
      }
    }

  }
  return nullptr;
}


/*
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
}*/

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
  for (long i=0; i<10*1000;i++)
  {
    uint8_t* buffer_ptr = buffer.data();

    flat_container = buildRosFlatType(type_map,main_type, "msgTransform", &buffer_ptr);
    // applyNameTransform( Rules(), &flat_container );
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = end - start;
  std::cout << "time elapsed: " << std::chrono::duration_cast< std::chrono::milliseconds>( elapsed ).count()  <<   std::endl;


  std::cout << " ROOT " <<  flat_container.tree.croot()->value() << std::endl;

  assert( flat_container.tree.croot()->parent() == nullptr);

  std::cout << flat_container.tree << std::endl;

  std::vector<SString> pattern = { "transforms", "#", "transform"};
  std::vector<SString> pattern_location = { "transforms", "#", "header", "frame_id"};
  std::vector<SString> pattern_substitution = { "transforms", "#"};

  std::vector<const StringElement*> patt_heads;
  FindPattern(pattern,0, flat_container.tree.croot(), patt_heads );

  std::vector<const StringElement*> loc_heads;
  FindPattern(pattern_location,0, flat_container.tree.croot(), loc_heads );

  std::cout << flat_container.tree << std::endl;


  for(auto&it: flat_container.value) {
    std::cout << it.first << " >> " << it.second << " ... ";

    for(auto& pattern_head: patt_heads)
    {
      auto& leaf = it.first;
      if( PatternMatch( pattern_head, leaf.node_ptr))   {
        std::cout << " match pattern... ";

        for(auto& location_pattern_head: loc_heads)
        {
            const SString* new_name = FindSubstitutionName(location_pattern_head, flat_container, leaf );
            if( new_name )
            {
              std::cout << " CONCATENATION\n";
              int char_count = 0;
              std::vector<SString> concatenated_name;
              concatenated_name.reserve( 10 );

              const StringElement* node_ptr = leaf.node_ptr;
              while( node_ptr != pattern_head)
              {
                const SString* value = &node_ptr->value();
                concatenated_name.push_back( *value );
                char_count += value->size();
                std::cout << "A: " << *value << std::endl;;
                node_ptr = node_ptr->parent();
              }

              for (int i = pattern_substitution.size()-1; i >= 0; i--)
              {
                const SString* value =& pattern_substitution[i];

                if( value->size()==1 && value->at(0) == '#')
                {
                   value = new_name;
                }

                concatenated_name.push_back( *value );
                char_count += value->size();
                std::cout << "B: " << *value << std::endl;;
              }

              for (int i = 0; i < pattern.size() && node_ptr; i++)
              {
                 node_ptr = node_ptr->parent();
              }

              while( node_ptr )
              {
                const SString* value = &node_ptr->value();
                concatenated_name.push_back( *value );
                char_count += value->size();
                std::cout << "C: " << *value << std::endl;;
                node_ptr = node_ptr->parent();
              }

              //------------------------
              SString new_identifier;
              new_identifier.reserve( char_count + concatenated_name.size() + 1 );

              for (int i = concatenated_name.size()-1; i >= 0; i--)
              {
                new_identifier.append( concatenated_name[i] );
                if( i>0) new_identifier.append(".");
              }
              std::cout << "Result: " << new_identifier << std::endl;;
            }
        }
      }
    }
    std::cout << std::endl;
  }

  std::cout << "time elapsed: " << std::chrono::duration_cast< std::chrono::milliseconds>( elapsed ).count()  <<   std::endl;

  return 0;
}

