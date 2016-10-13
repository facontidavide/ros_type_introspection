#ifndef ROS_INTRO_DESERIALIZE_H
#define ROS_INTRO_DESERIALIZE_H

#include <ros_type_introspection/parser.hpp>
#include <ros_type_introspection/stringtree.h>
#include <sstream>

namespace RosIntrospection{

class StringTreeLeaf{
  friend std::ostream& operator<<(std::ostream &os, const StringTreeLeaf& leaf );
public:
  StringTreeLeaf(): node_ptr(nullptr), array_size(0)
  {  for (int i=0; i<7; i++) index_array[i] = 0;}

  StringTreeNode* node_ptr;
  uint8_t array_size;
  std::array<uint16_t,7> index_array;

  SString toStr() const;
};

typedef struct{
  StringTree tree;
  std::vector< std::pair<StringTreeLeaf, double> > value;
  std::vector< std::pair<StringTreeLeaf, SString> > name;
  std::vector< std::pair<SString, double> > renamed_value;
}ROSTypeFlat;

void buildRosFlatType(const ROSTypeList& type_map,
                      ROSType type,
                      SString prefix,
                      uint8_t **buffer_ptr,
                      ROSTypeFlat* flat_container_output,
                      uint16_t max_array_size = 1000);


inline std::ostream& operator<<(std::ostream &os, const StringTreeLeaf& leaf )
{
  os << leaf.toStr();
  return os;
}

} //end namespace

#endif // ROS_INTRO_DESERIALIZE_H
