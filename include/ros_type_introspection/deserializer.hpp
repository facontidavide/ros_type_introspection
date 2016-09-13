#ifndef ROS_INTRO_DESERIALIZE_H
#define ROS_INTRO_DESERIALIZE_H

#include <ros_type_introspection/parser.hpp>
#include <ros_type_introspection/stringtree.h>

namespace RosIntrospection{

typedef struct{
    StringTree tree;
    std::vector< std::pair<StringElement*, double> > value;
    std::vector< std::pair<StringElement*, ShortString> > name_id;
}ROSTypeFlat;

ROSTypeFlat buildRosFlatType(const ROSTypeList& type_map,
                             ROSType type,
                             const ShortString& prefix,
                             uint8_t **buffer_ptr,
                             uint8_t max_array_size = 32);

} //end namespace

#endif // ROS_INTRO_DESERIALIZE_H
