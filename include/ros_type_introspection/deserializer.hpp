#ifndef ROS_INTRO_DESERIALIZE_H
#define ROS_INTRO_DESERIALIZE_H

#include <ros_type_introspection/parser.hpp>

namespace RosIntrospection{

typedef struct{
    std::vector< std::pair<SString, double> > value;
    std::vector< std::pair<SString, SString> > name_id;
}ROSTypeFlat;

void  buildRosFlatType(const ROSTypeList& type_map,
                             ROSType type,
                             const SString & prefix,
                             uint8_t** buffer_ptr,
                             ROSTypeFlat* flat_container,
                             uint8_t max_array_size= 32);

} //end namespace

#endif // ROS_INTRO_DESERIALIZE_H
