#ifndef ROS_INTRO_DESERIALIZE_H
#define ROS_INTRO_DESERIALIZE_H

#include <ros_type_introspection/parser.hpp>

namespace ROSIntrospection{

typedef struct{
    std::vector< std::pair<LongString, double> > value;
    std::vector< std::pair<LongString, LongString> > name_id;
}ROSTypeFlat;

ROSTypeFlat buildRosFlatType(const ROSTypeList& type_map,
                             ROSType type,
                             const LongString& prefix,
                             uint8_t **buffer_ptr);



} //end namespace

#endif // ROS_INTRO_DESERIALIZE_H
