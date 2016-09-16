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

    StringElement* node_ptr;
    uint16_t array_size;
    uint16_t index_array[7];

    std::string toStr(){
        std::stringstream ss;
        ss << *this;
        return ss.str();
    }
};

typedef struct{
    StringTree tree;
    std::vector< std::pair<StringTreeLeaf, double> > value;
    std::vector< std::pair<StringTreeLeaf, SString> > name_id;
    std::vector< std::pair<SString, double> > renamed_value;

}ROSTypeFlat;

ROSTypeFlat buildRosFlatType(const ROSTypeList& type_map,
                             ROSType type,
                             const SString& prefix,
                             uint8_t **buffer_ptr,
                             uint8_t max_array_size = 32);


std::ostream& operator<<(std::ostream &os, const StringTreeLeaf& leaf );

} //end namespace

#endif // ROS_INTRO_DESERIALIZE_H
