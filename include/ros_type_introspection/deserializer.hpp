#ifndef ROS_INTRO_DESERIALIZE_H
#define ROS_INTRO_DESERIALIZE_H

#include <ros_type_introspection/parser.hpp>
#include <ros_type_introspection/stringtree.h>
#include <sstream>

namespace RosIntrospection{

class StringTreeLeaf{
    friend std::ostream& operator<<(std::ostream &os, const StringTreeLeaf& leaf );
public:
    StringElement* element_ptr;
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
    std::vector< std::pair<StringTreeLeaf, ShortString> > name_id;
}ROSTypeFlat;

ROSTypeFlat buildRosFlatType(const ROSTypeList& type_map,
                             ROSType type,
                             const ShortString& prefix,
                             uint8_t **buffer_ptr,
                             uint8_t max_array_size = 32);



inline std::ostream& operator<<(std::ostream &os, const StringTreeLeaf& leaf )
{
    const StringElement* head = leaf.element_ptr;

    if( !head ) return os;

    const StringElement* array[64];
    int index = 0;
    array[index++] = head;

    while( head->parent())
    {
        head = head->parent();
        array[index++] = head;
    };
    array[index] = nullptr;
    index--;

    int array_count = 0;

    while ( index >=0)
    {
        const auto& value =  array[index]->value();
        if( value.compare("#") == 0)
        {
            os << leaf.index_array[ array_count++ ];
        }
        else{
            if( array[index] ) os << array[index]->value();
        }
        if( index >0 )  os << ".";
        index--;
    }
    return os;
}

} //end namespace

#endif // ROS_INTRO_DESERIALIZE_H
