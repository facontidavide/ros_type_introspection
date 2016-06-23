#ifndef ROSTYPEPARSER_H
#define ROSTYPEPARSER_H

#include <vector>
#include <string>
#include <map>

namespace RosTypeParser{

typedef struct
{
    std::string type_name;
    std::string field_name;

} RosTypeField;

class RosType
{
public:
    std::string full_name;
    std::vector<RosTypeField> fields;

    RosType( std::string name):
        full_name(name) { }

};

typedef std::map<std::string, RosType> RosTypeMap;

typedef std::map<std::string, double> RosTypeFlat;

//------------------------------


void parseRosTypeDescription(
        const std::string & type_name,
        const std::string & msg_definition,
        RosTypeMap* type_map);

void printRosTypeMap( const RosTypeMap& type_map );
void printRosType(const RosTypeMap& type_map, const std::string& type_name, int indent = 0 );

void buildOffsetTable(const RosTypeMap& type_map,
                       const std::string& type_name,
                       std::string prefix,
                       uint8_t** buffer_ptr,
                       RosTypeFlat* flat_container);

}

#endif // ROSTYPEPARSER_H
