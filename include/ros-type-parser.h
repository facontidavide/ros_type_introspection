#ifndef ROSTYPEPARSER_H
#define ROSTYPEPARSER_H

#include <vector>
#include <string>
#include <map>
#include <boost/utility/string_ref.hpp>

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

    RosType() {}

    RosType( std::string name):
        full_name(name) { }
};

typedef std::map<std::string, RosType> RosTypeMap;

typedef struct{
    std::map<std::string, double> value_renamed;
    std::map<std::string, double> value;
    std::map<std::string, std::string> name_id;
}RosTypeFlat;

//------------------------------
std::ostream& operator<<(std::ostream& s, const RosTypeFlat& c);

void parseRosTypeDescription(
        const std::string & type_name,
        const std::string & msg_definition,
        RosTypeMap* type_map);

void printRosTypeMap( const RosTypeMap& type_map );
void printRosType(const RosTypeMap& type_map, const std::string& type_name, int indent = 0 );

void buildRosFlatType(const RosTypeMap& type_map,
                       const std::string& type_name,
                       std::string prefix,
                       uint8_t **buffer_ptr,
                       RosTypeFlat* flat_container);


class SubstitutionRule{
public:
    SubstitutionRule(const char* pattern, const char* name_location, const char*substitution);

    boost::string_ref pattern_suf;
    boost::string_ref pattern_pre;

    boost::string_ref location_suf;
    boost::string_ref location_pre;

    boost::string_ref substitution_suf;
    boost::string_ref substitution_pre;
};

void applyNameTransform(const std::vector<SubstitutionRule> &rules,
                         RosTypeFlat* container);

}

#endif // ROSTYPEPARSER_H
