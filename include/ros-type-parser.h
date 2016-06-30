#ifndef ROSTYPEPARSER_H
#define ROSTYPEPARSER_H

#include <vector>
#include <string.hpp>
#include <map>
#include <boost/utility/string_ref.hpp>


namespace RosTypeParser{

// you might (will?) find annoying that we use a different String implementation.
// but most of the time this class, which uses "small string optimization" for
// string which have less than 62 character,
// will perform faster when the string has less than 62 characters.
#if 1
typedef sso63::string String;
#else
typedef std::string String;
#endif

typedef struct
{
    String type_name;
    String field_name;

} RosTypeField;

typedef struct RosType_
{
    String full_name;
    std::vector<RosTypeField> fields;

    RosType_(){}
    RosType_( const String& name) { full_name = name; }
}RosType;

typedef std::map<String, RosType> RosTypeMap;



typedef struct{
   std::vector< std::pair<String, double> > value_renamed;
   std::vector< std::pair<String, double> > value;
   std::vector< std::pair<String, String> > name_id;

}RosTypeFlat;

//------------------------------
std::ostream& operator<<(std::ostream& s, const RosTypeFlat& c);

void buildRosTypeMapFromDefinition(
        const std::string & type_name,
        const std::string & msg_definition,
        RosTypeMap* type_map);

RosTypeMap buildRosTypeMapFromDefinition(
        const std::string & type_name,
        const std::string & msg_definition);

std::ostream& operator<<(std::ostream& s, const RosTypeMap& c);

void printRosType(const RosTypeMap& type_map, const String &type_name, int indent = 0 );



RosTypeFlat buildRosFlatType(const RosTypeMap& type_map,
                      const String& type_name,
                      const String& prefix,
                      uint8_t **buffer_ptr);


class SubstitutionRule{
public:
    SubstitutionRule( std::string pattern, std::string name_location, std::string substitution);

    std::string pattern_suf;
    std::string pattern_pre;

    std::string location_suf;
    std::string location_pre;

    std::string substitution_suf;
    std::string substitution_pre;
private:

};

typedef std::map<std::string, SubstitutionRule> SubstitutionRuleSet;

void applyNameTransform(const std::vector<SubstitutionRule> &rules,
                        RosTypeFlat* container);

}

#endif // ROSTYPEPARSER_H
