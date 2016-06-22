#ifndef ROSTYPEPARSER_H
#define ROSTYPEPARSER_H

#include <vector>
#include <string>

namespace RosTypeParser{

typedef struct
{
    std::string type_name;
    std::string field_name;
} Field;

typedef struct
{
    std::string type_name;
    std::vector<Field> fields;
}Type;



std::vector<Field> buildFlatTypeHierarchy(std::string prefix,
             const std::vector<Type>& types,
             const std::string& type_name);

std::vector<Type>  parseRosTypeDescription(
        const std::string & type_name,
        const std::string & msg_definition );


}

#endif // ROSTYPEPARSER_H
