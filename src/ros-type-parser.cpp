#include "ros-type-parser.h"

#include <iostream>
#include <sstream>

namespace RosTypeParser{

bool isCommentOrEmpty(const std::string& line)
{
    if(line.empty()) return true;

    int index = 0;
    while( index < line.size()-1 && line[index] == ' '  )
    {
        index++;
    }
    return (line[index] == '#');
}

bool isSeparator(const std::string& line)
{
    if(line.size() != 80 ) return false;
    for (int i=0; i<80; i++)
    {
        if( line[i] != '=') return false;
    }
    return true;
}


std::string stripTypeName( std::string line)
{
    for(int index = line.size() -1; index >=0; index--)
    {
        if( line[index] == '/')
        {
            line.erase(0,index+1);
            break;
        }
    }

    return line;
}



// recursive funtion
bool buildFlatTypeHierarchy_Impl(std::string prefix,
             const std::vector<Type>& types,
             const std::string& type_name,
             std::vector<Field>* output)
{
    int index = 0;
    for ( index = 0; index < types.size(); index++)
    {
        if( type_name.compare( types[index].type_name ) == 0)
        {
            break;
        }
    }

    if( index >= types.size()) return false;

    const Type& type = types[index];

    for (int f=0; f< type.fields.size(); f++)
    {
        const Field& field = type.fields[f];
        std::string new_prefix ( prefix );
        new_prefix.append(".");
        new_prefix.append( field.field_name );

        if( !buildFlatTypeHierarchy_Impl( new_prefix, types, field.type_name, output) )
        {
            Field new_field;
            new_field.field_name = new_prefix;
            new_field.type_name  = field.type_name;
            output->push_back( new_field );
        }
    }
    return true;
}

std::vector<Field> buildFlatTypeHierarchy(std::string prefix,
             const std::vector<Type>& types,
             const std::string& type_name)
{
    std::vector<Field> output;
    buildFlatTypeHierarchy_Impl( prefix, types, type_name, &output);
    return output;
}

std::vector<Type> parseRosTypeDescription(const std::string & type_name, const std::string & msg_definition)
{
    std::istringstream messageDescriptor(msg_definition);

    std::vector<Type> types;
    types.push_back( Type() );

    Type* current_type = &types.back();

    std::string main_typename = (type_name);

    current_type->type_name = main_typename;

    for (std::string line; std::getline(messageDescriptor, line, '\n') ; )
    {
        if( isCommentOrEmpty(line) )
        {
            continue;
        }

        if( isSeparator(line) )
        {
            if( std::getline(messageDescriptor, line, '\n') == 0)
            {
                break;
            }

            types.push_back( Type() );
            current_type = &types.back();

            if( line.compare(0, 5, "MSG: ") == 0)
            {
                line.erase(0,5);
            }

            current_type->type_name =  stripTypeName(line);
        }
        else{
            Field field;

            std::stringstream ss2(line);
            ss2 >> field.type_name;
            ss2 >> field.field_name;

            current_type->fields.push_back( field );
        }
    }

    return types;
}

}
