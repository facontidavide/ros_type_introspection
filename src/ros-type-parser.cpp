#include "ros-type-parser.h"
#include <boost/algorithm/string.hpp>
#include <boost/utility/string_ref.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <iostream>
#include <sstream>

namespace RosTypeParser{

const std::string VECTOR_SYMBOL("[]");


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


std::string strippedTypeName( std::string line)
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


void parseRosTypeDescription(
        const std::string & type_name,
        const std::string & msg_definition,
        RosTypeMap* type_map)
{
    std::istringstream messageDescriptor(msg_definition);

    std::string current_type_name = strippedTypeName(type_name);

    if( type_map->find( current_type_name ) == type_map->end())
        type_map->insert( std::make_pair( current_type_name, RosType(type_name) ) );

    for (std::string line; std::getline(messageDescriptor, line, '\n') ; )
    {
        if( isCommentOrEmpty(line) )
        {
            continue;
        }

        if( isSeparator(line) ) // start to store a sub type
        {
            if( std::getline(messageDescriptor, line, '\n') == 0) {
                break;
            }

            if( line.compare(0, 5, "MSG: ") == 0)
            {
                line.erase(0,5);
            }

            current_type_name = strippedTypeName(line);

            if( type_map->find( current_type_name) == type_map->end())
                type_map->insert( std::make_pair( current_type_name, RosType( line ) ) );
        }
        else{
            RosTypeField field;

            std::stringstream ss2(line);
            ss2 >> field.type_name;
            ss2 >> field.field_name;

            field.type_name = strippedTypeName( field.type_name );

            auto& fields = type_map->find(current_type_name)->second.fields;

            bool found = false;
            for (int i=0; i<fields.size(); i++)
            {
                if( fields[i].field_name.compare( field.field_name) == 0 )
                {
                    found = true;
                    break;
                }
            }
            if( !found )
            {
                fields.push_back( field );
            }
        }
    }
}


void printRosTypeMap(const RosTypeMap& type_map)
{
    for (auto it = type_map.begin(); it != type_map.end(); it++)
    {
        std::cout<< "\n" << it->first <<" : " << std::endl;

        auto& fields = it->second.fields;
        for (int i=0; i< fields.size(); i++ )
        {
            std::cout<< "\t" << fields[i].field_name <<" : " << fields[i].type_name << std::endl;
        }
    }
}


void printRosType(const RosTypeMap& type_map, const std::string& type_name, int indent  )
{
    for (int d=0; d<indent; d++) std::cout << "  ";
    std::cout  << type_name <<" : " << std::endl;

    boost::string_ref type ( type_name );
    if( type.ends_with( VECTOR_SYMBOL))
    {
        type.remove_suffix(2);
    }

    auto it = type_map.find( type.to_string() );
    if( it != type_map.end())
    {
        auto fields = it->second.fields;
        for (int i=0; i< fields.size(); i++)
        {
            boost::string_ref field_type ( fields[i].type_name );
            if( field_type.ends_with( VECTOR_SYMBOL))
            {
                field_type.remove_suffix(2);
            }

            if( type_map.find( field_type.to_string() ) == type_map.end()) // go deeper with recursion
            {
                for (int d=0; d<indent; d++) std::cout << "   ";
                std::cout << "   " << fields[i].field_name <<" : " << fields[i].type_name << std::endl;
            }
            else{
                printRosType( type_map, fields[i].type_name , indent+1 );
            }
        }
    }
    else{
         std::cout << type << " not found " << std::endl;
    }
}

template <typename T> T ReadFromBufferAndMoveForward( uint8_t** buffer)
{
    for (int i=0; i< sizeof(T); i++)
    {
        printf(" %02X ", (*buffer)[i] );
    }
    T destination =  (*( reinterpret_cast<T*>( *buffer ) ) );
    *buffer +=  sizeof(T);
    return destination;
}


void buildOffsetTable( const RosTypeMap& type_map,
                       const std::string& type_name,
                       std::string prefix,
                       uint8_t** buffer_ptr,
                       RosTypeFlat* flat_container )
{
    std::cout << ">>>>>>>>   "<< type_name  << std::endl;
    const std::string separator(".");

    boost::string_ref type ( type_name );
    int32_t vect_size = 1;

    if( type.ends_with( VECTOR_SYMBOL ) )
    {
        vect_size = ReadFromBufferAndMoveForward<int32_t>( buffer_ptr );
        type.remove_suffix(2);
        std::cout << "vect_size : "<< vect_size  << std::endl;
    }

    for (int v=0; v<vect_size; v++)
    {
        double value = -1;
        std::string suffix;

        if( vect_size > 1)
        {
            suffix.append("[").append( boost::lexical_cast<std::string>( v ) ).append("]");
        }

        if( type.compare( "float64") == 0 )
        {
            value = ReadFromBufferAndMoveForward<double>(buffer_ptr);
            (*flat_container)[ prefix + suffix ] = value;
        }
        else if( type.compare( "uint32") == 0)
        {
            value = (double) ReadFromBufferAndMoveForward<uint32_t>(buffer_ptr);
            (*flat_container)[ prefix + suffix ] = value;
        }
        else if( type.compare("time") == 0 )
        {
            (*buffer_ptr) += sizeof( ros::Time );
        }
        else if( type.compare("string") == 0 )
        {
            //  std::string value;
            int32_t string_size = ReadFromBufferAndMoveForward<int32_t>( buffer_ptr );

            //  value.reserve( string_size );
            //  value.append( (const char*)(*buffer_ptr), string_size );

            std::cout << "string_size : "<< string_size  << std::endl;
            (*buffer_ptr) += string_size;
        }
        else {       // try recursion
            auto it = type_map.find( type.to_string() );
            if( it != type_map.end())
            {
                auto& fields  = it->second.fields;

                for (int i=0; i< fields.size(); i++ )
                {
                    std::cout << "subfield: "<<  fields[i].field_name << " / " <<  fields[i].type_name << std::endl;

                    buildOffsetTable(type_map, fields[i].type_name,
                                     (prefix + suffix + separator + fields[i].field_name  ),
                                     buffer_ptr, flat_container );
                }
            }
            else {
                std::cout << " type not recognized: " <<  type << std::endl;
            }
        }

    }


    std::cout << "<<<<<< " << std::endl;
}





}
