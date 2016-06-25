#include "ros-type-parser.h"
#include <boost/algorithm/string.hpp>
#include <boost/utility/string_ref.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <functional>

namespace RosTypeParser{

const std::string VECTOR_SYMBOL("[]");
const String SEPARATOR(".");


inline bool isCommentOrEmpty(const std::string& line)
{
    if(line.empty()) return true;

    int index = 0;
    while( index < line.size()-1 && line[index] == ' '  )
    {
        index++;
    }
    return (line[index] == '#');
}

inline bool isSeparator(const std::string& line)
{
    if(line.size() != 80 ) return false;
    for (int i=0; i<80; i++)
    {
        if( line[i] != '=') return false;
    }
    return true;
}


inline String strippedTypeName(const boost::string_ref& line )
{
    boost::string_ref output( line );
    int pos = line.find_last_of('/');
    if( pos != output.npos )
    {
        output.remove_prefix( pos+1 );
    }
    return String( output.data(), output.length() );
}


void parseRosTypeDescription(
        const std::string & type_name,
        const std::string & msg_definition,
        RosTypeMap* type_map)
{
    std::istringstream messageDescriptor(msg_definition);

    auto current_type_name = strippedTypeName(type_name);

    (*type_map)[ current_type_name ] = RosType(type_name );


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

            (*type_map)[ current_type_name ] = RosType(line );
        }
        else{
            RosTypeField field;

            std::string temp;
            std::stringstream ss2(line);
            ss2 >> temp;
            field.type_name = strippedTypeName( temp.data());
            ss2 >> temp;
            field.field_name = String( temp.data(), temp.length());

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

/*
void printRosType(const RosTypeMap& type_map, const std::string& type_name, int indent  )
{
    for (int d=0; d<indent; d++) std::cout << "  ";
    std::cout  << type_name <<" : " << std::endl;

    boost::string_ref type ( type_name );
    if( type.ends_with( VECTOR_SYMBOL))
    {
        type.remove_suffix(2);
    }

    auto it = type_map.find( String(type.data(), type.length()) );
    if( it != type_map.end())
    {
        auto fields = it->second.fields;
        for (int i=0; i< fields.size(); i++)
        {
            auto field_type ( fields[i].type_name );
            if( field_type.ends_with( VECTOR_SYMBOL))
            {
                field_type.remove_suffix(2);
            }

            if( type_map.find( field_type ) == type_map.end()) // go deeper with recursion
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
}*/

template <typename T> T ReadFromBufferAndMoveForward( uint8_t** buffer)
{
    for (int i=0; i< sizeof(T); i++)
    {
        //     printf(" %02X ", (*buffer)[i] );
    }
    T destination =  (*( reinterpret_cast<T*>( *buffer ) ) );
    *buffer +=  sizeof(T);
    return destination;
}


void buildRosFlatType(const RosTypeMap& type_map,
                      const String &type_name,
                      String prefix,
                      uint8_t** buffer_ptr,
                      RosTypeFlat* flat_container )
{

    boost::string_ref type ( type_name.data(), type_name.size() );
    int32_t vect_size = 1;

    bool is_vector = false;
    if( type.ends_with( VECTOR_SYMBOL ) )
    {
        vect_size = ReadFromBufferAndMoveForward<int32_t>( buffer_ptr );
        type.remove_suffix(2);
        is_vector = true;
    }

    std::function<void(const String&)> deserializeAndStore;

    if( type.compare( "float64") == 0 )
    {
        deserializeAndStore = [&](const String& key){
            double value = ReadFromBufferAndMoveForward<double>(buffer_ptr);
            flat_container->value[key ] = value;
        };
    }
    else if( type.compare( "uint32") == 0 )
    {
        deserializeAndStore = [&](const String& key){
            double value = (double)ReadFromBufferAndMoveForward<uint32_t>(buffer_ptr);
            flat_container->value[key ] = value;
        };
    }
    else if( type.compare("time") == 0 )
    {
        deserializeAndStore = [&](const String& key){
            ros::Time time = ReadFromBufferAndMoveForward<ros::Time>(buffer_ptr);
            double value = time.toSec();
            flat_container->value[ key ] = value;
        };
    }
    else if( type.compare("string") == 0 )
    {
        deserializeAndStore = [&](const String& key){
            int32_t string_size = ReadFromBufferAndMoveForward<int32_t>( buffer_ptr );
            String id( (const char*)(*buffer_ptr), string_size );
            (*buffer_ptr) += string_size;
            flat_container->name_id[ key ] = id;
        };
    }
    else {
        deserializeAndStore = [&](const String& key)
        {
            auto it = type_map.find( String( type.data(), type.size() ) );
            if( it != type_map.end())
            {
                auto& fields  = it->second.fields;

                for (int i=0; i< fields.size(); i++ ) {
                    String new_prefix( key );
                    new_prefix.append( SEPARATOR );
                    new_prefix.append( fields[i].field_name.data(), fields[i].field_name.size())  ;

                    buildRosFlatType(type_map, (fields[i].type_name),
                                     new_prefix,
                                     buffer_ptr, flat_container );
                }
            }
            else {
                std::cout << " type not recognized: " <<  type << std::endl;
            }
        };
    }


    for (int v=0; v<vect_size; v++)
    {
        String key (prefix);
        if( is_vector )
        {
            char suffix[16];
            sprintf(suffix,"[%d]", v);
            key.append( suffix );
        }
        deserializeAndStore(key);
    }
}


void applyNameTransform( const std::vector< SubstitutionRule >&  rules,
                         RosTypeFlat* container)
{
    for (auto it = container->value.begin(); it != container->value.end(); it++)
    {
        boost::string_ref name ( it->first.data(),  it->first.size());
        double value = it->second;

        bool substitution_done = false;

        for (int r=0; r < rules.size(); r++)
        {
            const auto& rule = rules[r];

            int posA = name.find(rule.pattern_pre );
            if( posA == name.npos) { continue; }

            int posB = posA + rule.pattern_pre.length();
            int posC = posB;

            while( isdigit(  name.at(posC) ) && posC < name.npos)
            {
                posC++;
            }
            if( posC == name.npos) continue;

            boost::string_ref name_prefix = name.substr( 0, posA );
            boost::string_ref index       = name.substr(posB, posC-posB );
            boost::string_ref name_suffix = name.substr( posC, name.length() - posC );

            int res = std::strncmp( name_suffix.data(), rule.pattern_suf.data(),  rule.pattern_suf.length() );
            if( res != 0)
            {
                continue;
            }
            name_suffix.remove_prefix( rule.pattern_suf.length() );

            char key[256];
            int buffer_index = 0;
            for (const char c: name_prefix       )  key[buffer_index++] = c;
            for (const char c: rule.location_pre )  key[buffer_index++] = c;
            for (const char c: index             )  key[buffer_index++] = c;
            for (const char c: rule.location_suf )  key[buffer_index++] = c;
            key[buffer_index] = '\0';

            auto substitutor = container->name_id.find( key ) ;
            if( substitutor != container->name_id.end())
            {
                auto& index_replacement = substitutor->second;

                char new_name[256];
                int name_index = 0;
                for (const char c: name_prefix           )  new_name[name_index++] = c;
                for (const char c: rule.substitution_pre )  new_name[name_index++] = c;

                for (int i=0; i< index_replacement.size(); i++ )
                    new_name[name_index++] = index_replacement.at(i);

                for (const char c: rule.substitution_suf )  new_name[name_index++] = c;
                for (const char c: name_suffix           )  new_name[name_index++] = c;
                new_name[name_index] = '\0';

                container->value_renamed[new_name] = value;

                /*std::cout << "---------------" << std::endl;
                std::cout << "index        " << index << std::endl;
                std::cout << "name_prefix  " << name_prefix << std::endl;
                std::cout << "key  " << key << std::endl;
                std::cout << "new_name   " << new_name << std::endl;
                std::cout << "---------------" << std::endl;
                */
                // DON'T apply more than one rule
                substitution_done = true;
                break;
            }
        }

        if( !substitution_done)
        {
            //just move it without changes
            container->value_renamed[ it->first ] = value;
        }
    }

}

std::ostream& operator<<(std::ostream& ss, const RosTypeFlat& container)
{

    for (auto it= container.name_id.begin(); it != container.name_id.end(); it++)
    {
        ss<< it->first << " = " << it->second << std::endl;
    }

    for (auto it= container.value.begin(); it != container.value.end(); it++)
    {
        ss << it->first << " = " << it->second << std::endl;
    }
    return ss;
}

SubstitutionRule::SubstitutionRule(const char *pattern, const char *name_location, const char *substitution):
    pattern_pre(pattern),            pattern_suf(pattern),
    location_pre(name_location),     location_suf(name_location),
    substitution_pre(substitution),  substitution_suf(substitution)
{
    int pos;
    pos = pattern_pre.find_first_of('#');
    pattern_pre.remove_suffix( pattern_pre.length() - pos );
    pattern_suf.remove_prefix( pos+1 );

    pos = location_pre.find_first_of('#');
    location_pre.remove_suffix( location_pre.length() - pos );
    location_suf.remove_prefix( pos+1 );

    pos = substitution_pre.find_first_of('#');
    substitution_pre.remove_suffix( substitution_pre.length() - pos );
    substitution_suf.remove_prefix( pos+1 );
}

}
