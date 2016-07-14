#include <ros_type_introspection/ros_type_introspection.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/utility/string_ref.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <functional>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>

namespace ROSTypeParser{


static const  boost::regex type_regex("[a-zA-Z][a-zA-Z0-9_]*"
                                      "(/[a-zA-Z][a-zA-Z0-9_]*){0,1}"
                                      "(\\[[0-9]*\\]){0,1}");

static const  boost::regex field_regex("[a-zA-Z][a-zA-Z0-9_]*");

static const  boost::regex msg_separation_regex("^=+\\n+");


inline bool isSeparator(const std::string& line)
{
    if(line.size() != 80 ) return false;
    for (int i=0; i<80; i++)
    {
        if( line[i] != '=') return false;
    }
    return true;
}


inline ShortString strippedTypeName(const boost::string_ref& line )
{
    boost::string_ref output( line );
    int pos = line.find_last_of('/');
    if( pos != output.npos )
    {
        output.remove_prefix( pos+1 );
    }
    return ShortString( output.data(), output.length() );
}


ROSType::ROSType(const std::string &name):
    _base_name(name)
{

    std::vector<std::string> split;
    std::string type_field;
    boost::split(split, _base_name, boost::is_any_of("/"));

    if( split.size() == 1)
    {
        _pkg_name = "";
        type_field = split[0];
    }
    else{
        _pkg_name = split[0];
        type_field = split[1];
    }
    //----------------------------

    static const boost::regex array_regex("(.+)(\\[([0-9]*)\\])");

    boost::smatch what;
    if (boost::regex_search(type_field, what, array_regex))
    {
      _msg_name = std::string(what[1].first, what[1].second);
      if (what.size() == 3) {
        _array_size = -1;
      }
      else if (what.size() == 4) {
        std::string size(what[3].first, what[3].second);
        _array_size = size.empty() ? -1 : atoi(size.c_str());
      }
      else {
        throw std::runtime_error("Didn't catch bad type string: " + name);
      }
    } else {
      _msg_name = type_field;
      _array_size = 1;
    }
    //------------------------------
    _id = ROSTypeParser::OTHER;

    if( _msg_name.compare( "bool" ) == 0 ) {
        _id = ROSTypeParser::BOOL;
    }
    else if(_msg_name.compare( "byte" ) == 0 ) {
        _id = ROSTypeParser::BYTE;
    }
    else if(_msg_name.compare( "char" ) == 0 ) {
        _id = ROSTypeParser::CHAR;
    }
    else if(_msg_name.compare( "uint8" ) == 0 ) {
        _id = ROSTypeParser::UINT8;
    }
    else if(_msg_name.compare( "uint16" ) == 0 ) {
        _id = ROSTypeParser::UINT16;
    }
    else if(_msg_name.compare( "uint32" ) == 0 ) {
        _id = ROSTypeParser::UINT32;
    }
    else if(_msg_name.compare( "uint64" ) == 0 ) {
        _id = ROSTypeParser::UINT64;
    }
    else if(_msg_name.compare( "int8" ) == 0 ) {
        _id = ROSTypeParser::INT8;
    }
    else if(_msg_name.compare( "int16" ) == 0 ) {
        _id = ROSTypeParser::INT16;
    }
    else if(_msg_name.compare( "int32" ) == 0 ) {
        _id = ROSTypeParser::INT32;
    }
    else if(_msg_name.compare( "int64" ) == 0 ) {
        _id = ROSTypeParser::INT64;
    }
    else if(_msg_name.compare( "float32" ) == 0 ) {
        _id = ROSTypeParser::FLOAT32;
    }
    else if(_msg_name.compare( "float64" ) == 0 ) {
        _id = ROSTypeParser::FLOAT64;
    }
    else if(_msg_name.compare( "time" ) == 0 ) {
        _id = ROSTypeParser::TIME;
    }
    else if(_msg_name.compare( "duration" ) == 0 ) {
        _id = ROSTypeParser::DURATION;
    }
    else if(_msg_name.compare( "string" ) == 0 ) {
        _id = ROSTypeParser::STRING;
    }
}


ROSTypeMap buildROSTypeMapFromDefinition(
        const std::string & type_name,
        const std::string & msg_definition)
{
    ROSTypeMap map;

    std::vector<std::string> split;
    boost::split_regex(split, msg_definition, msg_separation_regex);

    std::vector<ROSType> all_types;

    for (size_t i = 0; i < split.size(); ++i) {

        ROSMessage msg( split[i] );
        if( i == 0)
        {
            msg.type = ROSType(type_name);
        }

        map[ msg.type.msgName() ] = msg;
        all_types.push_back( msg.type );
    }

    for( auto& it: map )
    {
        ROSMessage& msg = it.second;
        msg.updateTypes( all_types );
    }

    return map;
}


std::ostream& operator<<(std::ostream& ss, const ROSTypeMap& type_map)
{
    for (auto it = type_map.begin(); it != type_map.end(); it++)
    {
        ss<< "\n" << it->first <<" : " << std::endl;

        const ROSMessage& message = it->second;
        for (int i=0; i< message.fields.size(); i++ )
        {
            ss << "\t" << message.fields.at(i).name()
               <<" : " << message.fields.at(i).type().msgName() << std::endl;
        }
    }
    return ss;
}


const std::string &ROSType::baseName() const
{
    return _base_name;
}

const ShortString &ROSType::msgName() const
{
    return _msg_name;
}

const ShortString &ROSType::pkgName() const
{
    return _pkg_name;
}

bool ROSType::isArray() const
{
    return _array_size != 1;
}

bool ROSType::isBuiltin() const
{
    return _id != ROSTypeParser::OTHER;
}

int ROSType::arraySize() const
{
    return _array_size;
}

int ROSType::typeSize() const
{
    const int sizes[] = {1, 1, 1,
                         1, 2, 4, 8,
                         1, 2, 4, 8,
                         4, 8,
                         8, 8,
                         -1, -1};
    return sizes[ _id ];
}

ROSMessage::ROSMessage(const std::string &msg_def)
{
    std::istringstream messageDescriptor(msg_def);
    boost::match_results<std::string::const_iterator> what;

    for (std::string line; std::getline(messageDescriptor, line, '\n') ; )
    {
        std::string::const_iterator begin = line.begin(), end = line.end();

        // Skip empty line or one that is a comment
        if (boost::regex_search( begin, end, what,
                                 boost::regex("(^\\s*$|^\\s*#)")))
        {
            continue;
        }

        if( line.compare(0, 5, "MSG: ") == 0)
        {
            line.erase(0,5);
            this->type = ROSType(line);
            if( ! std::getline(messageDescriptor, line, '\n') ) { break; }
        }
        else{
            fields.push_back( ROSField(line) );
        }
    }
}

void ROSMessage::updateTypes(std::vector<ROSType> all_types)
{
    for (ROSField& field: fields)
    {
        if( field.type().pkgName().empty() )
        {
            for (ROSType& known_type: all_types)
            {
                if( field.type().msgName() == known_type.msgName() )
                {
                    field._type = known_type;
                    break;
                }
            }
        }
    }
}

ROSField::ROSField(const std::string &definition)
{
    using boost::regex;
    std::string::const_iterator begin = definition.begin();
    std::string::const_iterator end   = definition.end();
    boost::match_results<std::string::const_iterator> what;

    // Get type and field
    std::string type, fieldname, value;

    if( regex_search(begin, end, what, type_regex)) {
        type = what[0];
        begin = what[0].second;
    }
    else {
        throw std::runtime_error("Bad type when parsing message\n" + definition);
    }

    if (regex_search(begin, end, what, field_regex))
    {
        fieldname = what[0];
        begin = what[0].second;
    }
    else {
        throw std::runtime_error("Bad field when parsing message\n" + definition);
    }

    // Determine next character
    // if '=' -> constant, if '#' -> done, if nothing -> done, otherwise error
    if (regex_search(begin, end, what, boost::regex("\\S")))
    {
        if (what[0] == "=")
        {
            begin = what[0].second;
            // Copy constant
            if (type == "string") {
                value.assign(begin, end);
            }
            else {
                if (regex_search(begin, end, what, boost::regex("\\s*#")))
                {
                    value.assign(begin, what[0].first);
                }
                else {
                    value.assign(begin, end);
                }
                // TODO: Raise error if string is not numeric
            }

            boost::algorithm::trim(value);
        } else if (what[0] == "#") {
            // Ignore comment
        } else {
            // Error
            throw std::runtime_error("Unexpected character after type and field '" +
                                     definition);
        }
    }
    _type  = ROSType( type );
    _name  = fieldname;
    _value = value;
}





} // end namespace


