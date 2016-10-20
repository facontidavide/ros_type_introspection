/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright 2016 Davide Faconti
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#include <ros_type_introspection/parser.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/utility/string_ref.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <functional>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>

namespace RosIntrospection{


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


inline SString strippedTypeName(const boost::string_ref& line )
{
    boost::string_ref output( line );
    int pos = line.find_last_of('/');
    if( pos != output.npos )
    {
        output.remove_prefix( pos+1 );
    }
    return SString( output.data(), output.length() );
}


ROSType::ROSType(const std::string &name):
    _base_name(name)
{

    std::vector<std::string> split;
    std::string type_field;
    boost::split(split, _base_name, boost::is_any_of("/"));

    if( split.size() == 1)
    {
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
    _id = RosIntrospection::OTHER;

    if( _msg_name.compare( "bool" ) == 0 ) {
        _id = RosIntrospection::BOOL;
    }
    else if(_msg_name.compare( "byte" ) == 0 ) {
        _id = RosIntrospection::BYTE;
    }
    else if(_msg_name.compare( "char" ) == 0 ) {
        _id = RosIntrospection::CHAR;
    }
    else if(_msg_name.compare( "uint8" ) == 0 ) {
        _id = RosIntrospection::UINT8;
    }
    else if(_msg_name.compare( "uint16" ) == 0 ) {
        _id = RosIntrospection::UINT16;
    }
    else if(_msg_name.compare( "uint32" ) == 0 ) {
        _id = RosIntrospection::UINT32;
    }
    else if(_msg_name.compare( "uint64" ) == 0 ) {
        _id = RosIntrospection::UINT64;
    }
    else if(_msg_name.compare( "int8" ) == 0 ) {
        _id = RosIntrospection::INT8;
    }
    else if(_msg_name.compare( "int16" ) == 0 ) {
        _id = RosIntrospection::INT16;
    }
    else if(_msg_name.compare( "int32" ) == 0 ) {
        _id = RosIntrospection::INT32;
    }
    else if(_msg_name.compare( "int64" ) == 0 ) {
        _id = RosIntrospection::INT64;
    }
    else if(_msg_name.compare( "float32" ) == 0 ) {
        _id = RosIntrospection::FLOAT32;
    }
    else if(_msg_name.compare( "float64" ) == 0 ) {
        _id = RosIntrospection::FLOAT64;
    }
    else if(_msg_name.compare( "time" ) == 0 ) {
        _id = RosIntrospection::TIME;
    }
    else if(_msg_name.compare( "duration" ) == 0 ) {
        _id = RosIntrospection::DURATION;
    }
    else if(_msg_name.compare( "string" ) == 0 ) {
        _id = RosIntrospection::STRING;
    }
}


ROSTypeList buildROSTypeMapFromDefinition(
        const std::string & type_name,
        const std::string & msg_definition)
{
    ROSTypeList type_list;

    std::vector<std::string> split;
    boost::split_regex(split, msg_definition, msg_separation_regex);

    std::vector<ROSType> all_types;

    for (size_t i = 0; i < split.size(); ++i) {

        ROSMessage msg( split[i] );
        if( i == 0)
        {
            msg.mutateType( ROSType(type_name) );
        }

        type_list.push_back( msg );
        all_types.push_back( msg.type() );
    }

    for( ROSMessage& msg: type_list )
    {
        msg.updateTypes( all_types );
    }

    return type_list;
}


std::ostream& operator<<(std::ostream& ss, const ROSTypeList& type_list)
{
    for (const ROSMessage& msg: type_list)
    {
        ss<< "\n" << msg.type().baseName() <<" : " << std::endl;

        for (const ROSField& field : msg.fields() )
        {
            ss << "\t" << field.name()
               <<" : " << field.type().baseName() << std::endl;
        }
    }
    return ss;
}


const std::string &ROSType::baseName() const
{
    return _base_name;
}

const SString &ROSType::msgName() const
{
    return _msg_name;
}

const SString &ROSType::pkgName() const
{
    return _pkg_name;
}

void ROSType::setPkgName(const SString &new_pkg)
{
    assert(_pkg_name.size() == 0);
    _pkg_name = new_pkg;
    std::string pkg( new_pkg.data(), new_pkg.size() );
    _base_name = pkg + std::string("/") + _base_name;
}

bool ROSType::isArray() const
{
    return _array_size != 1;
}

bool ROSType::isBuiltin() const
{
    return _id != RosIntrospection::OTHER;
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

BuiltinType ROSType::typeID() const
{
    return this->_id;
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
            _type = ROSType(line);
        }
        else{
            auto new_field = ROSField(line);
            _fields.push_back(new_field);
        }
    }
}

void ROSMessage::updateTypes(const std::vector<ROSType> &all_types)
{
    for (ROSField& field: _fields)
    {
        // if package name is missing, try to find msgName in the list of known_type
        if( field.type().pkgName().size() == 0 )
        {
            for (const ROSType& known_type: all_types)
            {
                if( field.type().msgName() == known_type.msgName() )
                {
                    field._type.setPkgName( known_type.pkgName() );
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


