/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright 2016-2017 Davide Faconti
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
* *******************************************************************/


#include "ros_type_introspection/ros_type.hpp"

namespace RosIntrospection{

ROSType::ROSType(const SString &name):
  _base_name(name)
{
  int pos = -1;
  for (size_t i=0; i<name.size(); i++)
  {
    if(name.at(i) == '/'){
      pos = i;
      break;
    }
  }

  if( pos != -1)
  {
    _msg_name = name;
  }
  else{
    _pkg_name.assign( name.data(), pos);
    pos++;
    _msg_name.assign( name.data() + pos, name.size() - pos);
  }

  _id = toBuiltinType( _msg_name );
}

ROSType::ROSType(const std::string &name):
  _base_name(name)
{
  size_t separator_pos = name.find_first_of('/');

  if( separator_pos == std::string::npos)
  {
    _msg_name = name;
  }
  else{
    _pkg_name.assign( &name[0], separator_pos);
    separator_pos++;
    _msg_name.assign( &name[separator_pos], name.size() - separator_pos);
  }

  //------------------------------
  _id = toBuiltinType( _msg_name );
}

void ROSType::setPkgName(const SString &new_pkg)
{
  assert(_pkg_name.size() == 0);
  _pkg_name = new_pkg;
  _base_name = SString(new_pkg).append("/").append(_base_name);
}


}
