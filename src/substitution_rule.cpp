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

#include "ros_type_introspection/substitution_rule.hpp"

namespace RosIntrospection{

SubstitutionRule::SubstitutionRule(const char *pattern, const char *alias, const char *substitution)
{
  std::vector<std::string> split_text;
  boost::split(split_text, pattern, boost::is_any_of("./"));

  _pattern.reserve(split_text.size());
  for (const auto& part: split_text){
    if(part.size()>0)  _pattern.push_back( part );
  }

  boost::split(split_text, alias, boost::is_any_of("./"));

  _alias.reserve(split_text.size());
  for (const auto& part: split_text){
    if(part.size()>0)  _alias.push_back( part );
  }

  boost::split(split_text, substitution, boost::is_any_of("./"));

  _substitution.reserve(split_text.size());
  for (const auto& part: split_text){
    if(part.size()>0)  _substitution.push_back( part );
  }
  size_t h1 = std::hash<std::string>{}(pattern);
  size_t h2 = std::hash<std::string>{}(alias);
  size_t h3 = std::hash<std::string>{}(substitution);
  _hash = (h1 ^ (h2 << 1)) ^ (h3 << 1 );
}

}
