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

#ifndef ROS_INTROSPECTION_RENAMER_H
#define ROS_INTROSPECTION_RENAMER_H

#include <ros_type_introspection/deserializer.hpp>

namespace RosIntrospection{


class SubstitutionRule {
public:

  /**
   * @brief Pass the three arguments (pattern, alias, substitution) as point separated strings.
   *
   * Example; consider the JointState message. Before renaming your key/value pair will look like
   *
   *   JointState.header.seq >> 1234
   *   JointState.header.stamp >> 2000.00
   *   JointState.header.frame_id >> base_frame
   *
   *   JointState.position.0 >> 11
   *   JointState.velocity.0 >> 21
   *   JointState.effort.0 >> 31
   *   JointState.name.0 >> first_joint
   *
   *   JointState.position.1 >> 12
   *   JointState.velocity.1 >> 22
   *   JointState.effort.1 >> 32
   *   JointState.name.1 >> second_joint
   *
   * you can "remap" this to
   *
   *    JointState.header.seq >> 1234
   *    JointState.header.stamp >> 2000.00
   *    JointState.header.frame_id >> base_frame
   *
   *    JointState.first_joint.pos >> 11
   *    JointState.first_joint.vel >> 21
   *    JointState.first_joint.eff >> 31
   *
   *    JointState.second_joint.pos >> 12
   *    JointState.second_joint.vel >> 22
   *    JointState.second_joint.eff >> 32
   *
   * using these three rules:
   *
   *
   *    std::vector<SubstitutionRule> rules;
   *    rules.push_back( SubstitutionRule( "position.#", "name.#", "@.pos" ));
   *    rules.push_back( SubstitutionRule( "velocity.#", "name.#", "@.vel" ));
   *    rules.push_back( SubstitutionRule( "effort.#", "name.#", "@.eff" ));
   *
   * These rules are pretty easy to use. For instance, let's consider the following example:
   *
   *   the rule   `SubstitutionRule( "position.#", "name.#", "@.pos" )`
   *   is using   `JointState.name.0 = first_joint`
   *   to convert `JointState.position.0 = 11`
   *   into       `JointState.first_joint.pos = 11`
   *
   *
   *  1. The first argument, __"position.#"__, means: "find any element in `ROSTypeFlat::value` which contains the pattern [position.#] where __#__ is a number".
   *
   *     JointState.position.0 = 11
   *
   * 2. The second argument, __"name.#"__, means: "find the element in `ROSTypeFlat::name` which contains the pattern [name.#] where __#__ is the __same__ number found in the previous pattern".
   *
   *     JointState.name.0 = first_joint
   *
   * 3. The third argument, __"@.pos"__, means: "substitute the pattern found in 1. with this string, where the symbol __@__ represent the name found in 2". The final result is therefore:
   *
   *     JointState.first_joint.pos = 11
   *
   *
   * @param pattern        The pattern to be found in ROSTypeFlat::value.
   * @param alias          The name_id that substitutes the number in the pattern. To be found in ROSTypeFlat::name.
   * @param substitution   The way the alias should be used to substitute the pattern in ROSTypeFlat::renamed_value.
   */
  SubstitutionRule(const char* pattern, const char* alias, const char* substitution);

  const std::vector<SString>& pattern() const { return _pattern; }
  const std::vector<SString>& alias() const { return _alias; }
  const std::vector<SString>& substitution() const { return _substitution; }


private:
  std::vector<SString> _pattern;
  std::vector<SString> _alias;
  std::vector<SString> _substitution;
} ;

typedef std::map< std::string, std::vector< RosIntrospection::SubstitutionRule > > SubstitutionRuleMap;

void applyNameTransform(const std::vector<SubstitutionRule> &rules,
                        ROSTypeFlat* container);


} //end namespace

#endif // ROS_INTROSPECTION_RENAMER_H
