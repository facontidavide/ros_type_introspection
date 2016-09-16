#ifndef ROS_INTROSPECTION_RENAMER_H
#define ROS_INTROSPECTION_RENAMER_H

#include <ros_type_introspection/deserializer.hpp>

namespace RosIntrospection{


typedef struct {
    std::vector<SString> pattern;
    std::vector<SString> location;
    std::vector<SString> substitution;
} SubstitutionRule;


typedef std::map<std::string, SubstitutionRule> SubstitutionRuleSet;


void applyNameTransform(const std::vector<SubstitutionRule> &rules,
                        ROSTypeFlat* container);


} //end namespace

#endif // ROS_INTROSPECTION_RENAMER_H
