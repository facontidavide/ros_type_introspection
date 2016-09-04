#ifndef ROS_INTROSPECTION_RENAMER_H
#define ROS_INTROSPECTION_RENAMER_H

#include <ros_type_introspection/deserializer.hpp>

namespace RosIntrospection{

class SubstitutionRule{
public:
    SubstitutionRule( std::string pattern, std::string name_location, std::string substitution);

    std::string pattern_suf;
    std::string pattern_pre;

    std::string location_suf;
    std::string location_pre;

    std::string substitution_suf;
    std::string substitution_pre;
private:

};

typedef std::map<std::string, SubstitutionRule> SubstitutionRuleSet;

void applyNameTransform(const std::vector<SubstitutionRule> &rules,
                        ROSTypeFlat* container);


} //end namespace

#endif // ROS_INTROSPECTION_RENAMER_H
