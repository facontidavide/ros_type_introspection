#ifndef ROS_INTROSPECTION_RENAMER_H
#define ROS_INTROSPECTION_RENAMER_H

#include <ros_type_introspection/deserializer.hpp>

namespace RosIntrospection{


typedef struct {
    std::vector<SString> pattern;
    std::vector<SString> location;
    std::vector<SString> substitution;
} SubstitutionRule;

const StringElement* FindPatternTail( const SString& value,
                                      const StringElement* tail);

bool FindPattern( const std::vector<SString>& pattern,  size_t index,
                  const StringElement* tail,
                  std::vector<const  StringElement*>& heads );

typedef std::map<std::string, SubstitutionRule> SubstitutionRuleSet;


bool PatternMatch(const StringElement* head, const StringElement* node_ptr );


/*
void applyNameTransform(const std::vector<SubstitutionRule> &rules,
                        ROSTypeFlat* container);*/


} //end namespace

#endif // ROS_INTROSPECTION_RENAMER_H
