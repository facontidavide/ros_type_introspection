#ifndef ROS_INTROSPECTION_RENAMER_H
#define ROS_INTROSPECTION_RENAMER_H

#include <ros_type_introspection/deserializer.hpp>

namespace RosIntrospection{

class SubstitutionRule{
public:
    SubstitutionRule(boost::string_ref pattern, boost::string_ref name_location, boost::string_ref substitution);

    std::vector<SString> pattern_suf;
    std::vector<SString> pattern_pre;

    std::vector<SString> location_suf;
    std::vector<SString> location_pre;

    std::vector<SString> substitution_suf;
    std::vector<SString> substitution_pre;

private:

    void split_boost_ref(boost::string_ref& s,
                         std::vector<SString>& pre,
                         std::vector<SString>& post);

};

const StringElement* FindPatternTail( const SString& value,
                                      const StringElement* tail);

bool FindPattern( const std::vector<SString>& pattern,  size_t index,
                  const StringElement* tail,
                  std::vector<const  StringElement*>& heads );

typedef std::map<std::string, SubstitutionRule> SubstitutionRuleSet;
/*
void applyNameTransform(const std::vector<SubstitutionRule> &rules,
                        ROSTypeFlat* container);*/


} //end namespace

#endif // ROS_INTROSPECTION_RENAMER_H
