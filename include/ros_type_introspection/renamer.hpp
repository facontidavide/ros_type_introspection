#ifndef ROS_INTROSPECTION_RENAMER_H
#define ROS_INTROSPECTION_RENAMER_H

#include <ros_type_introspection/deserializer.hpp>

namespace RosIntrospection{

class SubstitutionRule{
public:
    SubstitutionRule(boost::string_ref pattern, boost::string_ref name_location, boost::string_ref substitution);

    std::vector<ShortString> pattern_suf;
    std::vector<ShortString> pattern_pre;

    std::vector<ShortString> location_suf;
    std::vector<ShortString> location_pre;

    std::vector<ShortString> substitution_suf;
    std::vector<ShortString> substitution_pre;

private:

    void split_boost_ref(boost::string_ref& s,
                         std::vector<ShortString>& pre,
                         std::vector<ShortString>& post);

};

const StringElement* FindPatternTail( const ShortString& value,
                                      const StringElement* tail);

bool FindPattern( const std::vector<ShortString>& pattern,  size_t index,
                  const StringElement* tail,
                  std::vector<const  StringElement*>& heads );

typedef std::map<std::string, SubstitutionRule> SubstitutionRuleSet;
/*
void applyNameTransform(const std::vector<SubstitutionRule> &rules,
                        ROSTypeFlat* container);*/


} //end namespace

#endif // ROS_INTROSPECTION_RENAMER_H
