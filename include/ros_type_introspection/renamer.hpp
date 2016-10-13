#ifndef ROS_INTROSPECTION_RENAMER_H
#define ROS_INTROSPECTION_RENAMER_H

#include <ros_type_introspection/deserializer.hpp>

namespace RosIntrospection{


class SubstitutionRule {
public:

  /**
   * @brief Pass the three arguments (pattern, alias, substitution) as point separated strings.
   *
   * @param pattern        The pattern to be found in ROSTypeFlat::value
   * @param alias          The name_id that substitutes the number in the pattern. To be found in ROSTypeFlat::name
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
