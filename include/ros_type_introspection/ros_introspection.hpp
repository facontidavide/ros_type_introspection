#ifndef ROS_INTROSPECTION_HPP
#define ROS_INTROSPECTION_HPP

#include <ros_type_introspection/renamer.hpp>
#include <ros_type_introspection/deserializer.hpp>
#include <ros_type_introspection/parser.hpp>
#include <set>

namespace RosIntrospection{

class Parser{

public:
  Parser(): _global_warnings(&std::cerr) {}

  /**
   * @brief A single message definition will (most probably) generate myltiple ROSMessage(s).
   * In fact the "child" ROSTypes are parsed as well in a recursive and hierarchical way.
   * To make an example, given as input the [geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html)
   * the result will be a ROSTypeList containing Pose, Point and Quaternion.
   *
   * @param msg_identifier name to give to the main type to be extracted.
   *
   * @param msg_definition text obtained by either:
   *                       - topic_tools::ShapeShifter::getMessageDefinition()
   *                       - rosbag::MessageInstance::getMessageDefinition()
   *                       - ros::message_traits::Definition< __your_type__ >::value()
   */
  void registerMessageDefinition(const std::string& message_identifier,
                                 const ROSType &main_type,
                                 const std::string& definition);

  void registerRenamingRules(const std::string& message_identifier,
                             const std::vector<SubstitutionRule> &rules );

  const ROSMessageInfo* getMessageInfo(const std::string& msg_identifier);

  const ROSMessage *getMessageByType(const ROSType& type, const ROSMessageInfo &info);

  void deserializeIntoFlatContainer(const std::string& msg_identifier,
                                    const nonstd::VectorView<uint8_t>& buffer,
                                    ROSTypeFlat* flat_container_output,
                                    const uint32_t max_array_size );

  void applyNameTransform(const std::string& msg_identifier,
                          const ROSTypeFlat& container,
                          RenamedValues* renamed_value );

private:

  std::map<std::string,ROSMessageInfo> _registred_messages;

  struct RulesCache{
    RulesCache( const SubstitutionRule& other):
      rule(other), pattern_head(nullptr), alias_head(nullptr)
    {}
    SubstitutionRule rule;
    const StringTreeNode* pattern_head;
    const StringTreeNode* alias_head;
  };

  std::map<std::string, std::vector<RulesCache>> _registered_rules;

  void createTrees(ROSMessageInfo &info, const std::string &type_name);

  void deserializeImpl(const ROSMessageInfo & info,
                       const MessageTreeNode *msg_node,
                       StringTreeLeaf tree_leaf, // copy, not reference
                       const nonstd::VectorView<uint8_t>& buffer,
                       size_t& buffer_offset,
                       ROSTypeFlat* flat_container,
                       const uint32_t max_array_size,
                       bool do_store);

  std::ostream* _global_warnings;
};

}

#endif // ROS_INTROSPECTION_HPP
