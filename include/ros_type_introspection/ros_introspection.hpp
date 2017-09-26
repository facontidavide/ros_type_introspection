#ifndef ROS_INTROSPECTION_HPP
#define ROS_INTROSPECTION_HPP

#include <ros_type_introspection/renamer.hpp>
#include <ros_type_introspection/deserializer.hpp>
#include <ros_type_introspection/parser.hpp>
#include <set>

namespace RosIntrospection{

class Parser{

public:
  Parser(): _global_warnings(&std::cerr), _block_register_message(false) {}

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

  /**
   * @brief registerRenamingRules is used to register the renaming rules. You MUST use registerMessageDefinition first.
   *
   * @param type    The type which must be renamed
   * @param rules   A list of rules to apply to the type.
   */
  void registerRenamingRules(const ROSType& type,
                             const std::vector<SubstitutionRule> &rules );

  /**
   * @brief getMessageInfo provides some metadata amout a registered ROSMessage.
   *
   * @param msg_identifier String ID to identify the registered message (use registerMessageDefinition first).
   * @return               Pointer to the instance or nullptr if not registered.
   */
  const ROSMessageInfo* getMessageInfo(const std::string& msg_identifier);

  /**
   * @brief getMessageByType provides a pointer to a ROSMessage stored in ROSMessageInfo.
   *
   * @param type   ROSType to be found
   * @param info   Instance or ROSMessageInfo that shall contain the ROSType to be found.
   * @return       Pointer to the instance or nullptr if not registered.
   */
  const ROSMessage *getMessageByType(const ROSType& type, const ROSMessageInfo &info);

  /**
   * @brief deserializeIntoFlatContainer takes a raw buffer of memory and extract information from it.
   *  This data is stored in two key/value vectors, ROSTypeFlat::value and ROSTypeFlat::name.
   * It must be noted that the key type is StringTreeLeaf. this type is not particularly user-friendly,
   * but allows a much faster post-processing.
   * This funtion is almost always followed by applyNameTransform, which provide a more human-readable
   * key-value representation.
   *
   * @param msg_identifier   String ID to identify the registered message (use registerMessageDefinition first).
   * @param buffer           raw memory to be parsed.
   * @param flat_container_output  output to store the result. It is recommended to reuse the same object multiple times to
   *                               avoid memory allocations and speed up the parsing.
   * @param max_array_size   Usually we want to avoid special cases like maps and images, which contain very large arrays.
   *                         max_array_size is used to skip these arrays that are too large.
   */
  void deserializeIntoFlatContainer(const std::string& msg_identifier,
                                    const nonstd::VectorView<uint8_t>& buffer,
                                    ROSTypeFlat* flat_container_output,
                                    const uint32_t max_array_size );

  /**
   * @brief applyNameTransform is used to create a vector of type RenamedValues from
   *        the vector ROSTypeFlat::value. Additionally, it apply the renaming rules previously
   *        registred using registerRenamingRules.
   *
   * @param msg_identifier  String ID to identify the registered message (use registerMessageDefinition first).
   * @param container       Source. This instance must be created using deserializeIntoFlatContainer.
   * @param renamed_value   Destination.
   */
  void applyNameTransform(const std::string& msg_identifier,
                          const ROSTypeFlat& container,
                          RenamedValues* renamed_value );

  typedef std::function<void(const ROSType&, nonstd::VectorViewMutable<uint8_t>&)> VisitingCallback;

  /**
   * @brief applyVisitorToBuffer is used to pass a callback that is invoked every time
   *        a chunk of memory storing an instance of ROSType = monitored_type
   *        is reached.
   *        Note that the VisitingCallback can modify the original message, but can NOT
   *        change its size. This means that strings and vectors can not be change their length.
   *
   * @param msg_identifier    String ID to identify the registered message (use registerMessageDefinition first).
   * @param monitored_type    ROSType that triggers the invokation to the callback
   * @param buffer            Original buffer, passed as mutable since it might be modified.
   * @param callback          The callback.
   */
  void applyVisitorToBuffer(const std::string& msg_identifier, const ROSType &monitored_type,
                            nonstd::VectorViewMutable<uint8_t> &buffer,
                            VisitingCallback callback);

  /// Change where the warning messages are displayed.
  void setWarningsStream(std::ostream* output) { _global_warnings = output; }

private:

  std::map<std::string,ROSMessageInfo> _registred_messages;

  struct RulesCache{
    RulesCache( const SubstitutionRule& other):
      rule(other), pattern_head(nullptr), alias_head(nullptr)
    {}
    SubstitutionRule rule;
    const StringTreeNode* pattern_head;
    const StringTreeNode* alias_head;
    bool operator==(const RulesCache& other) { return  this->rule == other.rule; }
  };

  std::map<std::string, std::vector<RulesCache>> _registered_rules;

  void createTrees(ROSMessageInfo &info, const std::string &type_name);

  std::ostream* _global_warnings;
  bool _block_register_message;
};

}

#endif // ROS_INTROSPECTION_HPP
