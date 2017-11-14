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

#ifndef ROS_INTROSPECTION_HPP
#define ROS_INTROSPECTION_HPP

#include <unordered_set>
#include "ros_type_introspection/stringtree_leaf.hpp"
#include "ros_type_introspection/substitution_rule.hpp"
#include "absl/types/span.h"

namespace RosIntrospection{

struct FlatMessage {

  /// Tree that the StringTreeLeaf(s) refer to.
  const StringTree* tree;

  /// List of all those parsed fields that can be represented by a builtin value different from "string".
  /// This list will be filled by the funtion buildRosFlatType.
  std::vector< std::pair<StringTreeLeaf, Variant> > value;

  /// List of all those parsed fields that can be represented by a builtin value equal to "string".
  /// This list will be filled by the funtion buildRosFlatType.
  std::vector< std::pair<StringTreeLeaf, std::string> > name;

  // Not used yet
  std::vector< std::pair<StringTreeLeaf, std::vector<uint8_t>>> blob;
};

typedef std::vector< std::pair<std::string, Variant> > RenamedValues;

class Parser{

public:
  Parser(): _global_warnings(&std::cerr), _rule_cache_dirty(true) {}

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
  const ROSMessageInfo* getMessageInfo(const std::string& msg_identifier) const;

  /**
   * @brief getMessageByType provides a pointer to a ROSMessage stored in ROSMessageInfo.
   *
   * @param type   ROSType to be found
   * @param info   Instance or ROSMessageInfo that shall contain the ROSType to be found.
   * @return       Pointer to the instance or nullptr if not registered.
   */
  const ROSMessage *getMessageByType(const ROSType& type, const ROSMessageInfo &info) const;

  /**
   * @brief deserializeIntoFlatContainer takes a raw buffer of memory and extract information from it.
   *  This data is stored in two key/value vectors, FlatMessage::value and FlatMessage::name.
   * It must be noted that the key type is StringTreeLeaf. this type is not particularly user-friendly,
   * but allows a much faster post-processing.
   *
   * IMPORTANT: this approach is not meant to be used with use arrays such as maps, point clouds and images.
   * It would require a ridicoulous amount of memory and, franckly, make little sense.
   * For this reason the argument max_array_size is used.
   *
   * This funtion is almost always followed by applyNameTransform, which provide a more human-readable
   * key-value representation.
   *
   * @param msg_identifier   String ID to identify the registered message (use registerMessageDefinition first).
   * @param buffer           raw memory to be parsed.
   * @param flat_container_output  output to store the result. It is recommended to reuse the same object multiple times to
   *                               avoid memory allocations and speed up the parsing.
   * @param max_array_size   Usually we want to avoid special cases like maps and images, which contain very large arrays.
   *                         max_array_size is used to skip these arrays that are too large.
   *
   * return true if the entire message was parsed or false if parts of the message were
   * skipped because an array has (size > max_array_size)
   */
  bool deserializeIntoFlatContainer(const std::string& msg_identifier,
                                    absl::Span<uint8_t> buffer,
                                    FlatMessage* flat_container_output,
                                    const uint32_t max_array_size ) const;

  /**
   * @brief applyNameTransform is used to create a vector of type RenamedValues from
   *        the vector FlatMessage::value. Additionally, it apply the renaming rules previously
   *        registred using registerRenamingRules.
   *
   * For example if you apply this to [geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html)
   * the vector renamed_value.value will contain the following pairs (where ... is the number of that field) :
   *
   *  - Pose/Point/x = ...
   *  - Pose/Point/y = ...
   *  - Pose/Point/z = ...
   *  - Pose/Quaternion/x = ...
   *  - Pose/Quaternion/y = ...
   *  - Pose/Quaternion/z = ...
   *  - Pose.Quaternion/w = ...
   *
   * @param msg_identifier  String ID to identify the registered message (use registerMessageDefinition first).
   * @param container       Source. This instance must be created using deserializeIntoFlatContainer.
   * @param renamed_value   Destination.
   */
  void applyNameTransform(const std::string& msg_identifier,
                          const FlatMessage& container,
                          RenamedValues* renamed_value );

  typedef std::function<void(const ROSType&, absl::Span<uint8_t>&)> VisitingCallback;

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
                            absl::Span<uint8_t> &buffer,
                            VisitingCallback callback) const;

  /// Change where the warning messages are displayed.
  void setWarningsStream(std::ostream* output) { _global_warnings = output; }

private:


  struct RulesCache{
    RulesCache( const SubstitutionRule& r):
      rule( &r ), pattern_head(nullptr), alias_head(nullptr)
    {}
    const SubstitutionRule* rule;
    const StringTreeNode* pattern_head;
    const StringTreeNode* alias_head;
    bool operator==(const RulesCache& other) { return  this->rule == other.rule; }
  };

  std::unordered_map<std::string, ROSMessageInfo> _registered_messages;
  std::unordered_map<ROSType,     std::unordered_set<SubstitutionRule>>   _registered_rules;
  std::unordered_map<std::string, std::vector<RulesCache>>  _rule_caches;

  void updateRuleCache();

  bool _rule_cache_dirty;

  void createTrees(ROSMessageInfo &info, const std::string &type_name) const;

  std::ostream* _global_warnings;

  std::vector<int> _alias_array_pos;
  std::vector<std::string> _formatted_string;
  std::vector<int8_t> _substituted;
};

}

#endif // ROS_INTROSPECTION_HPP
