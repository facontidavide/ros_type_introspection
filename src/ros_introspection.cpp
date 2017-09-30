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

#include "ros_type_introspection/ros_introspection.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/utility/string_ref.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <functional>
#include "ros_type_introspection/helper_functions.hpp"

namespace RosIntrospection {

void Parser::createTrees(ROSMessageInfo& info, const std::string &type_name) const
{
  std::function<void(const ROSMessage*, StringTreeNode*, MessageTreeNode* )> recursiveTreeCreator;

  recursiveTreeCreator = [&](const ROSMessage* msg_definition, StringTreeNode* string_node, MessageTreeNode* msg_node)
  {
    // note: should use reserve here, NOT resize
    const size_t NUM_FIELDS = msg_definition->fields().size();

    string_node->children().reserve( NUM_FIELDS );
    msg_node->children().reserve( NUM_FIELDS );

    for (const ROSField& field : msg_definition->fields() )
    {
      if(field.isConstant() == false) {

        // Let's add first a child to string_node
        string_node->addChild( field.name() );
        StringTreeNode*  new_string_node = &(string_node->children().back());
        if( field.isArray())
        {
          new_string_node->children().reserve(1);
          new_string_node = new_string_node->addChild("#");
        }

        const ROSMessage* next_msg = nullptr;
        // builtin types will not trigger a recursion
        if( field.type().isBuiltin() == false)
        {
          next_msg = getMessageByType( field.type(), info );
          if( next_msg == nullptr)
          {
            throw std::runtime_error("This type was not registered " );
          }
          msg_node->addChild( next_msg );
          MessageTreeNode* new_msg_node = &(msg_node->children().back());
          recursiveTreeCreator(next_msg, new_string_node, new_msg_node);
        }
      } //end of field.isConstant()
    } // end of for fields
  };//end of lambda

  info.string_tree.root()->setValue( type_name );
  info.message_tree.root()->setValue( &info.type_list.front() );
  //TODO info.type_tree.root()->value() =
  // start recursion
  recursiveTreeCreator( &info.type_list.front(),
                        info.string_tree.root(),
                        info.message_tree.root());
}

inline bool FindPattern(const std::vector<std::string> &pattern,
                        size_t index, const StringTreeNode *tail,
                        const StringTreeNode **head)
{
  if(  tail->value() == pattern[index])
  {
    index++;
  }
  else{ // mismatch
    if( index > 0 ){
      // reset counter;
      FindPattern( pattern, 0, tail, head);
      return false;
    }
    index = 0;
  }

  if( index == pattern.size()){
    *head = ( tail );
    return true;
  }

  bool found = false;

  for (auto& child: tail->children() ) {

    found = FindPattern( pattern, index, &child, head);
    if( found) break;
  }
  return found;
}


void Parser::registerRenamingRules(const ROSType &type, const std::vector<SubstitutionRule> &rules)
{
  for(const auto& it: _registered_messages)
  {
    const std::string& msg_identifier = it.first;
    const ROSMessageInfo& msg_info    = it.second;
    if( getMessageByType(type, msg_info) )
    {
      std::vector<RulesCache>&  cache_vector = _registered_rules[msg_identifier];
      for(const auto& rule: rules )
      {
        RulesCache cache(rule);
        FindPattern( cache.rule.pattern(), 0, msg_info.string_tree.croot(), &cache.pattern_head );
        FindPattern( cache.rule.alias(),   0, msg_info.string_tree.croot(), &cache.alias_head );
        if( cache.pattern_head && cache.alias_head
           && std::find( cache_vector.begin(), cache_vector.end(), cache) == cache_vector.end()
            )
        {
          cache_vector.push_back( std::move(cache) );
        }
      }
    }
  }
  _block_register_message = true;
}


void Parser::registerMessageDefinition(const std::string &message_identifier,
                                       const ROSType &main_type,
                                       const std::string &definition)
{
  if( _block_register_message )
  {
    throw std::runtime_error("You have called registerRenamingRules already."
                             " It is forbidden to use registerMessageDefinition again" );
  }

  const boost::regex msg_separation_regex("^=+\\n+");

  static std::vector<std::string> split;
  split.clear();
  static std::vector<const ROSType*> all_types;
  all_types.clear();


  boost::split_regex(split, definition, msg_separation_regex);

  ROSMessageInfo info;
  info.type_list.reserve( split.size() );

  for (size_t i = 0; i < split.size(); ++i)
  {
    ROSMessage msg( split[i] );
    if( i == 0)
    {
      msg.mutateType( main_type );
    }

    info.type_list.push_back( std::move(msg) );
    all_types.push_back( &(info.type_list.back().type()) );
  }

  for( ROSMessage& msg: info.type_list )
  {
    msg.updateMissingPkgNames( all_types );
  }
  //------------------------------

  createTrees(info, message_identifier);

  //  std::cout << info.string_tree << std::endl;
  //  std::cout << info.message_tree << std::endl;
  _registered_messages.insert( std::make_pair(message_identifier, std::move(info) ) );
}

const ROSMessageInfo *Parser::getMessageInfo(const std::string &msg_identifier) const
{
  auto it = _registered_messages.find(msg_identifier);
  if( it != _registered_messages.end() )
  {
    return &(it->second);
  }
  return nullptr;
}

const ROSMessage* Parser::getMessageByType(const ROSType &type, const ROSMessageInfo& info) const
{
  for(const ROSMessage& msg: info.type_list) // find in the list
  {
    if( msg.type() == type )
    {
      return &msg;
    }
  }
  return nullptr;
}

void Parser::applyVisitorToBuffer(const std::string &msg_identifier,
                                  const ROSType& monitored_type,
                                  absl::Span<uint8_t> &buffer,
                                  Parser::VisitingCallback callback) const
{
  const ROSMessageInfo* msg_info = getMessageInfo(msg_identifier);

  if( msg_info == nullptr)
  {
    throw std::runtime_error("deserializeIntoFlatContainer: msg_identifier not registered. Use registerMessageDefinition" );
  }

  std::function<void(const MessageTreeNode*)> recursiveImpl;
  size_t buffer_offset = 0;

  recursiveImpl = [&](const MessageTreeNode* msg_node)
  {
    const ROSMessage* msg_definition = msg_node->value();
    const ROSType& msg_type = msg_definition->type();

    const bool matching = ( msg_type == monitored_type );

    uint8_t* prev_buffer_ptr = buffer.data() + buffer_offset;
    size_t prev_offset = buffer_offset;

    size_t index_m = 0;

    for (const ROSField& field : msg_definition->fields() )
    {
      if(field.isConstant() ) continue;

      const ROSType&  field_type = field.type();

      int32_t array_size = field.arraySize();
      if( array_size == -1)
      {
        ReadFromBuffer( buffer, buffer_offset, array_size );
      }

      //------------------------------------

      if( field_type.isBuiltin() )
      {
        for (int i=0; i<array_size; i++ )
        {
          //Skip
          ReadFromBufferToVariant( field_type.typeID(), buffer, buffer_offset );
        }
      }
      else{
        // field_type.typeID() == OTHER
        for (int i=0; i<array_size; i++ )
        {
          recursiveImpl( msg_node->child(index_m) );
        }
        index_m++;
      }
    } // end for fields
    if( matching )
    {
      absl::Span<uint8_t> view( prev_buffer_ptr, buffer_offset - prev_offset);
      callback( monitored_type, view );
    }
  }; //end lambda

  //start recursion
  recursiveImpl( msg_info->message_tree.croot() );
}

void Parser::deserializeIntoFlatContainer(const std::string& msg_identifier,
                                          absl::Span<uint8_t> buffer,
                                          FlatMessage* flat_container,
                                          const uint32_t max_array_size ) const
{

  const ROSMessageInfo* msg_info = getMessageInfo(msg_identifier);

  size_t value_index = 0;
  size_t name_index = 0;


  if( msg_info == nullptr)
  {
    throw std::runtime_error("deserializeIntoFlatContainer: msg_identifier not registerd. Use registerMessageDefinition" );
  }
  size_t buffer_offset = 0;

  std::function<void(const MessageTreeNode*,StringTreeLeaf,bool)> deserializeImpl;

  deserializeImpl = [&](
      const MessageTreeNode* msg_node,
      const StringTreeLeaf& tree_leaf,
      bool DO_STORE)
  {
      const ROSMessage* msg_definition = msg_node->value();
      size_t index_s = 0;
      size_t index_m = 0;

      for (const ROSField& field : msg_definition->fields() )
      {
        if(field.isConstant() ) continue;

        const ROSType&  field_type = field.type();

        auto new_tree_leaf = tree_leaf;
        new_tree_leaf.node_ptr = tree_leaf.node_ptr->child(index_s);

        int32_t array_size = field.arraySize();
        if( array_size == -1)
        {
          ReadFromBuffer( buffer, buffer_offset, array_size );
        }
        if( field.isArray())
        {
          new_tree_leaf.array_size++;
          new_tree_leaf.node_ptr = new_tree_leaf.node_ptr->child(0);
        }

        if( array_size > static_cast<int>(max_array_size) ) DO_STORE = false;

        //------------------------------------
        for (int i=0; i<array_size; i++ )
        {
          if( field.isArray() )
          {
            new_tree_leaf.index_array[ new_tree_leaf.array_size-1 ] = i;
          }

          if( field_type.typeID() == STRING )
          {
            if( flat_container->name.size() <= name_index)
            {
              const size_t increased_size = std::max( size_t(32), flat_container->name.size() *  3/2);
              flat_container->name.resize( increased_size );
            }

            std::string& name = flat_container->name[name_index].second;//read directly inside an existing std::string
            ReadFromBuffer<std::string>( buffer, buffer_offset, name );

            if( DO_STORE )
            {
              flat_container->name[name_index].first = new_tree_leaf ;
              name_index++;
            }
          }
          else if( field_type.isBuiltin() )
          {
            if( flat_container->value.size() <= value_index)
            {
              const size_t increased_size = std::max( size_t(32), flat_container->value.size() *  3/2);
              flat_container->value.resize( increased_size );
            }

            Variant var = ReadFromBufferToVariant( field_type.typeID(),
                                                   buffer,
                                                   buffer_offset );           
            if( DO_STORE )
            {
              flat_container->value[value_index] = std::make_pair( new_tree_leaf, std::move(var) );
              value_index++;
            }
            else{
              ReadFromBufferToVariant( field_type.typeID(), buffer, buffer_offset );
            }
          }
          else{ // field_type.typeID() == OTHER

            deserializeImpl(msg_node->child(index_m),
                            new_tree_leaf,
                            DO_STORE);
          }
        } // end for array_size

        if( field_type.typeID() == OTHER )
        {
          index_m++;
        }
        index_s++;
      } // end for fields
  }; //end of lambda


  flat_container->tree = &msg_info->string_tree;

  StringTreeLeaf rootnode;
  rootnode.node_ptr = msg_info->string_tree.croot();

  deserializeImpl( msg_info->message_tree.croot(),
                   rootnode,
                   true);

  flat_container->name.resize( name_index );
  flat_container->value.resize( value_index );

  if( buffer_offset != buffer.size() )
  {
    throw std::runtime_error("buildRosFlatType: There was an error parsing the buffer" );
  }
}

inline bool isNumberPlaceholder( const std::string& s)
{
  return s.size() == 1 && s.at(0) == '#';
}

inline bool isSubstitutionPlaceholder( const std::string& s)
{
  return s.size() == 1 && s.at(0) == '@';
}

// given a leaf of the tree, that can have multiple index_array,
// find the only index which corresponds to the # in the pattern
inline int  PatternMatchAndIndexPosition(const StringTreeLeaf& leaf,
                                  const StringTreeNode* pattern_head )
{
  const StringTreeNode* node_ptr = leaf.node_ptr;

  int pos = leaf.array_size-1;

  while( node_ptr )
  {
    if( node_ptr != pattern_head )
    {
      if( isNumberPlaceholder( node_ptr->value() ))
      {
        pos--;
      }
    }
    else{
      return pos;
    }
    node_ptr = node_ptr->parent();
  } // end while
  return -1;
}

inline void JoinStrings( const std::vector<const std::string*>& vect, const char separator, std::string& destination)
{
  size_t count = 0;
  for(const auto &v: vect ) count += v->size();

  // the following approach seems to be faster
  // https://github.com/facontidavide/InterestingBenchmarks/blob/master/StringAppend_vs_Memcpy.md

  destination.resize( count + vect.size() -1 );

  char* buffer = &destination[0];
  size_t buff_pos = 0;

  for (size_t c = 0; c < vect.size()-1; c++)
  {
    const size_t S = vect[c]->size();
    memcpy( &buffer[buff_pos], vect[c]->data(), S );
    buff_pos += S;
    buffer[buff_pos++] = separator;
  }
  memcpy( &buffer[buff_pos], vect.back()->data(), vect.back()->size() );
}

void Parser::applyNameTransform(const std::string& msg_identifier,
                                const FlatMessage& container,
                                RenamedValues *renamed_value ) const
{
  const std::vector<RulesCache>& rules_cache = _registered_rules.find(msg_identifier)->second;

  const size_t num_values = container.value.size();
  const size_t num_names  = container.name.size();

  renamed_value->resize( container.value.size() );
  //DO NOT clear() renamed_value

  static std::vector<int> alias_array_pos;
  static std::vector<std::string> formatted_string;
  static std::vector<int8_t> substituted;

  alias_array_pos.reserve( num_names );
  alias_array_pos.clear();
  formatted_string.reserve( num_values );
  formatted_string.clear();

  substituted.resize( num_values );
  for(size_t i=0; i<num_values; i++) { substituted[i] = false; }

  size_t renamed_index = 0;

  for(const RulesCache& cache: rules_cache)
  {
    const SubstitutionRule& rule       = cache.rule;
    const StringTreeNode* pattern_head = cache.pattern_head;
    const StringTreeNode* alias_head   = cache.alias_head;

    if( !pattern_head || !alias_head  ) continue;

    for (size_t n=0; n<num_names; n++)
    {
      const StringTreeLeaf& alias_leaf = container.name[n].first;
      alias_array_pos[n] = PatternMatchAndIndexPosition(alias_leaf, alias_head);
    }

    for(size_t i=0; i<num_values; i++)
    {
      if( substituted[i]) continue;

      const auto& value_leaf = container.value[i];

      const StringTreeLeaf& leaf = value_leaf.first;

      int pattern_array_pos = PatternMatchAndIndexPosition(leaf,   pattern_head);

      if( pattern_array_pos>= 0) // -1 if pattern doesn't match
      {
        const std::string* new_name = nullptr;

        for (size_t n=0; n < num_names; n++)
        {
          const auto & it = container.name[n];
          const StringTreeLeaf& alias_leaf = it.first;

          if( alias_array_pos[n] >= 0 ) // -1 if pattern doesn't match
          {
            if( alias_leaf.index_array[ alias_array_pos[n] ] ==
                leaf.index_array[ pattern_array_pos] )
            {
              new_name =  &(it.second);
              break;
            }
          }
        }

        //--------------------------
        if( new_name )
        {
          static std::vector<const std::string*> concatenated_name;
          concatenated_name.reserve( 10 );
          concatenated_name.clear();

          const StringTreeNode* node_ptr = leaf.node_ptr;

          int position = leaf.array_size - 1;

          while( node_ptr != pattern_head)
          {
            const std::string* value = &node_ptr->value();

            if( isNumberPlaceholder( *value) ){
              char buffer[16];
              int str_size = print_number( buffer, leaf.index_array[position--] );
              formatted_string.push_back( std::string(buffer, str_size) );
              value = &formatted_string.back();
            }

            concatenated_name.push_back( value );
            node_ptr = node_ptr->parent();
          }

          for (int s = rule.substitution().size()-1; s >= 0; s--)
          {
            const std::string* value = &rule.substitution()[s];

            if( isSubstitutionPlaceholder( *value) ) {
              value = new_name;
              position--;
            }
            concatenated_name.push_back( value );
          }

          for (size_t p = 0; p < rule.pattern().size() && node_ptr; p++)
          {
            node_ptr = node_ptr->parent();
          }

          while( node_ptr )
          {
            const std::string* value = &node_ptr->value();

            if( isNumberPlaceholder(*value) ){
              char buffer[16];
              int str_size = print_number( buffer, leaf.index_array[position--] );
              formatted_string.push_back( std::string(buffer, str_size) );
              value = &formatted_string.back();
            }
            concatenated_name.push_back( value );
            node_ptr = node_ptr->parent();
          }

          //------------------------
          auto& renamed_pair = (*renamed_value)[renamed_index];

          std::reverse(concatenated_name.begin(), concatenated_name.end());
          JoinStrings( concatenated_name, '/', renamed_pair.first);
          renamed_pair.second  = value_leaf.second ;

          renamed_index++;
          substituted[i] = true;

        }// end if( new_name )
      }// end if( PatternMatching )
      else  {

      }
    } // end for values
  } // end for rules

  for(size_t i=0; i< container.value.size(); i++)
  {
    if( !substituted[i] )
    {
      const std::pair<StringTreeLeaf, Variant> & value_leaf = container.value[i];

      std::string& destination = (*renamed_value)[renamed_index].first;
      value_leaf.first.toStr( destination );
      (*renamed_value)[renamed_index].second = value_leaf.second ;
      renamed_index++;
    }
  }
}

}


