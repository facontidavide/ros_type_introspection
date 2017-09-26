/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright 2016 Davide Faconti
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
********************************************************************/


#include <boost/algorithm/string.hpp>
#include <boost/utility/string_ref.hpp>
#include "ros_type_introspection/renamer.hpp"
#include "ros_type_introspection/ros_introspection.hpp"

namespace RosIntrospection{

inline bool isNumberPlaceholder( const SString& s)
{
  return s.size() == 1 && s.at(0) == '#';
}

inline bool isSubstitutionPlaceholder( const SString& s)
{
  return s.size() == 1 && s.at(0) == '@';
}

// given a leaf of the tree, that can have multiple index_array,
// find the only index which corresponds to the # in the pattern
int  PatternMatchAndIndexPosition(const StringTreeLeaf& leaf,
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


void Parser::applyNameTransform(const std::string& msg_identifier,
                                const ROSTypeFlat& container,
                                RenamedValues *renamed_value )
{
  const std::vector<RulesCache>& rules_cache = _registered_rules[msg_identifier];

  const size_t num_values = container.value.size();
  const size_t num_names  = container.name.size();

  renamed_value->resize( container.value.size() );
  //DO NOT clear() renamed_value

  static std::vector<int> alias_array_pos;
  static std::vector<SString> formatted_string;
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

    if( !pattern_head ) continue;
    if( !alias_head ) continue;

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
        const SString* new_name = nullptr;

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
          int char_count = 0;
          static std::vector<const SString*> concatenated_name;
          concatenated_name.reserve( 10 );
          concatenated_name.clear();

          const StringTreeNode* node_ptr = leaf.node_ptr;

          int position = leaf.array_size - 1;

          while( node_ptr != pattern_head)
          {
            const SString* value = &node_ptr->value();

            if( isNumberPlaceholder( *value) ){
              char buffer[16];
              int str_size = print_number( buffer, leaf.index_array[position--] );
              formatted_string.push_back( SString(buffer, str_size) );
              value = &formatted_string.back();
            }

            char_count += value->size();
            concatenated_name.push_back( value );
            node_ptr = node_ptr->parent();
          }

          for (int s = rule.substitution().size()-1; s >= 0; s--)
          {
            const SString* value = &rule.substitution()[s];

            if( isSubstitutionPlaceholder( *value) ) {
              value = new_name;
              position--;
            }

            char_count += value->size();
            concatenated_name.push_back( value );
          }

          for (size_t p = 0; p < rule.pattern().size() && node_ptr; p++)
          {
            node_ptr = node_ptr->parent();
          }

          while( node_ptr )
          {
            const SString* value = &node_ptr->value();

            if( isNumberPlaceholder( *value) ){
              char buffer[16];
              int str_size = print_number( buffer, leaf.index_array[position--] );
              formatted_string.push_back( SString(buffer, str_size) );
              value = &formatted_string.back();
            }

            char_count += value->size();
            concatenated_name.push_back( value );
            node_ptr = node_ptr->parent();
          }

          //------------------------
          std::string& new_identifier = (*renamed_value)[renamed_index].first;
          new_identifier.clear();

          for (int c = concatenated_name.size()-1; c >= 0; c--)
          {
            new_identifier.append( concatenated_name[c]->data(),
                                   concatenated_name[c]->size() );
            if( c>0 ) new_identifier.append("/",1);
          }

          (*renamed_value)[renamed_index].second  = value_leaf.second ;
          renamed_index++;
          substituted[i] = true;

        }// end if( new_name )
        else {

        }
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

SubstitutionRule::SubstitutionRule(const char *pattern, const char *alias, const char *substitution)
{
  std::vector<std::string> split_text;
  boost::split(split_text, pattern, boost::is_any_of("./"));

  _pattern.reserve(split_text.size());
  for (const auto& part: split_text){
    if(part.size()>0)  _pattern.push_back( part );
  }

  boost::split(split_text, alias, boost::is_any_of("./"));

  _alias.reserve(split_text.size());
  for (const auto& part: split_text){
    if(part.size()>0)  _alias.push_back( part );
  }

  boost::split(split_text, substitution, boost::is_any_of("./"));

  _substitution.reserve(split_text.size());
  for (const auto& part: split_text){
    if(part.size()>0)  _substitution.push_back( part );
  }
  size_t h1 = std::hash<std::string>{}(pattern);
  size_t h2 = std::hash<std::string>{}(alias);
  size_t h3 = std::hash<std::string>{}(substitution);
  _hash = (h1 ^ (h2 << 1)) ^ (h3 << 1 );
}

inline bool FindPattern(const std::vector<SString> &pattern,
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
  for(const auto& it: _registred_messages)
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
}



} //end namespace
