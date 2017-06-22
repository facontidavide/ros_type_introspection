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

#include <ros_type_introspection/renamer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/utility/string_ref.hpp>

namespace RosIntrospection{

inline bool isNumberPlaceholder( const SString& s)
{
  return s.size() == 1 && s.at(0) == '#';
}

inline bool isSubstitutionPlaceholder( const SString& s)
{
  return s.size() == 1 && s.at(0) == '@';
}

inline bool FindPattern( const std::vector<SString>& pattern,  size_t index,
                         const StringTreeNode* tail,
                         const  StringTreeNode** head )
{
  if(  tail->value()  == pattern[index])
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


void applyNameTransform(const std::vector<SubstitutionRule>& rules,
                        ROSTypeFlat* container )
{

  const bool debug = false ;

  if( debug) std::cout << container->tree << std::endl;

  container->renamed_value.resize( container->value.size() );

  std::vector<uint8_t> substituted( container->value.size() );
  for(auto& sub: substituted) { sub = false; }

  size_t renamed_index = 0;

  std::vector<int> alias_array_pos( container->name.size() );
  std::vector<SString> formatted_string;
  formatted_string.reserve(20);

  for(const auto& rule: rules)
  {
    const StringTreeNode* pattern_head = nullptr;
    const StringTreeNode* alias_head = nullptr;

    FindPattern( rule.pattern(), 0, container->tree.croot(), &pattern_head );
    if( !pattern_head) continue;

    FindPattern( rule.alias(),   0, container->tree.croot(), &alias_head );
    if(!alias_head) continue;

    for (int n=0; n< container->name.size(); n++)
    {
      const StringTreeLeaf& alias_leaf = container->name[n].first;
      alias_array_pos[n] = PatternMatchAndIndexPosition(alias_leaf, alias_head);
    }

    for(size_t i=0; i< container->value.size(); i++)
    {
      if( substituted[i]) continue;

      const auto& value_leaf = container->value[i];

      const StringTreeLeaf& leaf = value_leaf.first;

      int pattern_array_pos = PatternMatchAndIndexPosition(leaf,   pattern_head);

      if( pattern_array_pos>= 0) // -1 if pattern doesn't match
      {
        if( debug) std::cout << " match pattern... ";

        const SString* new_name = nullptr;

        for (int n=0; n< container->name.size(); n++)
        {
          const auto & it = container->name[n];
          const StringTreeLeaf& alias_leaf = it.first;

          if( alias_array_pos[n] >= 0 ) // -1 if pattern doesn't match
          {
            if( alias_leaf.index_array[ alias_array_pos[n] ] ==
                leaf.index_array[ pattern_array_pos] )
            {
              if( debug) std::cout << " substitution: " << it.second << std::endl;
              new_name =  &(it.second);
              break;
            }
          }
        }

        //--------------------------
        if( new_name )
        {
          int char_count = 0;
          std::vector<const SString*> concatenated_name;
          concatenated_name.reserve( 10 );

          const StringTreeNode* node_ptr = leaf.node_ptr;

          int position = leaf.array_size - 1;

          while( node_ptr != pattern_head)
          {
            const SString* value = &node_ptr->value();

            if( isNumberPlaceholder( *value) ){
              char buffer[16];
              sprintf( buffer,"%d", leaf.index_array[position--]);
              formatted_string.push_back( std::move(SString(buffer)) );
              value = &formatted_string.back();
            }

            char_count += value->size();
            if( debug) std::cout << "A: " << *value << std::endl;;
            concatenated_name.push_back( std::move(value) );
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
            if( debug) std::cout << "B: " << *value << std::endl;;
            concatenated_name.push_back( std::move(value) );
          }

          for (int p = 0; p < rule.pattern().size() && node_ptr; p++)
          {
            node_ptr = node_ptr->parent();
          }

          while( node_ptr )
          {
            const SString* value = &node_ptr->value();

            if( isNumberPlaceholder( *value) ){
              char buffer[16];
              sprintf( buffer,"%d", leaf.index_array[position--]);
              formatted_string.push_back( std::move(SString(buffer)) );
              value = &formatted_string.back();
            }

            char_count += value->size();
            if( debug) std::cout << "C: " << *value << std::endl;;
            concatenated_name.push_back( std::move(value) );
            node_ptr = node_ptr->parent();
          }

          //------------------------
          SString new_identifier;

          for (int c = concatenated_name.size()-1; c >= 0; c--)
          {
            new_identifier.append( *concatenated_name[c] );
            if( c>0 ) new_identifier.append("/");
          }
          if( debug) std::cout << "Result: " << new_identifier << std::endl;

          container->renamed_value[renamed_index].first = std::move( new_identifier );
          container->renamed_value[renamed_index].second  =  value_leaf.second ;
          renamed_index++;
          substituted[i] = true;
          if( debug) std::cout << std::endl;
        }// end if( new_name )
        else {
          if( debug ) std::cout << "NO substitution" << std::endl;
        }
      }// end if( PatternMatching )
      else  {
        if( debug ) std::cout << "NO MATCHING" << std::endl;
      }
    } // end for values
  } // end for rules

//  static std::map<const StringTreeLeaf*, SString> cache;

  for(size_t i=0; i< container->value.size(); i++)
  {
    if( substituted[i] == false)
    {
      const std::pair<StringTreeLeaf, VarNumber> & value_leaf = container->value[i];

      container->renamed_value[renamed_index].first  = value_leaf.first.toStr();
      container->renamed_value[renamed_index].second = value_leaf.second ;
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
}




} //end namespace
