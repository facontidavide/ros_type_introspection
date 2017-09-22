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

#include <functional>
#include "ros_type_introspection/deserializer.hpp"


namespace RosIntrospection{

extern std::ostream* _global_warnings_stream_;


void buildRosFlatTypeImpl(const std::vector<ROSMessage> & type_list,
                          const ROSType &type,
                          StringTreeLeaf tree_leaf, // easier to use copy instead of reference or pointer
                          const nonstd::VectorView<uint8_t>& buffer,
                          size_t& buffer_offset,
                          ROSTypeFlat* flat_container,
                          const uint32_t max_array_size,
                          bool do_store)
{

  int32_t array_size = type.arraySize();
  if( array_size == -1)
  {
    ReadFromBuffer( buffer, buffer_offset, array_size );
  }

  //---------------------------------------------------------------------------
  // we store in a function pointer the operation to be done later
  // This operation is different according to the typeID
  std::function<void(StringTreeLeaf, bool)> deserializeAndStore;

  if( type.typeID() == STRING )
  {
    deserializeAndStore = [&](StringTreeLeaf tree_node, bool STORE_RESULT)
    {
      SString string;
      // must do this even if STORE_RESULT==false to increment buffer_offset
      ReadFromBuffer<SString>( buffer, buffer_offset, string );

      if( STORE_RESULT ){
        flat_container->name.push_back( std::make_pair( std::move(tree_node), std::move(string) ) );
      }
    };
  }
  else if( type.isBuiltin())
  {
    deserializeAndStore = [&](StringTreeLeaf tree_node, bool STORE_RESULT)
    {
      // must do this even if STORE_RESULT==false to increment buffer_offset
      auto value = type.deserializeFromBuffer(buffer, buffer_offset);

      if( STORE_RESULT ){
        flat_container->value.push_back( std::make_pair( std::move(tree_node), std::move(value) ) );
      }
    };
  }
  else if( type.typeID() == OTHER)
  {
    const ROSMessage* mg_definition = nullptr;

    for(const ROSMessage& msg: type_list) // find in the list
    {
      if( msg.type() == type )
      {
        mg_definition = &msg;
        break;
      }
    }
    if( !mg_definition )
    {
      std::string output( "can't deserialize this stuff: ");
      output +=  type.baseName().toStdString() + "\n\n";
      output +=  "Available types are: \n\n";
      for(const ROSMessage& msg: type_list) // find in the list
      {
        output += "   " +msg.type().baseName().toStdString() + "\n";
      }
      throw std::runtime_error( output );
    }

    deserializeAndStore = [&](StringTreeLeaf tree_node, bool STORE_RESULT)
    {
      if( STORE_RESULT == false)
      {
        // must do this even if STORE_RESULT==false to increment buffer_offset
        for (const ROSField& field : mg_definition->fields() )
        {
          if(field.isConstant() == false)
          {
            buildRosFlatTypeImpl(type_list, field.type(),
                                 (tree_node),
                                 buffer, buffer_offset,
                                 flat_container,
                                 max_array_size, false);
          }
        }
      }
      else{
        auto& children_nodes = tree_node.node_ptr->children();

        const bool add_child_nodes_to_tree = children_nodes.empty();
        if( add_child_nodes_to_tree )
        {
          // note: should use reserve here, NOT resize
          children_nodes.reserve( mg_definition->fields().size() );
        }

        size_t index = 0;

        for (const ROSField& field : mg_definition->fields() )
        {
          if(field.isConstant() == false) {

            if( add_child_nodes_to_tree ){
              SString node_name( field.name() )  ;
              tree_node.node_ptr->addChild( node_name );
            }
            auto new_tree_node = tree_node;
            new_tree_node.node_ptr = &children_nodes[index++];

            buildRosFlatTypeImpl(type_list, field.type(),
                                 (new_tree_node),
                                 buffer, buffer_offset,
                                 flat_container,
                                 max_array_size, true);

          } //end of field.isConstant()
        } // end of for
      } // end of STORE_RESULTS == true
    };//end of lambda
  }
  else {
    throw std::runtime_error( "can't deserialize this stuff");
  }

  //---------------------------------------------------------------------------

  const bool STORE = ( do_store ) && ( array_size <= max_array_size );

  if( array_size > max_array_size && do_store)
  {
    (*_global_warnings_stream_) << "Warning: skipped a vector of type "
                                << type.baseName() << " and size "
                                << array_size << " because max_array_size = "
                                << max_array_size << "\n";
  }

  StringTreeNode* node = tree_leaf.node_ptr;

  if( type.isArray() == false  )
  {
    deserializeAndStore( tree_leaf, STORE );
  }
  else
  {
    if(STORE)
    {
      node->children().reserve(1);
      tree_leaf.node_ptr = node->addChild( "#" );

      tree_leaf.array_size++;

      for (int v=0; v<array_size; v++)
      {
        tree_leaf.index_array[ tree_leaf.array_size-1 ] = static_cast<uint16_t>(v);
        deserializeAndStore( tree_leaf, STORE );
      }
    }
    else{
      for (int v=0; v<array_size; v++)
      {
        deserializeAndStore( tree_leaf, STORE );
      }
    }
  }
}

void BuildRosFlatType(const ROSTypeList& type_list,
                      ROSType type,
                      SString prefix,
                      const nonstd::VectorView<uint8_t>& buffer,
                      ROSTypeFlat* flat_container_output,
                      const uint32_t max_array_size )
{

  flat_container_output->tree.root()->children().clear();
  flat_container_output->tree.root()->value() = prefix;
  flat_container_output->name.clear();
  flat_container_output->value.clear();

  StringTreeLeaf rootnode;
  rootnode.node_ptr = flat_container_output->tree.root();

  size_t offset = 0;

  buildRosFlatTypeImpl( type_list, type,
                        rootnode,
                        buffer, offset,
                        flat_container_output,
                        max_array_size, true);
  if( offset != buffer.size() )
  {
    throw std::runtime_error("buildRosFlatType: There was an error parsing the buffer" );
  }
}

StringTreeLeaf::StringTreeLeaf(): node_ptr(nullptr), array_size(0)
{  for (int i=0; i<7; i++) index_array[i] = 0;}


bool StringTreeLeaf::toStr(SString& destination) const
{
  char buffer[1024];
  int offset = this->toStr(buffer);

  if( offset < 0 ) {
    destination.clear();
    return false;
  }
  destination.assign(buffer, offset);
  return true;
}

bool StringTreeLeaf::toStr(std::string& destination) const
{
  char buffer[512];
  int offset = this->toStr(buffer);

  if( offset < 0 ) {
    destination.clear();
    return false;
  }
  destination.assign(buffer, offset);
  return true;
}

int StringTreeLeaf::toStr(char* buffer) const
{

  const StringTreeNode* leaf_node = this->node_ptr;
  if( !leaf_node ){
    return -1;
  }

  const SString* strings_from_leaf_to_root[64];
  int index = 0;

  int char_count = 0;

  while(leaf_node)
  {
    const bool is_root = leaf_node->parent() == nullptr;
    const SString& value = is_root ? tree_prefix : leaf_node->value();

    char_count += value.size();
    strings_from_leaf_to_root[index] = &value;
    index++;
    leaf_node = leaf_node->parent();
  };

  strings_from_leaf_to_root[index] = nullptr;
  index--;

  int array_count = 0;
  int off = 0;

  while ( index >=0 )
  {
    const SString& value =  strings_from_leaf_to_root[index];
    if( value.size()== 1 && value.at(0) == '#' )
    {
      buffer[off-1] = '.';
      off += print_number(&buffer[off], this->index_array[ array_count++ ] );
    }
    else{
      memcpy( &buffer[off], value.data(), value.size() );
      off += value.size();
    }
    if( index > 0 ){
      buffer[off] = '/';
      off += 1;
    }
    index--;
  }
  buffer[off] = '\0';
  return off;
}


} // end namespace
