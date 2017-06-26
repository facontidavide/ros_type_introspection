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

#include <ros_type_introspection/deserializer.hpp>
#include <functional>

namespace RosIntrospection{


inline void SkipBytesInBuffer( uint8_t** buffer, int vector_size, const BuiltinType& type )
{
  if( type == STRING)
  {
    for (int i=0; i<vector_size; i++){
      int32_t string_size = ReadFromBuffer<int32_t>( buffer );
      *buffer += string_size;
    }
  }
  else{
    *buffer += vector_size * BuiltinTypeSize[ static_cast<int>(type) ];
  }
}


void buildRosFlatTypeImpl(const ROSTypeList& type_list,
                          const ROSType &type,
                          StringTreeLeaf tree_node, // easier to use copy instead of reference or pointer
                          uint8_t** buffer_ptr,
                          ROSTypeFlat* flat_container,
                          uint16_t max_array_size )
{
  int array_size = type.arraySize();
  if( array_size == -1)
  {
    array_size = ReadFromBuffer<int32_t>( buffer_ptr );
  }

 // std::cout << type.msgName() << " type: " <<  type.typeID() << " size: " << array_size << std::endl;

  std::function<void(StringTreeLeaf)> deserializeAndStore;

  if( type.typeID() == STRING )
  {
      deserializeAndStore = [&](StringTreeLeaf tree_node)
      {
        size_t string_size = (size_t) ReadFromBuffer<int32_t>( buffer_ptr );
        SString id( (const char*)(*buffer_ptr), string_size );
        (*buffer_ptr) += string_size;
        flat_container->name.push_back( std::make_pair( std::move(tree_node), id ) );
      };
  }
  else if( type.isBuiltin())
  {
      deserializeAndStore = [&](StringTreeLeaf tree_node){
        flat_container->value.push_back( std::make_pair( std::move(tree_node), type.deserializeFromBuffer(buffer_ptr) ) );
      };
  }
  else if( type.typeID() == OTHER)
  {
    deserializeAndStore = [&](StringTreeLeaf tree_node)
    {
      bool done = false;
      for(const ROSMessage& msg: type_list) // find in the list
      {
        if( msg.type().msgName() == type.msgName() &&
            msg.type().pkgName() == type.pkgName()  )
        {
          auto& children_nodes = tree_node.node_ptr->children();

          bool to_add = false;
          if( children_nodes.empty() )
          {
            children_nodes.reserve( msg.fields().size() );
            to_add = true;
          }

          size_t index = 0;

          for (const ROSField& field : msg.fields() )
          {
              if(field.isConstant() == false) {

              if( to_add){
                 SString node_name( field.name() )  ;
                 tree_node.node_ptr->addChild( node_name );
              }
              auto new_tree_node = tree_node;
              new_tree_node.node_ptr = &children_nodes[index++];

              // note: this is not invalidated only because we reserved space in the vector
              //  tree_node.element_ptr = &children_nodes[index++];

              buildRosFlatTypeImpl(type_list,
                                   field.type(),
                                   (new_tree_node),
                                   buffer_ptr,
                                   flat_container,
                                   max_array_size);
            }
          }
          done = true;
          break;
        }
      }
      if( !done ){
          std::string output( "can't deserialize this stuff: ");
          output +=  type.baseName().toStdString() + "\n\n";
          output +=  "Available types are: \n\n";
          for(const ROSMessage& msg: type_list) // find in the list
          {
            output += "   " +msg.type().baseName().toStdString() + "\n";
          }
        throw std::runtime_error( output );
      }
    };
  }
  else {
      throw std::runtime_error( "can't deserialize this stuff");
  }

  if( array_size < max_array_size )
  {
    StringTreeNode* node = tree_node.node_ptr;

    if( type.isArray()  )
    {
      node->children().reserve(1);
      node->addChild( "#" );
      tree_node.node_ptr = &node->children().back();
      tree_node.array_size++;

      for (int v=0; v<array_size; v++)
      {
        tree_node.index_array[ tree_node.array_size-1 ] = v;
        deserializeAndStore( (tree_node) );
      }
    }
    else{
      deserializeAndStore( (tree_node) );
    }

  }
  else{
    SkipBytesInBuffer( buffer_ptr, array_size, type.typeID() );
  }
}


void buildRosFlatType(const ROSTypeList& type_map,
                             ROSType type,
                             SString prefix,
                             uint8_t *buffer_ptr,
                             ROSTypeFlat* flat_container_output,
                             uint16_t max_array_size)
{
  uint8_t** buffer = &buffer_ptr;

  flat_container_output->tree.root()->children().clear();
  flat_container_output->tree.root()->value() = prefix;
  flat_container_output->name.clear();
  flat_container_output->value.clear();
  flat_container_output->renamed_value.clear();

  StringTreeLeaf rootnode;
  rootnode.node_ptr = flat_container_output->tree.root();

  buildRosFlatTypeImpl( type_map,
                        type,
                        rootnode,
                        buffer,
                        flat_container_output,
                        max_array_size );
}

StringTreeLeaf::StringTreeLeaf(): node_ptr(nullptr), array_size(0)
{  for (int i=0; i<7; i++) index_array[i] = 0;}



// The idea comes from the talk by Alexandrescu
// "Three Optimization Tips for C++".

// much faster for numbers below 100
inline int print_number(char* buffer, uint16_t value)
{
    const char DIGITS[] =
            "00010203040506070809"
            "10111213141516171819"
            "20212223242526272829"
            "30313233343536373839"
            "40414243444546474849"
            "50515253545556575859"
            "60616263646566676869"
            "70717273747576777879"
            "80818283848586878889"
            "90919293949596979899";
    if (value < 10)
    {
        buffer[0] = static_cast<char>('0' + value);
        return 1;
    }
    else if (value < 100) {
        value *= 2;
        buffer[0] = DIGITS[ value+1 ];
        buffer[1] = DIGITS[ value ];
        return 2;
    }
    else{
        return sprintf( buffer,"%d", value );
    }
}

void StringTreeLeaf::toStr(SString& destination) const
{

  const StringTreeNode* leaf_node = this->node_ptr;

  if( !leaf_node ) return destination.clear();

  const StringTreeNode* nodes_from_leaf_to_root[64];
  int index = 0;

  int char_count = 0;

  while(leaf_node)
  {
    char_count += leaf_node->value().size();
    nodes_from_leaf_to_root[index] = leaf_node;
    index++;
    leaf_node = leaf_node->parent();
  };

  nodes_from_leaf_to_root[index] = nullptr;
  index--;

  int array_count = 0;

  char buffer[256];
  int off = 0;

  while ( index >=0 )
  {
    const SString& value =  nodes_from_leaf_to_root[index]->value();
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
  destination.assign(buffer, off);
}


} // end namespace
