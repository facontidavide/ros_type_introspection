#include "ros_type_introspection/ros_introspection.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/utility/string_ref.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <functional>

namespace RosIntrospection {

void Parser::createTrees(ROSMessageInfo& info, const std::string &type_name)
{
  std::function<void(const ROSType&, StringTreeNode*, TypeTreeNode* )> recursiveTreeCreator;

  recursiveTreeCreator = [&](const ROSType& type, StringTreeNode* string_node, TypeTreeNode* type_node)
  {
    if( type.typeID() == OTHER)
    {
      const ROSMessageDefinition* mg_definition = nullptr;

      for(const ROSMessageDefinition& msg: info.type_list) // find in the list
      {
        if( msg.type().msgName() == type.msgName() &&
            msg.type().pkgName() == type.pkgName() )
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
        for(const ROSMessageDefinition& msg: info.type_list) // find in the list
        {
          output += "   " +msg.type().baseName().toStdString() + "\n";
        }
        throw std::runtime_error( output );
      }

      // note: should use reserve here, NOT resize
      string_node->children().reserve( mg_definition->fields().size() );
      type_node->children().reserve( mg_definition->fields().size() );

      size_t index = 0;

      for (const ROSField& field : mg_definition->fields() )
      {
        if(field.isConstant() == false) {

          string_node->addChild( field.name() );
          type_node->addChild( &field.type() );

          StringTreeNode* new_string_node = string_node->child(index);
          TypeTreeNode*   new_type_node = type_node->child(index);
          index++;

          if( field.type().isArray())
          {
            new_string_node->children().reserve(1);
            new_string_node = new_string_node->addChild("#");
          }

          recursiveTreeCreator(field.type(), new_string_node, new_type_node);

        } //end of field.isConstant()
      } // end of for fields
    } // end OTHER

  };//end of lambda

  info.string_tree.root()->value() = type_name;
 //TODO info.type_tree.root()->value() =
  // start recursion
  recursiveTreeCreator( info.type_list.front().type(),
                        info.string_tree.root(),
                        info.type_tree.root());
}



void Parser::registerMessageDefinition(const std::string &message_identifier,
                                       const ROSType &main_type,
                                       const std::string &definition)
{
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
    ROSMessageDefinition msg( split[i] );
    if( i == 0)
    {
      msg.mutateType( main_type );
    }

    info.type_list.push_back( std::move(msg) );
    all_types.push_back( &(info.type_list.back().type()) );
  }

  for( ROSMessageDefinition& msg: info.type_list )
  {
    msg.updateMissingPkgNames( all_types );
  }
  //------------------------------

  createTrees(info, message_identifier);

  std::cout << info.string_tree << std::endl;

  _registred_messages.insert( std::make_pair(message_identifier, std::move(info) ) );
}

const ROSMessageInfo *Parser::getMessageInfo(const std::string &msg_identifier)
{
  auto it = _registred_messages.find(msg_identifier);
  if( it != _registred_messages.end() )
  {
    return &(it->second);
  }
  return nullptr;
}


void Parser::deserializeImpl(const ROSMessageInfo & info,
                             const ROSType* type,
                             StringTreeLeaf tree_leaf, // copy, not reference
                             const nonstd::VectorView<uint8_t>& buffer,
                             size_t& buffer_offset,
                             ROSTypeFlat* flat_container,
                             const uint32_t max_array_size,
                             bool do_store)
{

  int32_t array_size = type->arraySize();
  if( array_size == -1)
  {
    ReadFromBuffer( buffer, buffer_offset, array_size );
  }

  //---------------------------------------------------------------------------
  // we store in a function pointer the operation to be done later
  // This operation is different according to the typeID
  std::function<void(StringTreeLeaf, bool)> deserializeAndStore;

  if( type->typeID() == STRING )
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
  else if( type->isBuiltin())
  {
    deserializeAndStore = [&](StringTreeLeaf tree_node, bool STORE_RESULT)
    {
      // must do this even if STORE_RESULT==false to increment buffer_offset
      auto value = type->deserializeFromBuffer(buffer, buffer_offset);

      if( STORE_RESULT ){
        flat_container->value.push_back( std::make_pair( std::move(tree_node), std::move(value) ) );
      }
    };
  }
  else if( type->typeID() == OTHER)
  {
    const ROSMessageDefinition* mg_definition = nullptr;

    //TODO
//    for(const ROSMessageDefinition& msg: info.type_list) // find in the list
//    {
//      if( msg.type() == type )
//      {
//        mg_definition = &msg;
//        break;
//      }
//    }
    if( !mg_definition )
    {
      std::string output( "can't deserialize this stuff: ");
      output +=  type->baseName().toStdString() + "\n\n";
      output +=  "Available types are: \n\n";
      for(const ROSMessageDefinition& msg: info.type_list) // find in the list
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
            deserializeImpl(info, field.type(),
                            (tree_node),
                            buffer, buffer_offset,
                            flat_container,
                            max_array_size, false);
          }
        }
      }
      else{
        size_t index = 0;

        for (const ROSField& field : mg_definition->fields() )
        {
          if(field.isConstant() == false)
          {
            auto new_tree_node = tree_node;
            new_tree_node.node_ptr = tree_node.node_ptr->child(index++);

            deserializeImpl(info, field.type(),
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
    (*_global_warnings) << "Warning: skipped a vector of type "
                        << type->baseName() << " and size "
                        << array_size << " because max_array_size = "
                        << max_array_size << "\n";
  }

  const StringTreeNode* node = tree_leaf.node_ptr;

  if(STORE)
  {
    tree_leaf.node_ptr = node->child(0);
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


void Parser::deserializeIntoFlatContainer(const std::string& msg_identifier,
                                          const nonstd::VectorView<uint8_t>& buffer,
                                          ROSTypeFlat* flat_container_output,
                                          const uint32_t max_array_size )
{
  const ROSMessageInfo* msg_info = getMessageInfo(msg_identifier);

  if( msg_info == nullptr)
  {
    throw std::runtime_error("deserializeIntoFlatContainer: msg_identifier not registerd. Use registerMessageDefinition" );
  }

  flat_container_output->name.clear();
  flat_container_output->value.clear();

  StringTreeLeaf rootnode;
  rootnode.node_ptr = msg_info->string_tree.croot();

  size_t offset = 0;

  deserializeImpl( msg_info,
                   *(msg_info->type_tree.croot()),
                   rootnode,
                   buffer, offset,
                   flat_container_output,
                   max_array_size, true);

  if( offset != buffer.size() )
  {
    throw std::runtime_error("buildRosFlatType: There was an error parsing the buffer" );
  }

}

}


