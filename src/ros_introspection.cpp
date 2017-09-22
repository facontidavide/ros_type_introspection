#include "ros_type_introspection/ros_introspection.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/utility/string_ref.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <functional>

namespace RosIntrospection {

void Parser::registerMessageDefinition(const std::string &type_name, const std::string &definition)
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
    ROSMessage msg( split[i] );
    if( i == 0)
    {
      msg.mutateType( ROSType(type_name) );
    }

    info.type_list.push_back( std::move(msg) );
    all_types.push_back( &(info.type_list.back().type()) );
  }

  for( ROSMessage& msg: info.type_list )
  {
    msg.updateMissingPkgNames( all_types );
  }
  //------------------------------

  std::function<void(const ROSType& type, StringTreeNode* node_ptr)> recursiveTreeCreator;

  recursiveTreeCreator = [&](const ROSType& type, StringTreeNode* node_ptr)
  {
    if( type.typeID() == OTHER)
    {
      const ROSMessage* mg_definition = nullptr;

      for(const ROSMessage& msg: info.type_list) // find in the list
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
        for(const ROSMessage& msg: info.type_list) // find in the list
        {
          output += "   " +msg.type().baseName().toStdString() + "\n";
        }
        throw std::runtime_error( output );
      }

      auto& children_nodes = node_ptr->children();

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
            node_ptr->addChild( field.name() );
          }
          StringTreeNode* new_node_ptr = &( children_nodes[index++] );

          if( field.type().isArray())
          {
            new_node_ptr->children().reserve(1);
            new_node_ptr = new_node_ptr->addChild("#");
          }

          recursiveTreeCreator(field.type(), new_node_ptr);

        } //end of field.isConstant()
      } // end of for fields
    } // end OTHER

  };//end of lambda

  info.tree.root()->value() = type_name;
  // start recursion
  recursiveTreeCreator( info.type_list.front().type(), info.tree.root() );

  std::cout << info.tree << std::endl;

  _registred_messages.insert( std::make_pair(type_name, std::move(info) ) );
}

const ROSMessageInfo *Parser::getMessageInfo(const std::string &type_name)
{
  auto it = _registred_messages.find(type_name);
  if( it != _registred_messages.end() )
  {
    return &(it->second);
  }
  return nullptr;
}

void Parser::deserializeIntoFlatContainer(const ROSType& type,
                                          const SString& prefix,
                                          const nonstd::VectorView<uint8_t> &buffer,
                                          ROSTypeFlat *flat_container_output,
                                          const uint32_t max_array_size)
{
//    tree->root()->value() = prefix;
//    flat_container_output->name.clear();
//    flat_container_output->value.clear();

//    StringTreeLeaf rootnode;
//    rootnode.node_ptr = flat_container_output->tree.root();

//    size_t offset = 0;

//    buildRosFlatTypeImpl( type_list, type,
//                          rootnode,
//                          buffer, offset,
//                          flat_container_output,
//                          max_array_size, true);
//    if( offset != buffer.size() )
//    {
//      throw std::runtime_error("buildRosFlatType: There was an error parsing the buffer" );
//    }
//  }
}

}


