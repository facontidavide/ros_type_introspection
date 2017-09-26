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
  std::function<void(const ROSMessage*, StringTreeNode*, MessageTreeNode* )> recursiveTreeCreator;

  recursiveTreeCreator = [&](const ROSMessage* msg_definition, StringTreeNode* string_node, MessageTreeNode* msg_node)
  {
    // note: should use reserve here, NOT resize
    string_node->children().reserve( msg_definition->fields().size() );
    msg_node->children().reserve( msg_definition->fields().size() );

    for (const ROSField& field : msg_definition->fields() )
    {
      if(field.isConstant() == false) {

        // Let's add first a child to string_node
        string_node->addChild( field.name() );
        StringTreeNode*  new_string_node = &(string_node->children().back());
        if( field.type().isArray())
        {
          new_string_node->children().reserve(1);
          new_string_node = new_string_node->addChild("#");
        }

        const ROSMessage* next_msg = nullptr;
        // builtin types will not trigger a recursion
        if( field.type().isBuiltin() == false)
        {
          next_msg = getMessageByType( field.type(), info );
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

const ROSMessage* Parser::getMessageByType(const ROSType &type, const ROSMessageInfo& info)
{
  for(const ROSMessage& msg: info.type_list) // find in the list
  {
    if( msg.type().msgName() == type.msgName() &&
        msg.type().pkgName() == type.pkgName() )
    {
      return &msg;
    }
  }
  return nullptr;
}

void Parser::applyVisitorToBuffer(const std::string &msg_identifier, const ROSType& monitored_type,
                                  nonstd::VectorViewMutable<uint8_t> &buffer, Parser::VisitingCallback callback)
{
  const ROSMessageInfo* msg_info = getMessageInfo(msg_identifier);

  if( msg_info == nullptr)
  {
    throw std::runtime_error("deserializeIntoFlatContainer: msg_identifier not registerd. Use registerMessageDefinition" );
  }

  std::function<void(const MessageTreeNode*)> recursiveImpl;
  size_t buffer_offset = 0;

  recursiveImpl = [&](const MessageTreeNode* msg_node)
  {
    const ROSMessage* msg_definition = msg_node->value();
    const ROSType& msg_type = msg_definition->type();

    const bool matching =( msg_type.msgName() == monitored_type.msgName() &&
                           msg_type.pkgName() == monitored_type.pkgName() );

    uint8_t* prev_buffer_ptr = buffer.data() + buffer_offset;
    size_t prev_offset = buffer_offset;

    size_t index_m = 0;

    for (const ROSField& field : msg_definition->fields() )
    {
      if(field.isConstant() ) continue;

      const ROSType&  field_type = field.type();

      int32_t array_size = field_type.arraySize();
      if( array_size == -1)
      {
        ReadFromBuffer( buffer, buffer_offset, array_size );
      }

      //------------------------------------

      if( field_type.isBuiltin() )
      {
        for (int i=0; i<array_size; i++ )
        {
          ReadFromBuffer( field_type.typeID(), buffer, buffer_offset );
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
      nonstd::VectorViewMutable<uint8_t> view( prev_buffer_ptr, buffer_offset - prev_offset);
      callback( monitored_type, view );
    }
  }; //end lambda

  //start recursion
  recursiveImpl( msg_info->message_tree.croot() );
}

void Parser::deserializeIntoFlatContainer(const std::string& msg_identifier,
                                          const nonstd::VectorView<uint8_t>& buffer,
                                          ROSTypeFlat* flat_container,
                                          const uint32_t max_array_size )
{

  const ROSMessageInfo* msg_info = getMessageInfo(msg_identifier);

  if( msg_info == nullptr)
  {
    throw std::runtime_error("deserializeIntoFlatContainer: msg_identifier not registerd. Use registerMessageDefinition" );
  }
  size_t buffer_offset = 0;

  std::function<void(const MessageTreeNode*,StringTreeLeaf,bool)> deserializeImpl;

  deserializeImpl = [&](
      const MessageTreeNode* msg_node,
      StringTreeLeaf tree_leaf, // copy, not reference
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

        int32_t array_size = field_type.arraySize();
        if( array_size == -1)
        {
          ReadFromBuffer( buffer, buffer_offset, array_size );
        }
        if( field_type.isArray())
        {
          new_tree_leaf.array_size++;
          new_tree_leaf.node_ptr = new_tree_leaf.node_ptr->child(0);
        }

        if( array_size > max_array_size ) DO_STORE = false;

        //------------------------------------
        for (int i=0; i<array_size; i++ )
        {
          if( field_type.isArray() )
          {
            new_tree_leaf.index_array[ new_tree_leaf.array_size-1 ] = i;
          }

          if( field_type.typeID() == STRING )
          {
            SString string;
            ReadFromBuffer<SString>( buffer, buffer_offset, string );

            if( DO_STORE ){
              flat_container->name.push_back( std::make_pair( new_tree_leaf, std::move(string) ) );
            }
          }
          else if( field_type.isBuiltin() )
          {
            Variant var = ReadFromBuffer( field_type.typeID(), buffer, buffer_offset );

            if( DO_STORE ){
              flat_container->value.push_back( std::make_pair( new_tree_leaf, std::move(var) ) );
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
  flat_container->name.clear();
  flat_container->value.clear();

  StringTreeLeaf rootnode;
  rootnode.node_ptr = msg_info->string_tree.croot();

  deserializeImpl( msg_info->message_tree.croot(),
                   rootnode,
                   true);

  if( buffer_offset != buffer.size() )
  {
    throw std::runtime_error("buildRosFlatType: There was an error parsing the buffer" );
  }

}

}


