#include <ros_type_introspection/deserializer.hpp>
#include <functional>

namespace RosIntrospection{


template <typename T> T ReadFromBuffer( uint8_t** buffer)
{
  T destination =  (*( reinterpret_cast<T*>( *buffer ) ) );
  *buffer +=  sizeof(T);
  return destination;
}

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

  switch( type.typeID())
  {
  case STRING: {
    deserializeAndStore = [&](StringTreeLeaf tree_node)
    {
      size_t string_size = (size_t) ReadFromBuffer<int32_t>( buffer_ptr );
      SString id( (const char*)(*buffer_ptr), string_size );
      (*buffer_ptr) += string_size;
      flat_container->name.push_back( std::make_pair( std::move(tree_node), id ) );
    };
  }break;

  case FLOAT64: {
    deserializeAndStore = [&](StringTreeLeaf tree_node){
      flat_container->value.push_back( std::make_pair( std::move(tree_node), (double) ReadFromBuffer<double>(buffer_ptr) ) );
    };
  }break;
  case FLOAT32: {
    deserializeAndStore = [&](StringTreeLeaf tree_node){
      flat_container->value.push_back( std::make_pair( std::move(tree_node), (double) ReadFromBuffer<float>(buffer_ptr) ) );
    };
  }break;
  case TIME: {
    deserializeAndStore = [&](StringTreeLeaf tree_node){
      double sec  = (double) ReadFromBuffer<uint32_t>(buffer_ptr);
      double nsec = (double) ReadFromBuffer<uint32_t>(buffer_ptr);
      flat_container->value.push_back( std::make_pair( std::move(tree_node), (double)( sec + nsec/(1000*1000*1000) ) ) );
    };
  }break;
  case UINT64: {
    deserializeAndStore = [&](StringTreeLeaf tree_node){
      flat_container->value.push_back( std::make_pair( std::move(tree_node), (double) ReadFromBuffer<uint64_t>(buffer_ptr) ) );
    };
  }break;
  case INT64: {
    deserializeAndStore = [&](StringTreeLeaf tree_node){
      flat_container->value.push_back( std::make_pair( std::move(tree_node), (double) ReadFromBuffer<int64_t>(buffer_ptr) ) );
    };
  }break;
  case UINT32: {
    deserializeAndStore = [&](StringTreeLeaf tree_node){
      flat_container->value.push_back( std::make_pair( std::move(tree_node), (double) ReadFromBuffer<uint32_t>(buffer_ptr) ) );
    };
  }break;
  case INT32: {
    deserializeAndStore = [&](StringTreeLeaf tree_node){
      flat_container->value.push_back( std::make_pair( std::move(tree_node), (double) ReadFromBuffer<int32_t>(buffer_ptr) ) );
    };
  }break;
  case UINT16: {
    deserializeAndStore = [&](StringTreeLeaf tree_node){
      flat_container->value.push_back( std::make_pair( std::move(tree_node), (double) ReadFromBuffer<uint16_t>(buffer_ptr) ) );
    };
  }break;
  case INT16: {
    deserializeAndStore = [&](StringTreeLeaf tree_node){
      flat_container->value.push_back( std::make_pair( std::move(tree_node), (double) ReadFromBuffer<int16_t>(buffer_ptr) ) );
    };
  }break;
  case BOOL:
  case UINT8: {
    deserializeAndStore = [&](StringTreeLeaf tree_node){
      flat_container->value.push_back( std::make_pair( std::move(tree_node), (double) ReadFromBuffer<uint8_t>(buffer_ptr) ) );
    };
  }break;
  case BYTE:
  case INT8: {
    deserializeAndStore = [&](StringTreeLeaf tree_node){
      flat_container->value.push_back( std::make_pair( std::move(tree_node), (double) ReadFromBuffer<int8_t>(buffer_ptr) ) );
    };
  }break;

  case DURATION: {
    deserializeAndStore = [&](StringTreeLeaf tree_node){
      double sec  = (double) ReadFromBuffer<int32_t>(buffer_ptr);
      double nsec = (double) ReadFromBuffer<int32_t>(buffer_ptr);
      flat_container->value.push_back( std::make_pair( std::move(tree_node), (double)( sec + nsec/(1000*1000*1000) ) ) );
    };
  }break;

  case OTHER:{
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
                 SString node_name( field.name().data(), field.name().size() )  ;
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
        throw std::runtime_error( "can't deserialize this stuff: " + type.baseName() );
      }
    };
  }break;

  default: throw std::runtime_error( "can't deserialize this stuff"); break;
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

SString StringTreeLeaf::toStr() const{


  const StringTreeNode* node = this->node_ptr;

  if( !node ) return SString();

  const StringTreeNode* array[64];
  int index = 0;

  int char_count = 0;

  while(node )
  {
    char_count += node->value().size();
    array[index] = node;
    index++;
    node = node->parent();
  };

  array[index] = nullptr;
  index--;

  int array_count = 0;

  char buffer[200];
  int off = 0;

  while ( index >=0)
  {
    const SString& value =  array[index]->value();
    if( value.size()== 1 && value.at(0) == '#' )
    {
     // char buffer[10];
      //sprintf(buffer, "%d", this->index_array[ array_count++ ] );
      off += sprintf( &buffer[off],"%d", this->index_array[ array_count++ ] );
    //  output.append( buffer );
    }
    else{
      off += sprintf( &buffer[off],"%s", array[index]->value().data() );
    //  output.append( array[index]->value() );
    }
    if( index > 0 )  off += sprintf( &buffer[off],"."); //output.append(".");
    index--;
  }
  return SString(buffer);
}


} // end namespace
