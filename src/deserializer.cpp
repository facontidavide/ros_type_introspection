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
                          StringTreeLeaf leaf, // easier to use copy instead of reference or pointer
                          uint8_t** buffer_ptr,
                          ROSTypeFlat* flat_container,
                          uint8_t max_array_size )
{
    //std::cout << flat_container->tree << std::endl;

    int array_size = type.arraySize();
    if( array_size == -1)
    {
        array_size = ReadFromBuffer<int32_t>( buffer_ptr );
    }

    std::function<void(StringTreeLeaf)> deserializeAndStore;

    switch( type.typeID())
    {
        case STRING: {
            deserializeAndStore = [&](StringTreeLeaf leaf)
            {
                size_t string_size = (size_t) ReadFromBuffer<int32_t>( buffer_ptr );
                ShortString id( (const char*)(*buffer_ptr), string_size );
                (*buffer_ptr) += string_size;
                flat_container->name_id.push_back( std::make_pair( leaf, id ) );
            };
        }break;

        case FLOAT64: {
            deserializeAndStore = [&](StringTreeLeaf leaf){
                flat_container->value.push_back( std::make_pair( leaf, (double) ReadFromBuffer<double>(buffer_ptr) ) );
            };
        }break;
        case FLOAT32: {
            deserializeAndStore = [&](StringTreeLeaf leaf){
                flat_container->value.push_back( std::make_pair( leaf, (double) ReadFromBuffer<float>(buffer_ptr) ) );
            };
        }break;
        case TIME: {
            deserializeAndStore = [&](StringTreeLeaf leaf){
                double sec  = (double) ReadFromBuffer<uint32_t>(buffer_ptr);
                double nsec = (double) ReadFromBuffer<uint32_t>(buffer_ptr);
                flat_container->value.push_back( std::make_pair( leaf, (double)( sec + nsec/(1000*1000*1000) ) ) );
            };
        }break;
        case UINT64: {
            deserializeAndStore = [&](StringTreeLeaf leaf){
                flat_container->value.push_back( std::make_pair( leaf, (double) ReadFromBuffer<uint64_t>(buffer_ptr) ) );
            };
        }break;
        case INT64: {
            deserializeAndStore = [&](StringTreeLeaf leaf){
                flat_container->value.push_back( std::make_pair( leaf, (double) ReadFromBuffer<int64_t>(buffer_ptr) ) );
            };
        }break;
        case UINT32: {
            deserializeAndStore = [&](StringTreeLeaf leaf){
                flat_container->value.push_back( std::make_pair( leaf, (double) ReadFromBuffer<uint32_t>(buffer_ptr) ) );
            };
        }break;
        case INT32: {
            deserializeAndStore = [&](StringTreeLeaf leaf){
                flat_container->value.push_back( std::make_pair( leaf, (double) ReadFromBuffer<int32_t>(buffer_ptr) ) );
            };
        }break;
        case UINT16: {
            deserializeAndStore = [&](StringTreeLeaf leaf){
                flat_container->value.push_back( std::make_pair( leaf, (double) ReadFromBuffer<uint16_t>(buffer_ptr) ) );
            };
        }break;
        case INT16: {
            deserializeAndStore = [&](StringTreeLeaf leaf){
                flat_container->value.push_back( std::make_pair( leaf, (double) ReadFromBuffer<int16_t>(buffer_ptr) ) );
            };
        }break;
        case BOOL:
        case UINT8: {
            deserializeAndStore = [&](StringTreeLeaf leaf){
                flat_container->value.push_back( std::make_pair( leaf, (double) ReadFromBuffer<uint8_t>(buffer_ptr) ) );
            };
        }break;
        case BYTE:
        case INT8: {
            deserializeAndStore = [&](StringTreeLeaf leaf){
                flat_container->value.push_back( std::make_pair( leaf, (double) ReadFromBuffer<int8_t>(buffer_ptr) ) );
            };
        }break;

        case DURATION: {
            deserializeAndStore = [&](StringTreeLeaf leaf){
                double sec  = (double) ReadFromBuffer<int32_t>(buffer_ptr);
                double nsec = (double) ReadFromBuffer<int32_t>(buffer_ptr);
                flat_container->value.push_back( std::make_pair( leaf, (double)( sec + nsec/(1000*1000*1000) ) ) );
            };
        }break;

        case OTHER:{
            deserializeAndStore = [&](StringTreeLeaf leaf)
            {
                bool done = false;
                for(const ROSMessage& msg: type_list) // find in the list
                {
                    if( msg.type.msgName() == type.msgName() &&
                            msg.type.pkgName() == type.pkgName()  )
                    {
                        auto& children_nodes = leaf.element_ptr->children();

                        bool to_add = false;
                        if( children_nodes.empty() )
                        {
                            children_nodes.reserve( msg.fields.size() );
                            to_add = true;
                        }

                      //  size_t index = 0;
                        for (const ROSField& field: msg.fields )
                        {
                            if(field.isConstant() == false) {

                                if( to_add){
                                    ShortString node_name( field.name().data(), field.name().size() )  ;
                                    leaf.element_ptr->addChild( node_name );
                                }
                                auto new_leaf = leaf;
                                new_leaf.element_ptr = &children_nodes.back();

                                // note: this is not invalidated only because we reserved space in the vector
                              //  leaf.element_ptr = &children_nodes[index++];

                                buildRosFlatTypeImpl(type_list,
                                                     field.type(),
                                                     new_leaf,
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
        StringElement* node = leaf.element_ptr;

        if( type.isArray()  )
        {
            node->addChild( "#" );
            leaf.element_ptr = &node->children().back();
            leaf.array_size++;

            for (int v=0; v<array_size; v++)
            {
                leaf.index_array[ leaf.array_size-1 ] = v;
                deserializeAndStore( leaf );
            }
        }
        else{
            deserializeAndStore( leaf );
        }

    }
    else{
        SkipBytesInBuffer( buffer_ptr, array_size, type.typeID() );
    }
}


ROSTypeFlat buildRosFlatType(const ROSTypeList& type_map,
                             ROSType type,
                             const ShortString& prefix,
                             uint8_t** buffer_ptr,
                             uint8_t max_array_size)
{
    ROSTypeFlat flat_container;

    flat_container.tree.root()->value() = prefix;

    StringTreeLeaf root;
    root.element_ptr = flat_container.tree.root();
    root.array_size = 0;

    buildRosFlatTypeImpl( type_map,
                          type,
                          root,
                          buffer_ptr,
                          &flat_container,
                          max_array_size );

  /*  std::sort( flat_container.name_id.begin(),  flat_container.name_id.end(),
               []( const std::pair<ShortString,ShortString> & left,
               const std::pair<ShortString,ShortString> & right)
    {
        return left.first < right.first;
    }
    );*/

   // std::cout << flat_container.tree << std::endl;

    return flat_container;
}


} // end namespace
