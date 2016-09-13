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
                          StringElement* node,
                          uint8_t** buffer_ptr,
                          ROSTypeFlat* flat_container,
                          uint8_t max_array_size )
{
    int array_size = type.arraySize();
    if( array_size == -1)
    {
        array_size = ReadFromBuffer<int32_t>( buffer_ptr );
    }

    std::function<void(StringElement*)> deserializeAndStore;

    switch( type.typeID())
    {
        case STRING: {
            deserializeAndStore = [&](StringElement* node)
            {
                int32_t string_size = ReadFromBuffer<int32_t>( buffer_ptr );
                ShortString id( (const char*)(*buffer_ptr), string_size );
                (*buffer_ptr) += string_size;
                flat_container->name_id.push_back( std::make_pair( node, id ) );
            };
        }break;

        case FLOAT64: {
            deserializeAndStore = [&](StringElement* node){
                flat_container->value.push_back( std::make_pair( node, (double) ReadFromBuffer<double>(buffer_ptr) ) );
            };
        }break;
        case FLOAT32: {
            deserializeAndStore = [&](StringElement* node){
                flat_container->value.push_back( std::make_pair( node, (double) ReadFromBuffer<float>(buffer_ptr) ) );
            };
        }break;
        case TIME: {
            deserializeAndStore = [&](StringElement* node){
                double sec  = (double) ReadFromBuffer<uint32_t>(buffer_ptr);
                double nsec = (double) ReadFromBuffer<uint32_t>(buffer_ptr);
                flat_container->value.push_back( std::make_pair( node, (double)( sec + nsec/(1000*1000*1000) ) ) );
            };
        }break;
        case UINT64: {
            deserializeAndStore = [&](StringElement* node){
                flat_container->value.push_back( std::make_pair( node, (double) ReadFromBuffer<uint64_t>(buffer_ptr) ) );
            };
        }break;
        case INT64: {
            deserializeAndStore = [&](StringElement* node){
                flat_container->value.push_back( std::make_pair( node, (double) ReadFromBuffer<int64_t>(buffer_ptr) ) );
            };
        }break;
        case UINT32: {
            deserializeAndStore = [&](StringElement* node){
                flat_container->value.push_back( std::make_pair( node, (double) ReadFromBuffer<uint32_t>(buffer_ptr) ) );
            };
        }break;
        case INT32: {
            deserializeAndStore = [&](StringElement* node){
                flat_container->value.push_back( std::make_pair( node, (double) ReadFromBuffer<int32_t>(buffer_ptr) ) );
            };
        }break;
        case UINT16: {
            deserializeAndStore = [&](StringElement* node){
                flat_container->value.push_back( std::make_pair( node, (double) ReadFromBuffer<uint16_t>(buffer_ptr) ) );
            };
        }break;
        case INT16: {
            deserializeAndStore = [&](StringElement* node){
                flat_container->value.push_back( std::make_pair( node, (double) ReadFromBuffer<int16_t>(buffer_ptr) ) );
            };
        }break;
        case BOOL:
        case UINT8: {
            deserializeAndStore = [&](StringElement* node){
                flat_container->value.push_back( std::make_pair( node, (double) ReadFromBuffer<uint8_t>(buffer_ptr) ) );
            };
        }break;
        case BYTE:
        case INT8: {
            deserializeAndStore = [&](StringElement* node){
                flat_container->value.push_back( std::make_pair( node, (double) ReadFromBuffer<int8_t>(buffer_ptr) ) );
            };
        }break;

        case DURATION: {
            deserializeAndStore = [&](StringElement* node){
                double sec  = (double) ReadFromBuffer<int32_t>(buffer_ptr);
                double nsec = (double) ReadFromBuffer<int32_t>(buffer_ptr);
                flat_container->value.push_back( std::make_pair( node, (double)( sec + nsec/(1000*1000*1000) ) ) );
            };
        }break;

        case OTHER:{
            deserializeAndStore = [&](StringElement* node)
            {
                bool done = false;
                for(const ROSMessage& msg: type_list) // find in the list
                {
                    if( msg.type.msgName() == type.msgName() &&
                            msg.type.pkgName() == type.pkgName()  )
                    {
                        for (const ROSField& field: msg.fields )
                        {
                            if(field.isConstant() == false) {

                                ShortString node_name( field.name().data(), field.name().size() )  ;
                                StringElement& new_node = node->addChild( node_name );

                                buildRosFlatTypeImpl(type_list, field.type(),
                                                     &new_node,
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
        StringElement* new_node = node;

        for (int v=0; v<array_size; v++)
        {
            if( type.isArray() )
            {
                char suffix[16];
                sprintf(suffix,"[%d]", v);
                ShortString node_name( suffix );
                new_node = &(node->addChild( node_name ));
            }
            deserializeAndStore( new_node );
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

    buildRosFlatTypeImpl( type_map,
                          type,
                          flat_container.tree.root(),
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

    return flat_container;
}


} // end namespace
