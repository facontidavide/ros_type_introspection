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
                          SString prefix,
                          uint8_t** buffer_ptr,
                          ROSTypeFlat* flat_container,
                          uint8_t max_array_size )
{
    int array_size = type.arraySize();
    if( array_size == -1)
    {
        array_size = ReadFromBuffer<int32_t>( buffer_ptr );
    }

    std::function<void(SString&)> deserializeAndStore;

    switch( type.typeID())
    {
        case STRING: {
            deserializeAndStore = [&](SString& key)
            {
                int32_t string_size = ReadFromBuffer<int32_t>( buffer_ptr );
                SString id( (const char*)(*buffer_ptr), string_size );
                (*buffer_ptr) += string_size;
                flat_container->name_id.push_back( std::make_pair( std::move(key), id ) );
            };
        }break;

        case FLOAT64: {
            deserializeAndStore = [&](SString& key){
                flat_container->value.push_back( std::make_pair( std::move(key), (double) ReadFromBuffer<double>(buffer_ptr) ) );
            };
        }break;
        case FLOAT32: {
            deserializeAndStore = [&](SString& key){
                flat_container->value.push_back( std::make_pair( std::move(key), (double) ReadFromBuffer<float>(buffer_ptr) ) );
            };
        }break;
        case TIME: {
            deserializeAndStore = [&](SString& key){
                double sec  = (double) ReadFromBuffer<uint32_t>(buffer_ptr);
                double nsec = (double) ReadFromBuffer<uint32_t>(buffer_ptr);
                flat_container->value.push_back( std::make_pair( std::move(key), (double)( sec + nsec/(1000*1000*1000) ) ) );
            };
        }break;
        case UINT64: {
            deserializeAndStore = [&](SString& key){
                flat_container->value.push_back( std::make_pair( std::move(key), (double) ReadFromBuffer<uint64_t>(buffer_ptr) ) );
            };
        }break;
        case INT64: {
            deserializeAndStore = [&](SString& key){
                flat_container->value.push_back( std::make_pair( std::move(key), (double) ReadFromBuffer<int64_t>(buffer_ptr) ) );
            };
        }break;
        case UINT32: {
            deserializeAndStore = [&](SString& key){
                flat_container->value.push_back( std::make_pair( std::move(key), (double) ReadFromBuffer<uint32_t>(buffer_ptr) ) );
            };
        }break;
        case INT32: {
            deserializeAndStore = [&](SString& key){
                flat_container->value.push_back( std::make_pair( std::move(key), (double) ReadFromBuffer<int32_t>(buffer_ptr) ) );
            };
        }break;
        case UINT16: {
            deserializeAndStore = [&](SString& key){
                flat_container->value.push_back( std::make_pair( std::move(key), (double) ReadFromBuffer<uint16_t>(buffer_ptr) ) );
            };
        }break;
        case INT16: {
            deserializeAndStore = [&](SString& key){
                flat_container->value.push_back( std::make_pair( std::move(key), (double) ReadFromBuffer<int16_t>(buffer_ptr) ) );
            };
        }break;
        case BOOL:
        case UINT8: {
            deserializeAndStore = [&](SString& key){
                flat_container->value.push_back( std::make_pair( std::move(key), (double) ReadFromBuffer<uint8_t>(buffer_ptr) ) );
            };
        }break;
        case BYTE:
        case INT8: {
            deserializeAndStore = [&](SString& key){
                flat_container->value.push_back( std::make_pair( std::move(key), (double) ReadFromBuffer<int8_t>(buffer_ptr) ) );
            };
        }break;

        case DURATION: {
            deserializeAndStore = [&](SString& key){
                double sec  = (double) ReadFromBuffer<int32_t>(buffer_ptr);
                double nsec = (double) ReadFromBuffer<int32_t>(buffer_ptr);
                flat_container->value.push_back( std::make_pair( std::move(key), (double)( sec + nsec/(1000*1000*1000) ) ) );
            };
        }break;

        case OTHER:{
            deserializeAndStore = [&](SString& key)
            {
                bool done = false;
                for(const ROSMessage& msg: type_list) // find in the list
                {
                    if( msg.type.msgName() == type.msgName() &&
                            msg.type.pkgName() == type.pkgName()  )
                    {
                        for (const ROSField& field: msg.fields )
                        {
                            if(field.isConstant() ) {
                                // SKIP
                                continue;
                            }
                            SString new_prefix( key );
                            new_prefix.append( "." );
                            new_prefix.append( field.name().data(), field.name().size() )  ;

                            buildRosFlatTypeImpl(type_list, field.type(),
                                                 new_prefix,
                                                 buffer_ptr,
                                                 flat_container,
                                                 max_array_size);
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
        for (int v=0; v<array_size; v++)
        {
            SString key (prefix);
            if( type.isArray() )
            {
                char suffix[16];
                sprintf(suffix,"[%d]", v);
                key.append( suffix );
            }
            deserializeAndStore(key);
        }
    }
    else{
        SkipBytesInBuffer( buffer_ptr, array_size, type.typeID() );
    }
}


ROSTypeFlat buildRosFlatType(const ROSTypeList& type_map,
                             ROSType type,
                             const SString & prefix,
                             uint8_t** buffer_ptr,
                             uint8_t max_array_size)
{
    ROSTypeFlat flat_container;


    buildRosFlatTypeImpl( type_map, type, prefix, buffer_ptr,  &flat_container, max_array_size );

    std::sort( flat_container.name_id.begin(),  flat_container.name_id.end(),
               []( const std::pair<SString,SString> & left,
               const std::pair<SString,SString> & right)
    {
        return left.first < right.first;
    }
    );

    return flat_container;
}


} // end namespace
