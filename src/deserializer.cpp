#include <ros_type_introspection/deserializer.hpp>
#include <functional>

namespace RosIntrospection{


template <typename T> T ReadFromBuffer( uint8_t** buffer)
{
    for (int i=0; i< sizeof(T); i++)
    {
        //     printf(" %02X ", (*buffer)[i] );
    }
    T destination =  (*( reinterpret_cast<T*>( *buffer ) ) );
    *buffer +=  sizeof(T);
    return destination;
}


void buildRosFlatTypeImpl(const ROSTypeList& type_list,
                          const ROSType &type,
                          LongString prefix,
                          uint8_t** buffer_ptr,
                          ROSTypeFlat* flat_container )
{
    int array_size = type.arraySize();
    if( array_size == -1)
    {
        array_size = ReadFromBuffer<int32_t>( buffer_ptr );
    }

    std::function<void(const LongString&)> deserializeAndStore;

    switch( type.typeID())
    {
        case STRING: {
            deserializeAndStore = [&](const LongString& key)
            {
                int32_t string_size = ReadFromBuffer<int32_t>( buffer_ptr );
                LongString id( (const char*)(*buffer_ptr), string_size );
                (*buffer_ptr) += string_size;
                flat_container->name_id.push_back( std::make_pair( key, id ) );
            };
        }break;

        case FLOAT64: {
            deserializeAndStore = [&](const LongString& key){
                flat_container->value.push_back( std::make_pair( key, (double) ReadFromBuffer<double>(buffer_ptr) ) );
            };
        }break;
        case FLOAT32: {
            deserializeAndStore = [&](const LongString& key){
                flat_container->value.push_back( std::make_pair( key, (double) ReadFromBuffer<float>(buffer_ptr) ) );
            };
        }break;
        case TIME: {
            deserializeAndStore = [&](const LongString& key){
                double sec  = (double) ReadFromBuffer<uint32_t>(buffer_ptr);
                double nsec = (double) ReadFromBuffer<uint32_t>(buffer_ptr);
                flat_container->value.push_back( std::make_pair( key, (double)( sec + nsec/(1000*1000*1000) ) ) );
            };
        }break;
        case UINT64: {
            deserializeAndStore = [&](const LongString& key){
                flat_container->value.push_back( std::make_pair( key, (double) ReadFromBuffer<uint64_t>(buffer_ptr) ) );
            };
        }break;
        case INT64: {
            deserializeAndStore = [&](const LongString& key){
                flat_container->value.push_back( std::make_pair( key, (double) ReadFromBuffer<int64_t>(buffer_ptr) ) );
            };
        }break;
        case UINT32: {
            deserializeAndStore = [&](const LongString& key){
                flat_container->value.push_back( std::make_pair( key, (double) ReadFromBuffer<uint32_t>(buffer_ptr) ) );
            };
        }break;
        case INT32: {
            deserializeAndStore = [&](const LongString& key){
                flat_container->value.push_back( std::make_pair( key, (double) ReadFromBuffer<int32_t>(buffer_ptr) ) );
            };
        }break;
        case UINT16: {
            deserializeAndStore = [&](const LongString& key){
                flat_container->value.push_back( std::make_pair( key, (double) ReadFromBuffer<uint16_t>(buffer_ptr) ) );
            };
        }break;
        case INT16: {
            deserializeAndStore = [&](const LongString& key){
                flat_container->value.push_back( std::make_pair( key, (double) ReadFromBuffer<int16_t>(buffer_ptr) ) );
            };
        }break;
        case BOOL:
        case UINT8: {
            deserializeAndStore = [&](const LongString& key){
                flat_container->value.push_back( std::make_pair( key, (double) ReadFromBuffer<uint8_t>(buffer_ptr) ) );
            };
        }break;
        case BYTE:
        case INT8: {
            deserializeAndStore = [&](const LongString& key){
                flat_container->value.push_back( std::make_pair( key, (double) ReadFromBuffer<int8_t>(buffer_ptr) ) );
            };
        }break;

        case DURATION: {
            deserializeAndStore = [&](const LongString& key){
                double sec  = (double) ReadFromBuffer<int32_t>(buffer_ptr);
                double nsec = (double) ReadFromBuffer<int32_t>(buffer_ptr);
                flat_container->value.push_back( std::make_pair( key, (double)( sec + nsec/(1000*1000*1000) ) ) );
            };
        }break;

        case OTHER:{
            deserializeAndStore = [&](const LongString& key)
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
                            LongString new_prefix( key );
                            new_prefix.append( "." );
                            new_prefix.append( field.name().data(), field.name().size() )  ;

                            buildRosFlatTypeImpl(type_list, field.type(),
                                                 new_prefix,
                                                 buffer_ptr,
                                                 flat_container);
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

    for (int v=0; v<array_size; v++)
    {
        LongString key (prefix);
        if( type.isArray() )
        {
            char suffix[16];
            sprintf(suffix,"[%d]", v);
            key.append( suffix );
        }
        deserializeAndStore(key);
    }
}


ROSTypeFlat buildRosFlatType(const ROSTypeList& type_map,
                             ROSType type,
                             const LongString & prefix,
                             uint8_t** buffer_ptr)
{
    ROSTypeFlat flat_container;


    buildRosFlatTypeImpl( type_map, type, prefix, buffer_ptr,  &flat_container );

    std::sort( flat_container.name_id.begin(),  flat_container.name_id.end(),
               []( const std::pair<LongString,LongString> & left,
               const std::pair<LongString,LongString> & right)
    {
        return left.first < right.first;
    }
    );

    return flat_container;
}


} // end namespace
