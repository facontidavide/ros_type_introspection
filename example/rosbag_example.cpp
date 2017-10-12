#include "ros_type_introspection/ros_introspection.hpp"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

RosIntrospection::Parser parser;


// usage: pass the name of the file as command line argument
int main(int argc, char** argv)
{
    using namespace RosIntrospection;

    if( argc != 2 ){
        printf("Usage: rosbag_example rosbag_file.bag\n");
        return 1;
    }
    rosbag::Bag bag;

    try{
        bag.open( argv[1] );
    }
    catch( rosbag::BagException&  ex)
    {
        printf("rosbag::open thrown an exception:\n");
        return 1;
    }

    // this  rosbag::View will accept ALL the messages
    rosbag::View bag_view ( bag );

    // register (only once at the beginning) the type of messages
    for(const rosbag::ConnectionInfo* connection: bag_view.getConnections() )
    {
        const std::string&  topic_name =  connection->topic;
        const std::string&  datatype   =  connection->datatype;
        const std::string&  definition =  connection->msg_def;
        // register the type using the topic_name as identifier.
        parser.registerMessageDefinition(topic_name, ROSType(datatype), definition);
    }

    // it is efficient to reuse the same instance of FlatMessage and RenamedValues
    // over and over again, to reduce the amount of memory allocations
    std::map<std::string, FlatMessage>   flat_containers;
    std::map<std::string, RenamedValues> renamed_vectors;

    // This buffer will store the raw bytes where the message is encoded
    std::vector<uint8_t> buffer;

    for(rosbag::MessageInstance msg_instance: bag_view)
    {
        const std::string& topic_name  = msg_instance.getTopic();

        // write the message into the buffer
        const size_t msg_size  = msg_instance.size();
        buffer.resize(msg_size);
        ros::serialization::OStream stream(buffer.data(), buffer.size());
        msg_instance.write(stream);

        FlatMessage&   flat_container = flat_containers[topic_name];
        RenamedValues& renamed_values = renamed_vectors[topic_name];

        // deserialize and rename the vectors
        parser.deserializeIntoFlatContainer( topic_name,
                                             absl::Span<uint8_t>(buffer),
                                             &flat_container, 100 );
        parser.applyNameTransform( topic_name,
                                   flat_container,
                                   &renamed_values );

        // Print the content of the message
        printf("--------- %s ----------\n", topic_name.c_str() );
        for (auto it: renamed_values)
        {
            const std::string& key = it.first;
            const Variant& value   = it.second;
            printf(" %s = %f\n", key.c_str(), value.convert<double>() );
        }
        for (auto it: flat_container.name)
        {
            const std::string& key    = it.first.toStdString();
            const std::string& value  = it.second;
            printf(" %s = %s\n", key.c_str(), value.c_str() );
        }
    }

    return 0;
}
