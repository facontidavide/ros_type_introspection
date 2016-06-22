#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "topic_tools/shape_shifter.h"
#include "ros-type-parser.h"
#include <sensor_msgs/JointState.h>
#include "boost/algorithm/string.hpp"

void callbackPose(const geometry_msgs::Pose::ConstPtr& msg)
{
    /*  ROS_INFO("I heard: %.2f %.2f %.2f \t %.2f %.2f %.2f %.2f\n",
             msg->position.x, msg->position.y, msg->position.z,
             msg->orientation.x,  msg->orientation.y,  msg->orientation.z,  msg->orientation.w);*/
}

void callbackJoint(const sensor_msgs::JointState::ConstPtr& msg)
{
    /*  ROS_INFO("I heard: %.2f %.2f %.2f \t %.2f %.2f %.2f %.2f\n",
             msg->position.x, msg->position.y, msg->position.z,
             msg->orientation.x,  msg->orientation.y,  msg->orientation.z,  msg->orientation.w);*/
    std::cout << "---------------------------------------" << std::endl;
    std::cout << *msg << std::endl;
    std::cout << "-------------" << std::endl;
}

std::vector<uint8_t> buffer( 1024*1024 );


void callbackShapeShiter(const topic_tools::ShapeShifter::ConstPtr& msg)
{
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg->write(stream);

    std::vector<RosTypeParser::Type> type_hierarchy;
    std::vector<RosTypeParser::Field> flat;

    type_hierarchy = RosTypeParser::parseRosTypeDescription(
                msg->getDataType(),
                msg->getMessageDefinition()   );

    flat = RosTypeParser::buildFlatTypeHierarchy( "pose", type_hierarchy, msg->getDataType() );

    uint8_t* buffer_ptr = &buffer[0];

    const std::string vector_symbol("[]");

    for (int i=0; i<flat.size(); i++ )
    {
        double value = -1;
        std::string type = flat[i].type_name ;
        int32_t vect_size = 1;
        int32_t string_size = 0;
        char string_data[1024];

        if( boost::contains(type, vector_symbol ))
        {
            vect_size = (*(reinterpret_cast<int32_t*>( buffer_ptr ) ) );
            buffer_ptr += sizeof( vect_size );

         //   std::cout << "vect size: " << vect_size  << std::endl;
            type.erase( type.length() -2 );
        }

        if( vect_size > 100) return;



        for (int v=0; v < vect_size; v++)
        {
            std::cout << v <<"  >> "<< flat[i].field_name <<  " / " << type << " = \t" ;

            if( type.compare("float64") == 0 )
            {
                value = *(reinterpret_cast<double*>( buffer_ptr ) );
                buffer_ptr += sizeof( double );
                std::cout <<  value << std::endl;
            }
            else if( type.compare("uint32") == 0 )
            {
                value = (double)(*(reinterpret_cast<uint32_t*>( buffer_ptr ) ) );
                buffer_ptr += sizeof( uint32_t );
                std::cout <<  value << std::endl;
            }
            else if( type.compare("time") == 0 )
            {
                ros::Time time = (*(reinterpret_cast<ros::Time*>( buffer_ptr ) ) );
                buffer_ptr += sizeof( ros::Time );
                std::cout <<  time << std::endl;
            }
            else if( type.compare("string") == 0 )
            {
                string_size = (*(reinterpret_cast<int32_t*>( buffer_ptr ) ) );
                buffer_ptr += sizeof( string_size );

                for (int i=0; i< string_size; i++)
                    string_data[i] =  (char)*(buffer_ptr++);

                string_data[string_size] = '\0';
                std::cout <<  string_data << std::endl;
            }
        }
    }


}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");

    ros::NodeHandle n;
    // ros::Subscriber sub1 = n.subscribe("test_pose", 1000, callbackPose);
    // ros::Subscriber sub2 = n.subscribe("test_pose", 1000, callbackShapeShiter);

    ros::Subscriber sub3 = n.subscribe("test_joint", 1000, callbackJoint);
    ros::Subscriber sub4 = n.subscribe("test_joint", 1000, callbackShapeShiter);

    ros::spin();

    return 0;
}

