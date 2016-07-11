#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/JointState.h>
#include "rosbag/bag.h"
#include <sstream>
#include <stdio.h>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    geometry_msgs::Pose pose;

    pose.position.x = 1;
    pose.position.y = 2;
    pose.position.z = 3;

    pose.orientation.x = 4;
    pose.orientation.y = 5;
    pose.orientation.z = 6;
    pose.orientation.w = 7;

    sensor_msgs::JointState joint_state;

   /*
    >> joint_state.header.seq	/ uint32
    >> joint_state.header.stamp	/ time
    >> joint_state.header.frame_id	/ string
    >> joint_state.name	/ string[]
    >> joint_state.position	/ float64[]
    >> joint_state.velocity	/ float64[]
    >> joint_state.effort	/ float64[]*/

    joint_state.header.seq = 2016;
    joint_state.header.stamp = ros::Time::now();
    joint_state.header.frame_id = "pippo";

    joint_state.name.resize( 3 );
    joint_state.position.resize( 3 );
    joint_state.velocity.resize( 3 );
    joint_state.effort.resize( 3 );

    std::string names[3];
    names[0] = std::string("hola");
    names[1] = std::string("ciao");
    names[2] = std::string("bye");

    for (int i=0; i<3; i++)
    {
        joint_state.name[i] = names[i];
        joint_state.position[i]= 11+i;
        joint_state.velocity[i]= 21+i;
        joint_state.effort[i]= 31+i;
    }


    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose>("test_pose", 1000);
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("test_joint", 1000);

    ros::Rate loop_rate(1);

    int count = 0;
    while (count < 100)
    {
        printf("publish %d\n", count);

        pose.orientation.w  = count;

        chatter_pub.publish(pose);


        joint_pub.publish(joint_state);

        ros::spinOnce();

        //------------------------
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
