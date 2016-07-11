#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "rosbag/bag.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");


  ros::NodeHandle n;
  
  sensor_msgs::JointState joint_state;

  int NUM_JOINTS = 30;
  joint_state.name.resize(NUM_JOINTS);
  joint_state.position.resize(NUM_JOINTS);
  joint_state.velocity.resize(NUM_JOINTS);
  joint_state.effort.resize(NUM_JOINTS);

  for (int i=0; i<NUM_JOINTS; i++ )
  {
      char buff[100];
      sprintf(buff, "pepito_joint_%d", i);
      joint_state.name[i] = std::string(buff);
      joint_state.position[i] = i;
      joint_state.velocity[i] = i*2.1111;
      joint_state.effort[i] = i*3.222;
  }

  rosbag::Bag raw_bag("raw_bag.bag",   rosbag::bagmode::Write );
  rosbag::Bag comp_bag("comp_bag.bag", rosbag::bagmode::Write );

  raw_bag.setCompression( rosbag::compression::Uncompressed );
  comp_bag.setCompression( rosbag::compression::BZ2 );

  for (int a=0; a< 1000; a++)
  {

      for (int i=0; i<NUM_JOINTS; i++ )
      {
          joint_state.position[i] = i + (a+i)*0.1;
          joint_state.velocity[i] = i*2.1111 + (a+i)*0.11;
          joint_state.effort[i] = i*3.111 + (a+i)*0.12;
      }

      raw_bag.write("joint_state", ros::Time::now(),  joint_state);
      comp_bag.write("joint_state", ros::Time::now(),  joint_state);
  }




  return 0;
}
