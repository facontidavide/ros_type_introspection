#Ros message introspection
###... If you don't know why you need it, probably you don't need it.

This simple library extracts information from a ROS message/type
unknown at compilation time. 

Have you ever wanted to build an app that can subscribe to __any__ 
`topic` and extract its content, or can read from __any__ `rosbag`? 
What if the topic and/or the bag contains user defined ROS types ignored at compilation time?

The common solution in the ROS ecosystem is to use Python, that provides
the needed introspection. Tools, for instance, like __rqt_plot__ and __rqt_bag__ take this approach.
This library implements a __C++ alternative__.

Introspections is achieved parsing the schema stored in `ros::message_traits::Definition< ... >`

Once this schema is parsed and understood, it can be used to deserialize a raw message.

It also offers an easy way to remap/rename the data using a simple set of 
rules.

#Background

The ROS Message Types can be described as 
an [Interface Describtion Language](https://en.wikipedia.org/wiki/Interface_description_language).
This approach is very well known and commonly used on the web and in distributed systems in general.

A [rosmsg](http://wiki.ros.org/rosmsg) is defined by the user; [gencpp](http://wiki.ros.org/gencpp)
is an "IDL compiler" that reads the schema and generates a header file. The latter shall 
be included in the applications, both on the publisher *and* the subscriber side.

This approach creates strong and type-safe contracts between the producer and the consumer 
of the message and, additionally, is needed to implements a fast 
serialization / deserialization mechanism.

The only "limitation", at least in C++, is the fact that the generated header files 
must be included in the source code.

Using ros_type_introspection you can avoid using this generate headers files on the 
consumer/subscriber side.

##The parser

In most cases we have access to the Ros Message Type Definition.
This is true for both rosbags and topic subscribers.

The definition contains __all__ the information we need to know how to deserialize 
the ROS message.
The goal of the [parser](ros-type-introspection/blob/master/include/ros_type_introspection/parser.hpp)
is to extract the schema and allow the user (and the deserializer) to introspect it.

You just need to invoke a single funtion:

```c++
  ROSTypeList buildROSTypeMapFromDefinition( const std::string& type_name,
                                             const std::string& msg_definition );
```
###Example 1

Let's take a look to a simple example. Further we try to parse the type(s) contained in
[geometry_msgs::Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html)

```c++
  #include <geometry_msgs/Pose.h>
  // NOTE: in this trivial example we need to include geometry_msgs/Pose
  // even if the main goal of this library is to AVOID that.
  
  ///...
  
  using namespace RosIntrospection;
  ROSTypeList typelist = buildROSTypeMapFromDefinition(
                ros::message_traits::DataType< geometry_msgs::Pose >::value(),
                ros::message_traits::Definition< geometry_msgs::Pose >::value());
  std::cout << typelist << std::endl;             
```
The expected output is:

```
   geometry_msgs/Pose : 
      position : geometry_msgs/Point
      orientation : geometry_msgs/Quaternion

   geometry_msgs/Point : 
      x : float64
      y : float64
      z : float64

   geometry_msgs/Quaternion : 
      x : float64
      y : float64
      z : float64
      w : float64
```

As expected each message type has a set of fields with a fieldname and a typename.

Note as all of the non-built-in types in the hierarchy 
are parsed as well *recursively*.
In this case `geometry_msgs/Point` and `geometry_msgs/Quaternion`.

###Example 2

In the next example we will parse all the types stored in a single ROS bag.
We will __not__ need to __`#include`__ any of those ROS Types. 

To understand this chunk of code you must
be familiar with the [rosbag::Bag API](http://wiki.ros.org/rosbag/Code%20API)

```c++
  rosbag::Bag bag;
  bag.open( file_name, rosbag::bagmode::Read );

  rosbag::View bag_view ( bag, ros::TIME_MIN, ros::TIME_MAX, true );
  auto first_time = bag_view.getBeginTime();

  const auto& connections = bag_view.getConnections();

  // create a list and a type map for each topic
  std::map<std::string, RosIntrospection::ROSTypeList> type_map;

  for(unsigned i=0; i<connections.size(); i++)
  {
     auto topic_map = buildROSTypeMapFromDefinition( connections[i]->datatype,
                                                     connections[i]->msg_def);
                                                     
     type_map.insert( std::make_pair(connections[i]->topic, topic_map));
  }
```

# The deserializer

The next thing to understand is the 
[deserializer](ros-type-introspection/blob/master/include/ros_type_introspection/deserializer.hpp).
Once the schema is available in the form of a 'RosIntrospection::ROSTypeList` we are
able to take a raw message and extract valuable information from that.

We don't have the support of the C++ typesystem, which was provided by the 
included file generated by the IDL compiler, therefore the fields of the message can not be
"composed" into a `struct` or `class`.

The only data structure that can contain our data is currently a flat structure that store
simple key-value pairs:

```c++
  // note; LongString is just a string with improved small object optimization.
  typedef struct{
    std::vector< std::pair<LongString, double> > value;
    std::vector< std::pair<LongString, LongString> > name_id;
  }ROSTypeFlat;
```

This highlights already some of the main limitations of the parser:

* It is not well suited for objects with large arrays, like images, maps point clouds.
From a very selfish point of view, I am not optimizing this use case because I don't need it.
Very large arrays are simply discarted. 

* A double is used as a "conservative" type to store any integral. This, together with the LongString class
makes the code simpler but inefficient from the point of view of memory.

* LongString run faster than std:string in many cases, because it use stack allocation instead of heap allocation.
Unfortunately, as a result more RAM is needed. Nevertheless it is easy to change this with compilation flags.

### Example 3 
 
Let's see how parser and deserializer work together with a slightly more complex type, 
[sensor_msgs::JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html). 

Let's suppose that a publisher sends this instance of __sensor_msgs::JointState__ using a ROS topic
(I am not including the ROS related code):

```c++
    sensor_msgs::JointState joint_state;

    joint_state.header.seq = 2016;
    joint_state.header.stamp.sec  = 1234;
    joint_state.header.stamp.nsec = 567*1000*1000;
    joint_state.header.frame_id = "pippo";

    joint_state.name.resize( 3 );
    joint_state.position.resize( 3 );
    joint_state.velocity.resize( 3 );
    joint_state.effort.resize( 3 );

    std::string names[3];
    names[0] = ("hola");
    names[1] = ("ciao");
    names[2] = ("bye");

    for (int i=0; i<3; i++){
        joint_state.name[i] = names[i];
        joint_state.position[i]= 11+i;
        joint_state.velocity[i]= 21+i;
        joint_state.effort[i]= 31+i;
    }
  //publish this on a ros topic...
  
```

On the receiver side we want to read this data but we don't know that it is a `sensor_msgs::Imu` at compilation
time. Therefore we are not able to include '<sensor_msgs/JointState>`.

To solve this problem we need the support of an usefull but not well know class:
[topic_tools::ShapeShifter](http://docs.ros.org/diamondback/api/topic_tools/html/classtopic__tools_1_1ShapeShifter.html)

```c++
//callback subscribed to the topic
void DataStreamROS::topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg)
{
    using namespace RosIntrospection;

	// reuse type already parsed.
    static std::map<std::string, ROSTypeList> registered_types;

    auto& datatype_name = msg->getDataType();

	// new type. Need to be parsed and stored.
    if( registered_types.find( datatype_name ) == registered_types.end() )
    {
        registered_type[datatype_name] = buildROSTypeMapFromDefinition(
										datatype_name,
										msg->getMessageDefinition() );
    }
    std::vector<uint8_t> buffer( msg->size() ); 

    ROSTypeFlat& flat_container;

    ros::serialization::OStream stream(buffer, sizeof(buffer));
    msg->write(stream);

    // Important: use a copy of the pointer.
    uint8_t* buffer_ptr = buffer;
    
    LongString topicname( topic_name.data(), topic_name.length() );

    flat_container = buildRosFlatType( registered_type[datatype_name], 
                                       ROSType(datatype_name), 
                                       topicname, 
                                       &buffer_ptr);
                                       
    for(auto&it: flat_container.value) {
        std::cout << it.first << " >> " << it.second << std::endl;
    }
    std::cout << "----" << std::endl;
    for(auto&it: flat_container.name_id) {
        std::cout << it.first << " >> " << it.second << std::endl;
    }
}

```

The exepected output is:

```
JointState.header.seq >> 2016
JointState.header.stamp >> 1234.57
JointState.position[0] >> 11
JointState.position[1] >> 12
JointState.position[2] >> 13
JointState.velocity[0] >> 21
JointState.velocity[1] >> 22
JointState.velocity[2] >> 23
JointState.effort[0] >> 31
JointState.effort[1] >> 32
JointState.effort[2] >> 33
----
JointState.header.frame_id >> pippo
JointState.name[0] >> hola
JointState.name[1] >> ciao
JointState.name[2] >> bye

```





 



 
