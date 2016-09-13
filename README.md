#Ros message introspection
###... If you don't know why you need it, probably you don't need it.

This is a simple library that extracts information from a ROS message/type
unknown at compilation time. 

Have you ever wanted to build an app that can subscribe to __any__ 
`topic` and extract its content, or can read from __any__ `rosbag`?

What if the topic and/or the bag contains user defined ROS types ignored at compilation time?

The common solution in the ROS ecosystem is to use Python, that provides
the needed introspection. Tools, for instance, like __rqt_plot__ and __rqt_bag__ take this approach.

This library implements a C++ alternative.

Introspections is achieved parsing the schema stored in `ros::message_traits::Definition< ... >::value()`

Once this schema is parsed and understood, it can be used to deserialize a raw message.

It also offer an easy way to remap/rename the data using a simple set of 
rules.

##Background

The ROS message types (I will refer to them as "ROS types") can be described as 
a [Interface Describtion Language](https://en.wikipedia.org/wiki/Interface_description_language).
This approach is very well known and commonly used on the web and in distributed systems in general.

In general a [rosmsg](http://wiki.ros.org/rosmsg) is defined by the user; an "IDL compiler" 
uses this schema to generate an header file that contains the code that we usually included
in our application.

This approach creates strong and type-safe contracts between the producer and the consumer 
of the message and, additionally, is needed to implements the fast 
serialization / deserialization mechanism.

The only "limitation", at least in C++, is the fact that the generated header files 
must be included in the source code.

##The parser
In most cases we have access to the Ros Message Type Definition.
Luckily for us this string contains __all_ the information we need to know how to deserialize 
the ROS message.
The goal of the [parser](ros-type-introspection/blob/master/include/ros_type_introspection/parser.hpp)
is to extract the schema and made it available to the user and the deserializer.

This can be simply done calling the function:

```c++
  ROSTypeList buildROSTypeMapFromDefinition( const std::string& type_name,
                                             const std::string& msg_definition );
```

Let's take a look to a simple example. Further we try to parse the type(s) contained in
[geometry_msgs::Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html)

```c++
  #include <geometry_msgs/Pose.h>
  // NOTE: in this trivial example we need to include geometry_msgs/Pose
  // even if the main goal of this library is to avoid that.
  
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

As expected each message typw has a set of fields with a name and a type.
What is interesting is that also all the non-built-in type in the hierarchy 
are parsed, specifically ` geometry_msgs/Point` and `geometry_msgs/Point`.

In the next example we parse all the types found in a ROS bag.
We will __not__ need to `#include` any of those types. 

To understand this chunk of code you must
be familiar with the [rosbag::Bag API](http://wiki.ros.org/rosbag/Code%20API)

```c++
  rosbag::Bag bag;
  bag.open( file_name, rosbag::bagmode::Read );

  rosbag::View bag_view ( bag, ros::TIME_MIN, ros::TIME_MAX, true );
  auto first_time = bag_view.getBeginTime();

  const auto& connections = bag_view.getConnections();

  // create a list and a type map for each topic
  std::map<std::string,ROSTypeList> type_map;

  for(unsigned i=0; i<connections.size(); i++)
  {
     auto topic_map = buildROSTypeMapFromDefinition( connections[i]->topic,
                                                     connections[i]->msg_def);
                                                     
     type_map.insert( std::make_pair(connections[i]->datatype, topic_map));
  }
```



 



 
