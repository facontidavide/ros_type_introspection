cmake_minimum_required(VERSION 2.8.3)
project(ros_type_introspection)

find_package(Boost REQUIRED COMPONENTS regex)

find_package(catkin REQUIRED COMPONENTS 
   roscpp 
   rostime 
   topic_tools 
   rosbag 
   rosbag_storage 
   roscpp_serialization
)

# Build flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ros_type_introspection
   CATKIN_DEPENDS 
   roscpp 
   rostime 
   topic_tools 
   rosbag 
   rosbag_storage 
   roscpp_serialization
   DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ library
 add_library(ros_type_introspection
   src/parser.cpp
   src/deserializer.cpp
   src/renamer.cpp

   include/ros_type_introspection/parser.hpp
   include/ros_type_introspection/deserializer.hpp
   include/ros_type_introspection/string.hpp
   include/ros_type_introspection/renamer.hpp
   include/ros_type_introspection/stringtree.hpp
 )

target_link_libraries(ros_type_introspection ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(ros_introspection_benchmark src/tests/benchmark.cpp)
add_dependencies(ros_introspection_benchmark
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros_introspection_benchmark
 ${catkin_LIBRARIES}
  ros_type_introspection
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


## Mark executables and/or libraries for installation
install(TARGETS ros_type_introspection 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )

install(TARGETS ros_introspection_benchmark 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".git" EXCLUDE
)


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)

 catkin_add_gtest(ros_introspection_test
     src/tests/parser_test.cpp
     src/tests/deserializer_test.cpp
     src/tests/renamer_test.cpp
     )

 target_link_libraries(ros_introspection_test
   ${catkin_LIBRARIES} boost_regex
   ros_type_introspection
   )

endif()
