cmake_minimum_required(VERSION 3.15)

project(kinematics_node)

add_subdirectory(msg)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    catix_messages)

include_directories(
    src/
    ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE ALL_SOURCES "src/*.cpp")
add_executable(kinematics_node ${ALL_SOURCES})
target_link_libraries(kinematics_node ${catkin_LIBRARIES})

add_dependencies(kinematics_node 
    catix_messages_generate_messages)
