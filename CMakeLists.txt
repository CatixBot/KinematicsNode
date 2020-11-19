cmake_minimum_required(VERSION 3.15)

project(kinematics_node)

add_subdirectory(msg)


find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    catix_messages)

qt5_wrap_cpp(QT_MOC_HEADERS
    src/SimulationWindow.h)

qt5_wrap_ui(QT_UI_HEADERS
    src/simulationwindow.ui)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}) 

include_directories(
    src/
    ${catkin_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS})

file(GLOB_RECURSE ALL_SOURCES "src/*.cpp")
add_executable(kinematics_node 
    ${ALL_SOURCES} 
    ${QT_MOC_HEADERS}
    ${QT_UI_HEADERS}
    ${QT_UI_SOURCES})

target_link_libraries(kinematics_node 
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
    Qt5::Widgets)

add_dependencies(kinematics_node 
    catix_messages_generate_messages)
