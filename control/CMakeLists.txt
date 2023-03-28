cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
project(control)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_AUTOMOC ON)
find_package(Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set (CMAKE_SHARED_LINKER_FLAGS "-lpthread -pthread")



add_executable(control main.cpp subscriber.cpp odometry.cpp motorcontrol.cpp trajectoryexecutor.cpp pathplanner.cpp varibleradiusmotion.cpp publisherbase.cpp
    particlefilter.cpp guiwindow.cpp nmeamsgreader.cpp linefollower.cpp)
qt5_use_modules(control Widgets Core)
target_link_libraries(control ${GAZEBO_LIBRARIES}   ${Qt5Core_LIBRARIES} ${Qt5Widgets_LIBRARIES} pthread)
