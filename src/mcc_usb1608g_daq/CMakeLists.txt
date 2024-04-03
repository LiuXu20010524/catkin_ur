cmake_minimum_required(VERSION 2.8.3)
project(mcc_usb1608g_daq)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES mcc_usb1608g_daq
        CATKIN_DEPENDS message_runtime
        #  DEPENDS system_lib
)

link_directories(
        lib
        ${catkin_LIB_DIRS}
)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        /usr/include/eigen3
)

## Declare a C++ library
add_library(mcc_1608g
        src/mcc_usb1608g.cpp
        src/filter/onlineFilter.cpp
)
target_link_libraries(mcc_1608g
        mccusb
        hidapi-libusb
        usb-1.0
)

add_executable(publish_mcc_usb1608g src/publish_mcc_usb1608g.cpp)
target_link_libraries(publish_mcc_usb1608g
        ${catkin_LIBRARIES}
        mcc_1608g
)

add_executable(start_mcc_usb1608g src/start.cpp)
target_link_libraries(start_mcc_usb1608g
        ${catkin_LIBRARIES}
        mcc_1608g
)

add_executable(mcc_bias_acquire src/mcc_bias_acquire.cpp)
target_link_libraries(mcc_bias_acquire
        ${catkin_LIBRARIES}
        mcc_1608g
)
