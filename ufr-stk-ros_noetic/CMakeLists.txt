cmake_minimum_required(VERSION 3.5)
project(ufr-stk-ros_noetic)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

include_directories(${catkin_INCLUDE_DIRS})

## Declare ROS messages and services
# add_message_files(DIRECTORY msg FILES Num.msg)
# add_service_files(DIRECTORY srv FILES AddTwoInts.srv)
# generate_messages(DEPENDENCIES std_msgs)
catkin_package()


add_library(ufr_gtw_ros_noetic SHARED src/ufr_gtw_ros_noetic.cpp 
    src/ufr_enc_string.cpp 
    src/ufr_enc_laserscan.cpp 
    src/ufr_enc_twist.cpp
    src/ufr_enc_image.cpp)
target_link_libraries(ufr_gtw_ros_noetic ${catkin_LIBRARIES})

# add_library(ufr_enc_ros_noetic SHARED src/ufr_enc_string.cpp)
# target_link_libraries(ufr_enc_ros_noetic ${catkin_LIBRARIES})

# Tests
# enable_testing()
add_subdirectory(./tests)