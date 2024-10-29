cmake_minimum_required(VERSION 3.5)
project(ufr-stk-ros_humble)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# Gateway
add_library(ufr_gtw_ros_humble SHARED src/ufr_gtw_ros_humble.cpp)
ament_target_dependencies(ufr_gtw_ros_humble rclcpp std_msgs sensor_msgs)
target_link_libraries(ufr_gtw_ros_humble ufr)
install(TARGETS ufr_gtw_ros_humble DESTINATION lib)

# Encoder
add_library(ufr_enc_ros_humble SHARED 
    src/ufr_enc_string.cpp
    src/ufr_enc_pose.cpp
    src/ufr_enc_image.cpp
    src/ufr_enc_twist.cpp
    src/ufr_enc_laserscan.cpp
    )
ament_target_dependencies(ufr_enc_ros_humble rclcpp std_msgs geometry_msgs sensor_msgs)
target_link_libraries(ufr_enc_ros_humble ufr)
install(TARGETS ufr_enc_ros_humble DESTINATION lib)

# Decoder
add_library(ufr_dcr_ros_humble SHARED
    src/ufr_dcr_string.cpp
    src/ufr_dcr_pose.cpp
    src/ufr_dcr_image.cpp
    src/ufr_dcr_twist.cpp
    src/ufr_dcr_laserscan.cpp
    )
ament_target_dependencies(ufr_dcr_ros_humble rclcpp std_msgs geometry_msgs sensor_msgs turtlesim)
target_link_libraries(ufr_dcr_ros_humble ufr)
install(TARGETS ufr_dcr_ros_humble DESTINATION lib)

# Tests
enable_testing()
add_subdirectory(./tests)