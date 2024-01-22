cmake_minimum_required(VERSION 3.8)
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


add_library(ufr_gtw_ros_humble SHARED src/ufr_gtw_ros_humble.cpp)
ament_target_dependencies(ufr_gtw_ros_humble rclcpp std_msgs)
target_link_libraries(ufr_gtw_ros_humble ufr)

add_library(ufr_ecr_ros_humble SHARED src/ufr_ecr_string.cpp)
ament_target_dependencies(ufr_ecr_ros_humble rclcpp std_msgs geometry_msgs)
target_link_libraries(ufr_ecr_ros_humble ufr)

add_library(ufr_dcr_ros_humble SHARED src/ufr_dcr_twist.cpp)
ament_target_dependencies(ufr_dcr_ros_humble rclcpp std_msgs geometry_msgs)
target_link_libraries(ufr_dcr_ros_humble ufr)
