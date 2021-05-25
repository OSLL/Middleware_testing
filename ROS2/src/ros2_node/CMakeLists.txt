cmake_minimum_required(VERSION 3.5)
project(ros2_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(../../../interface)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_compile_definitions(ROS2_TESTING)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(ros2_node
        "msg/TestData.msg"
        )

ament_export_dependencies(rosidl_default_runtime)

add_executable(node src/main.cpp src/Publisher.cpp src/Subscriber.cpp src/PingPong.cpp)
ament_target_dependencies(node rclcpp std_msgs)

rosidl_target_interfaces(node
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
        node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
