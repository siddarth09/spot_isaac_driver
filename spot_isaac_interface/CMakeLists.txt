cmake_minimum_required(VERSION 3.8)
project(spot_isaac_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Find dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

# PCL + conversions (IMPORTANT)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Message generation
find_package(rosidl_default_generators REQUIRED)

# =========================
# Messages
# =========================
set(msg_files
  "msg/Pose.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs
)

# =========================
# Converter Node
# =========================
add_executable(rs_converter_node
  src/converter.cpp
)

ament_target_dependencies(rs_converter_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
)

target_link_libraries(rs_converter_node
  ${PCL_LIBRARIES}
)

target_include_directories(rs_converter_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)



# =========================
# Install
# =========================
install(TARGETS
  rs_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

# =========================
# Testing
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
