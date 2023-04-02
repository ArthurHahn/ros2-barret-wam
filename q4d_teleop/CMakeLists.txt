cmake_minimum_required(VERSION 3.8)
project(q4d_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)

add_executable(q4d_teleop_rviz src/q4d_teleop_rviz.cpp)
target_include_directories(q4d_teleop_rviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(q4d_teleop_rviz PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  q4d_teleop_rviz
  "rclcpp"
  "geometry_msgs"
  "kdl_parser"
  "orocos_kdl"
  "sensor_msgs"
  "std_msgs"
  "urdf"
)

install(TARGETS q4d_teleop_rviz
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
	DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
