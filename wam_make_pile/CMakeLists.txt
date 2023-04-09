cmake_minimum_required(VERSION 3.8)
project(wam_make_pile)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(pose_trajectory_publisher src/pose_trajectory_publisher.cpp)
target_include_directories(pose_trajectory_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_trajectory_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pose_trajectory_publisher
  "rclcpp"
  "tf2_kdl"
  "geometry_msgs"
  "trajectory_msgs"
  "orocos_kdl"
  "kdl_parser"
)

install(TARGETS pose_trajectory_publisher
  DESTINATION lib/${PROJECT_NAME})

add_executable(joint_trajectory_publisher src/joint_trajectory_publisher.cpp)
target_include_directories(joint_trajectory_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joint_trajectory_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  joint_trajectory_publisher
  "rclcpp"
  "tf2_kdl"
  "geometry_msgs"
  "trajectory_msgs"
  "orocos_kdl"
)

install(TARGETS joint_trajectory_publisher
  DESTINATION lib/${PROJECT_NAME})
  
  
add_executable(pose_stamped2joint src/pose_stamped2joint.cpp)
target_include_directories(pose_stamped2joint PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_stamped2joint PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pose_stamped2joint
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "tf2_msgs"
  "trajectory_msgs"
  "urdf"
  "tf2_kdl"
  "kdl_parser"
  "orocos_kdl"
)

add_executable(pose_stamped2tf src/pose_stamped2tf.cpp)
target_include_directories(pose_stamped2tf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_stamped2tf PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pose_stamped2tf
  "rclcpp"
  "geometry_msgs"
  "tf2_msgs"
  "tf2_kdl"
)

install(TARGETS pose_stamped2joint pose_stamped2tf
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
)  

install(PROGRAMS
	scripts/set_home.sh
	scripts/step.sh
	scripts/step_home.sh
	scripts/step_zero.sh
	scripts/spawn_coke_can.sh
	DESTINATION lib/${PROJECT_NAME}
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
