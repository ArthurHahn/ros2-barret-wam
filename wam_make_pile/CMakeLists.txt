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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(wam_make_pile src/wam_make_pile.cpp)
target_include_directories(wam_make_pile PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(wam_make_pile PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  wam_make_pile
  "rclcpp"
  "tf2_kdl"
  "geometry_msgs"
  "trajectory_msgs"
  "orocos_kdl"
  "sensor_msgs"
  "std_msgs"
)

install(TARGETS wam_make_pile
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
	scripts/set_home.sh
	scripts/step.sh
	scripts/step_home.sh
	scripts/step_zero.sh
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
