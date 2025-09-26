cmake_minimum_required(VERSION 3.8)
project(meca500_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(meca500_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(sine_joint1 src/sine_joint1.cpp)
target_include_directories(sine_joint1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(sine_joint1 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  sine_joint1
  "std_msgs"
  "rclcpp"
  "sensor_msgs"
  "meca500_interfaces"
  "tf2"
  "tf2_geometry_msgs"
)

add_executable(circle_end_effector src/circle_end_effector.cpp)
target_include_directories(circle_end_effector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(circle_end_effector PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  circle_end_effector
  "std_msgs"
  "rclcpp"
  "sensor_msgs"
  "meca500_interfaces"
  "tf2"
  "tf2_geometry_msgs"
)

add_executable(ibvs_dumb src/ibvs_dumb.cpp)
target_include_directories(ibvs_dumb PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(ibvs_dumb PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ibvs_dumb
  "std_msgs"
  "rclcpp"
  "sensor_msgs"
  "meca500_interfaces"
  "tf2"
  "tf2_geometry_msgs"
)

install(TARGETS sine_joint1
                circle_end_effector
                ibvs_dumb
  DESTINATION lib/${PROJECT_NAME})

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
