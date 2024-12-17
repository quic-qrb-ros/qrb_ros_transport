cmake_minimum_required(VERSION 3.8)
project(qrb_ros_transport_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic --std=c++17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# for end-to-end test
find_package(rclcpp_components REQUIRED)
macro(add_test_component component_name class_name)
  ament_auto_add_library(${PROJECT_NAME}_${component_name}_component SHARED
    src/${component_name}.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_${component_name}_component
    rclcpp_components
  )
  rclcpp_components_register_nodes(${PROJECT_NAME}_${component_name}_component
    ${class_name}
  )
endmacro(add_test_component)

add_test_component(test_pub "qrb_ros::transport::TestPubComponent")
add_test_component(test_sub "qrb_ros::transport::TestSubComponent")
add_test_component(test_dump "qrb_ros::transport::TestDumpComponent")

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

ament_auto_package()
