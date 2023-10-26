cmake_minimum_required(VERSION 3.8)
project(pandu_ros2_kit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(var_dependencies rclcpp geometry_msgs visualization_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(dependency ${var_dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(pandu_ros2_kit src/help_marker.cpp src/udp.cpp)
ament_target_dependencies(pandu_ros2_kit ${var_dependencies})

target_include_directories(
  pandu_ros2_kit PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS pandu_ros2_kit
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${var_dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
