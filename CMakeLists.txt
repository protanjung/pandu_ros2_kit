cmake_minimum_required(VERSION 3.8)
project(pandu_ros2_kit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(var_dependencies
    cv_bridge
    pcl_conversions
    rclcpp
    rclpy
    sensor_msgs
    visualization_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
foreach(var_dependency ${var_dependencies})
  find_package(${var_dependency} REQUIRED)
endforeach()

add_library(
  ${PROJECT_NAME}
  src/help_logger.cpp
  src/help_marker.cpp
  src/help_pid.cpp
  src/help_pure_pursuit.cpp
  src/help_udp.cpp)
ament_target_dependencies(${PROJECT_NAME} ${var_dependencies})

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${var_dependencies})

include_directories(include)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(io_lslidar_c16 src/io_lslidar_c16.cpp)
add_executable(io_lslidar_n301 src/io_lslidar_n301.cpp)
add_executable(io_vision src/io_vision.cpp)
ament_target_dependencies(io_lslidar_c16 ${var_dependencies})
ament_target_dependencies(io_lslidar_n301 ${var_dependencies})
ament_target_dependencies(io_vision ${var_dependencies})
target_link_libraries(io_lslidar_c16 ${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(io_lslidar_n301 ${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(io_vision ${PROJECT_NAME} ${OpenCV_LIBS})

install(TARGETS io_lslidar_c16 io_lslidar_n301 io_vision
        DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/io_sound.py scripts/tool_face_detector.py
                 scripts/tool_mjpeg_server.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY assets DESTINATION share/${PROJECT_NAME})

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
