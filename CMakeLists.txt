cmake_minimum_required(VERSION 3.8)
project(pcl_samples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

set(pass_through_executable pass_through)
set(pass_through_library ${pass_through_executable}_core)
set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(${pass_through_executable}
  src/filters/pass_through_node.cpp
  src/filters/pass_through_component.cpp
)
add_library(${pass_through_library} SHARED
  src/filters/pass_through_component.cpp
)
ament_target_dependencies(${pass_through_executable}
  ${dependencies}
)
ament_target_dependencies(${pass_through_library}
  ${dependencies}
)
rclcpp_components_register_nodes(${pass_through_library}
  "pcl_filters::PassThrough"
)
target_link_libraries(${pass_through_executable}
  ${PCL_LIBRARIES}
)
target_link_libraries(${pass_through_library}
  ${PCL_LIBRARIES}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

install(TARGETS
  ${pass_through_executable}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  ${pass_through_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_export_include_directories(include)
ament_export_libraries(${pass_through_executable} ${pass_through_library})
ament_export_dependencies(${dependencies})
ament_package()
