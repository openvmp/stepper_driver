cmake_minimum_required(VERSION 3.10)
project(remote_stepper_driver)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if($ENV{ROS_DISTRO} STREQUAL "foxy")
  add_compile_options(-DNO_PARAMETER_EVENT_HANDLER)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(remote_actuator REQUIRED)

# Logic shared by all targets
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ParamPprGet.srv"
  "srv/ParamPprSet.srv"
)
include_directories(
  include
  ${remote_actuator_INCLUDE_DIRS}
)

set(project_SOURCE_FILES
  src/implementation.cpp
)

set(project_DEPENDENCIES
  rclcpp
  std_msgs
  remote_actuator
)

# Executable target
add_executable(${PROJECT_NAME}_standalone src/main.cpp ${project_SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME}_standalone ${project_DEPENDENCIES})
install(TARGETS
  ${PROJECT_NAME}_standalone
  DESTINATION lib/${PROJECT_NAME}
)

# Library target
add_library(${PROJECT_NAME}_native ${project_SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME}_native ${project_DEPENDENCIES})
ament_export_targets(${PROJECT_NAME}_native_library HAS_LIBRARY_TARGET)
ament_export_dependencies(${project_DEPENDENCIES})
ament_export_include_directories(include)
install(TARGETS ${PROJECT_NAME}_native
  EXPORT ${PROJECT_NAME}_native_library
  LIBRARY DESTINATION lib
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)


if($ENV{ROS_DISTRO} STREQUAL "foxy")
rosidl_target_interfaces(${PROJECT_NAME}_standalone
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(${PROJECT_NAME}_native
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_standalone ${rclcpp_LIBRARIES} "${cpp_typesupport_target}")
target_link_libraries(${PROJECT_NAME}_native ${rclcpp_LIBRARIES} "${cpp_typesupport_target}")
endif()

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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
