cmake_minimum_required(VERSION 3.8)
project(planning_scene_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddCollisionObject.srv"
)

add_executable(add_collision_object_service src/add_collision_object_service.cpp)

ament_target_dependencies(add_collision_object_service
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometry_msgs
  rosidl_default_runtime
)

target_include_directories(add_collision_object_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

# Link generated service headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(add_collision_object_service ${cpp_typesupport_target})

install(TARGETS 
  add_collision_object_service
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY srv
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
