cmake_minimum_required(VERSION 3.8)
project(nhatbot_stack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(static_tf_pub_node src/static_tf_pub.cpp)
ament_target_dependencies(static_tf_pub_node rclcpp tf2 tf2_ros geometry_msgs)
target_include_directories(static_tf_pub_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)    

add_executable(initial_pose_node src/initial_pose_pub.cpp)
ament_target_dependencies(initial_pose_node rclcpp  geometry_msgs)


install(TARGETS static_tf_pub_node
                initial_pose_node
  DESTINATION lib/${PROJECT_NAME})



  install(PROGRAMS
  ${PROJECT_NAME}/static_tf_publisher.py 
  DESTINATION lib/${PROJECT_NAME}
 )
 


install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
