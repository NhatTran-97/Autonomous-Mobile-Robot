cmake_minimum_required(VERSION 3.8)
project(zlac8015d_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)


find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)
include_directories(
  include
  ${LIBMODBUS_INCLUDE_DIRS}
)


include_directories(/usr/include/modbus)


set(SOURCES
  src/zlac_interfaces.cpp
  src/zlac8015d_driver.cpp
)

add_executable(zlac_driver_node ${SOURCES})
target_link_libraries(zlac_driver_node
  ${LIBMODBUS_LIBRARIES}
)

ament_target_dependencies(zlac_driver_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

install(TARGETS zlac_driver_node
  DESTINATION lib/${PROJECT_NAME}
)




install(
  DIRECTORY  
  launch 
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS

 ${PROJECT_NAME}/zlac_interfacer.py 
 ${PROJECT_NAME}/zlac8015d_driver.py 

  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
