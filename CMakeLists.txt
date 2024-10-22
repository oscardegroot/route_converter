cmake_minimum_required(VERSION 3.5)
project(route_converter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

## DEPENDENCIES ##
set(DEPENDENCIES
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  ros_tools
  autoware_route_handler
  autoware_planning_msgs
  autoware_map_msgs
)

find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/configuration.cpp  
  src/route_converter.cpp
)
target_precompile_headers(${PROJECT_NAME} PRIVATE include/route_converter/pch.h)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

add_executable(gps_gate src/gps_gate.cpp)
ament_target_dependencies(gps_gate ${DEPENDENCIES})
target_link_libraries(gps_gate ${PROJECT_NAME})

## INSTALL ##
install(
  TARGETS 
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  ${PROJECT_NAME}_node
  gps_gate
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()