cmake_minimum_required(VERSION 3.16)
project(voxblox_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-sign-compare)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED)
find_package(minkindr REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/transformer.cc
  src/tsdf_server.cc
  src/esdf_server.cc
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_conversions
  minkindr
  voxblox
  voxblox_msgs
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  glog::glog
  gflags
)

add_executable(tsdf_server src/tsdf_server_node.cc)
target_link_libraries(tsdf_server ${PROJECT_NAME})

add_executable(esdf_server src/esdf_server_node.cc)
target_link_libraries(esdf_server ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS tsdf_server esdf_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp sensor_msgs std_msgs std_srvs geometry_msgs visualization_msgs
  tf2 tf2_ros pcl_conversions minkindr voxblox voxblox_msgs glog gflags
)
ament_package()
