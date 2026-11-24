cmake_minimum_required(VERSION 3.16)
project(voxblox)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-sign-compare)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(minkindr REQUIRED)
find_package(Protobuf REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Threads REQUIRED)

set(PROTO_DEFNS
  proto/voxblox/Block.proto
  proto/voxblox/Layer.proto
)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

add_library(${PROJECT_NAME}_proto SHARED ${PROTO_SRCS})
target_include_directories(${PROJECT_NAME}_proto PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_proto PUBLIC ${Protobuf_LIBRARIES})

add_library(${PROJECT_NAME} SHARED
  src/alignment/icp.cc
  src/core/block.cc
  src/core/esdf_map.cc
  src/core/tsdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/integrator/integrator_utils.cc
  src/integrator/intensity_integrator.cc
  src/integrator/tsdf_integrator.cc
  src/io/mesh_ply.cc
  src/io/sdf_ply.cc
  src/mesh/marching_cubes.cc
  src/simulation/objects.cc
  src/simulation/simulation_world.cc
  src/utils/camera_model.cc
  src/utils/evaluation_utils.cc
  src/utils/layer_utils.cc
  src/utils/neighbor_tools.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  src/utils/voxel_utils.cc
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_proto
  minkindr::minkindr
  Eigen3::Eigen
  glog::glog
  gflags
  Threads::Threads
)

install(TARGETS ${PROJECT_NAME}_proto ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES ${PROTO_HDRS}
  DESTINATION include/voxblox
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 minkindr Protobuf glog gflags)
ament_package()
