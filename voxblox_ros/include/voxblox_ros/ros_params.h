#ifndef VOXBLOX_ROS_ROS_PARAMS_H_
#define VOXBLOX_ROS_ROS_PARAMS_H_

#include <rclcpp/rclcpp.hpp>

#include <voxblox/alignment/icp.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>

namespace voxblox {

template <typename T>
inline T declareAndGetParam(rclcpp::Node* node, const std::string& name,
                            const T& default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_value);
  }
  T value = default_value;
  node->get_parameter(name, value);
  return value;
}

inline float declareAndGetFloatParam(rclcpp::Node* node,
                                     const std::string& name,
                                     float default_value) {
  return static_cast<float>(
      declareAndGetParam<double>(node, name, static_cast<double>(default_value)));
}

inline TsdfMap::Config getTsdfMapConfigFromRosParam(rclcpp::Node* node) {
  TsdfMap::Config tsdf_config;

  double voxel_size =
      declareAndGetParam<double>(node, "tsdf_voxel_size", tsdf_config.tsdf_voxel_size);
  int voxels_per_side = static_cast<int>(declareAndGetParam<int64_t>(
      node, "tsdf_voxels_per_side", tsdf_config.tsdf_voxels_per_side));
  if (!isPowerOfTwo(voxels_per_side)) {
    RCLCPP_ERROR(node->get_logger(),
                 "voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  }

  tsdf_config.tsdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
  tsdf_config.tsdf_voxels_per_side = voxels_per_side;
  return tsdf_config;
}

inline ICP::Config getICPConfigFromRosParam(rclcpp::Node* node) {
  ICP::Config icp_config;
  icp_config.min_match_ratio = declareAndGetFloatParam(
      node, "icp_min_match_ratio", icp_config.min_match_ratio);
  icp_config.subsample_keep_ratio = declareAndGetFloatParam(
      node, "icp_subsample_keep_ratio", icp_config.subsample_keep_ratio);
  icp_config.mini_batch_size = static_cast<size_t>(declareAndGetParam<int64_t>(
      node, "icp_mini_batch_size", static_cast<int64_t>(icp_config.mini_batch_size)));
  icp_config.refine_roll_pitch = declareAndGetParam<bool>(
      node, "icp_refine_roll_pitch", icp_config.refine_roll_pitch);
  icp_config.inital_translation_weighting = declareAndGetFloatParam(
      node, "icp_inital_translation_weighting",
      icp_config.inital_translation_weighting);
  icp_config.inital_rotation_weighting = declareAndGetFloatParam(
      node, "icp_inital_rotation_weighting", icp_config.inital_rotation_weighting);
  return icp_config;
}

inline TsdfIntegratorBase::Config getTsdfIntegratorConfigFromRosParam(
    rclcpp::Node* node) {
  TsdfIntegratorBase::Config integrator_config;
  integrator_config.voxel_carving_enabled = true;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(node);
  integrator_config.default_truncation_distance = tsdf_config.tsdf_voxel_size * 4;

  integrator_config.voxel_carving_enabled = declareAndGetParam<bool>(
      node, "voxel_carving_enabled", integrator_config.voxel_carving_enabled);
  integrator_config.default_truncation_distance = declareAndGetFloatParam(
      node, "truncation_distance", integrator_config.default_truncation_distance);
  integrator_config.max_ray_length_m = declareAndGetFloatParam(
      node, "max_ray_length_m", integrator_config.max_ray_length_m);
  integrator_config.min_ray_length_m = declareAndGetFloatParam(
      node, "min_ray_length_m", integrator_config.min_ray_length_m);
  integrator_config.max_weight =
      declareAndGetFloatParam(node, "max_weight", integrator_config.max_weight);
  integrator_config.use_const_weight = declareAndGetParam<bool>(
      node, "use_const_weight", integrator_config.use_const_weight);
  integrator_config.use_symmetric_weight_dropoff = declareAndGetParam<bool>(
      node, "use_symmetric_weight_dropoff",
      integrator_config.use_symmetric_weight_dropoff);
  integrator_config.use_weight_dropoff = declareAndGetParam<bool>(
      node, "use_weight_dropoff", integrator_config.use_weight_dropoff);
  integrator_config.allow_clear = declareAndGetParam<bool>(
      node, "allow_clear", integrator_config.allow_clear);
  integrator_config.start_voxel_subsampling_factor = declareAndGetFloatParam(
      node, "start_voxel_subsampling_factor",
      integrator_config.start_voxel_subsampling_factor);
  integrator_config.max_consecutive_ray_collisions =
      static_cast<int>(declareAndGetParam<int64_t>(
          node, "max_consecutive_ray_collisions",
          integrator_config.max_consecutive_ray_collisions));
  integrator_config.clear_checks_every_n_frames =
      static_cast<int>(declareAndGetParam<int64_t>(
          node, "clear_checks_every_n_frames",
          integrator_config.clear_checks_every_n_frames));
  integrator_config.max_integration_time_s = declareAndGetFloatParam(
      node, "max_integration_time_s", integrator_config.max_integration_time_s);
  integrator_config.enable_anti_grazing = declareAndGetParam<bool>(
      node, "anti_grazing", integrator_config.enable_anti_grazing);
  integrator_config.clearing_ray_weight_factor = declareAndGetFloatParam(
      node, "clearing_ray_weight_factor",
      integrator_config.clearing_ray_weight_factor);
  integrator_config.weight_ray_by_range = declareAndGetParam<bool>(
      node, "weight_ray_by_range", integrator_config.weight_ray_by_range);
  integrator_config.integration_order_mode = declareAndGetParam<std::string>(
      node, "integration_order_mode", integrator_config.integration_order_mode);

  return integrator_config;
}

inline EsdfMap::Config getEsdfMapConfigFromRosParam(rclcpp::Node* node) {
  EsdfMap::Config esdf_config;
  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(node);
  esdf_config.esdf_voxel_size = tsdf_config.tsdf_voxel_size;
  esdf_config.esdf_voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  return esdf_config;
}

inline EsdfIntegrator::Config getEsdfIntegratorConfigFromRosParam(
    rclcpp::Node* node) {
  EsdfIntegrator::Config esdf_integrator_config;
  TsdfIntegratorBase::Config tsdf_integrator_config =
      getTsdfIntegratorConfigFromRosParam(node);

  esdf_integrator_config.min_distance_m =
      tsdf_integrator_config.default_truncation_distance / 2.0f;

  esdf_integrator_config.full_euclidean_distance = declareAndGetParam<bool>(
      node, "esdf_euclidean_distance",
      esdf_integrator_config.full_euclidean_distance);
  esdf_integrator_config.max_distance_m = declareAndGetFloatParam(
      node, "esdf_max_distance_m", esdf_integrator_config.max_distance_m);
  esdf_integrator_config.min_distance_m = declareAndGetFloatParam(
      node, "esdf_min_distance_m", esdf_integrator_config.min_distance_m);
  esdf_integrator_config.default_distance_m = declareAndGetFloatParam(
      node, "esdf_default_distance_m",
      esdf_integrator_config.default_distance_m);
  esdf_integrator_config.min_diff_m = declareAndGetFloatParam(
      node, "esdf_min_diff_m", esdf_integrator_config.min_diff_m);
  esdf_integrator_config.clear_sphere_radius = declareAndGetFloatParam(
      node, "clear_sphere_radius", esdf_integrator_config.clear_sphere_radius);
  esdf_integrator_config.occupied_sphere_radius = declareAndGetFloatParam(
      node, "occupied_sphere_radius",
      esdf_integrator_config.occupied_sphere_radius);
  esdf_integrator_config.add_occupied_crust = declareAndGetParam<bool>(
      node, "esdf_add_occupied_crust",
      esdf_integrator_config.add_occupied_crust);

  if (esdf_integrator_config.default_distance_m <
      esdf_integrator_config.max_distance_m) {
    esdf_integrator_config.default_distance_m =
        esdf_integrator_config.max_distance_m;
  }
  return esdf_integrator_config;
}

inline MeshIntegratorConfig getMeshIntegratorConfigFromRosParam(
    rclcpp::Node* node) {
  MeshIntegratorConfig mesh_integrator_config;
  mesh_integrator_config.min_weight = declareAndGetFloatParam(
      node, "mesh_min_weight", mesh_integrator_config.min_weight);
  mesh_integrator_config.use_color = declareAndGetParam<bool>(
      node, "mesh_use_color", mesh_integrator_config.use_color);
  return mesh_integrator_config;
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_ROS_PARAMS_H_
