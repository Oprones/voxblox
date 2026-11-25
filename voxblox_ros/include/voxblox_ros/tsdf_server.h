#ifndef VOXBLOX_ROS_TSDF_SERVER_H_
#define VOXBLOX_ROS_TSDF_SERVER_H_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <queue>
#include <string>

#include <voxblox/alignment/icp.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/utils/color_maps.h>
#include <voxblox_msgs/msg/layer.hpp>
#include <voxblox_msgs/msg/mesh.hpp>
#include <voxblox_msgs/srv/file_path.hpp>

#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/transformer.h"

namespace voxblox {

constexpr float kDefaultMaxIntensity = 100.0f;

class TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit TsdfServer(rclcpp::Node* node);
  TsdfServer(rclcpp::Node* node, const TsdfMap::Config& config,
             const TsdfIntegratorBase::Config& integrator_config,
             const MeshIntegratorConfig& mesh_config);
  virtual ~TsdfServer() = default;

  void getServerConfigFromRosParam(rclcpp::Node* node);

  void insertPointcloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud);
  void insertFreespacePointcloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud);

  virtual void processPointCloudMessageAndInsert(
      const sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_msg,
      const Transformation& T_G_C, const bool is_freespace_pointcloud);

  void integratePointcloud(const Transformation& T_G_C,
                           const Pointcloud& ptcloud_C, const Colors& colors,
                           const bool is_freespace_pointcloud = false);
  virtual void newPoseCallback(const Transformation& /*new_pose*/) {}

  void publishAllUpdatedTsdfVoxels();
  void publishTsdfSurfacePoints();
  void publishTsdfOccupiedNodes();

  virtual void publishSlices();
  virtual void updateMesh();
  virtual bool generateMesh();
  virtual void publishPointclouds();
  virtual void publishMap(bool reset_remote_map = false);
  virtual bool saveMap(const std::string& file_path);
  virtual bool loadMap(const std::string& file_path);

  void clearMapCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void saveMapCallback(
      const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,
      std::shared_ptr<voxblox_msgs::srv::FilePath::Response> response);
  void loadMapCallback(
      const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,
      std::shared_ptr<voxblox_msgs::srv::FilePath::Response> response);
  void generateMeshCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void publishPointcloudsCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void publishTsdfMapCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void updateMeshEvent();
  void publishMapEvent();

  std::shared_ptr<TsdfMap> getTsdfMapPtr() { return tsdf_map_; }

  double getSliceLevel() const { return slice_level_; }
  void setSliceLevel(double slice_level) { slice_level_ = slice_level; }

  bool setPublishSlices() const { return publish_slices_; }
  void setPublishSlices(const bool publish_slices) {
    publish_slices_ = publish_slices;
  }

  void setWorldFrame(const std::string& world_frame) {
    world_frame_ = world_frame;
  }
  std::string getWorldFrame() const { return world_frame_; }

  virtual void clear();

  void tsdfMapCallback(const voxblox_msgs::msg::Layer::SharedPtr layer_msg);

 protected:
  bool getNextPointcloudFromQueue(
      std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>* queue,
      sensor_msgs::msg::PointCloud2::SharedPtr* pointcloud_msg,
      Transformation* T_G_C);

  rclcpp::Node* node_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      freespace_pointcloud_sub_;

  rclcpp::Publisher<voxblox_msgs::msg::Mesh>::SharedPtr mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      tsdf_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      surface_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tsdf_slice_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      occupancy_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr
      icp_transform_pub_;

  rclcpp::Publisher<voxblox_msgs::msg::Layer>::SharedPtr tsdf_map_pub_;
  rclcpp::Subscription<voxblox_msgs::msg::Layer>::SharedPtr tsdf_map_sub_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr generate_mesh_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_map_srv_;
  rclcpp::Service<voxblox_msgs::srv::FilePath>::SharedPtr save_map_srv_;
  rclcpp::Service<voxblox_msgs::srv::FilePath>::SharedPtr load_map_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_pointclouds_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_tsdf_map_srv_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr update_mesh_timer_;
  rclcpp::TimerBase::SharedPtr publish_map_timer_;

  bool verbose_;
  std::string world_frame_;
  std::string icp_corrected_frame_;
  std::string pose_corrected_frame_;

  double max_block_distance_from_body_;
  double slice_level_;
  bool use_freespace_pointcloud_;
  std::string mesh_filename_;
  ColorMode color_mode_;
  std::shared_ptr<ColorMap> color_map_;
  rclcpp::Duration min_time_between_msgs_;

  bool publish_pointclouds_on_update_;
  bool publish_slices_;
  bool publish_pointclouds_;
  bool publish_tsdf_map_;
  bool cache_mesh_;
  bool enable_icp_;
  bool accumulate_icp_corrections_;

  FloatingPoint occupancy_min_distance_voxel_size_factor_;
  int pointcloud_queue_size_;
  size_t num_subscribers_tsdf_map_;

  std::shared_ptr<TsdfMap> tsdf_map_;
  std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;
  std::shared_ptr<ICP> icp_;

  std::shared_ptr<MeshLayer> mesh_layer_;
  std::unique_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;
  voxblox_msgs::msg::Mesh cached_mesh_msg_;

  Transformer transformer_;
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointcloud_queue_;
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>
      freespace_pointcloud_queue_;

  rclcpp::Time last_msg_time_ptcloud_;
  rclcpp::Time last_msg_time_freespace_ptcloud_;
  Transformation icp_corrected_transform_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_TSDF_SERVER_H_
