#include "voxblox_ros/tsdf_server.h"
#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ros_params.h"

#include <chrono>

namespace voxblox {

TsdfServer::TsdfServer(rclcpp::Node* node)
    : TsdfServer(node, getTsdfMapConfigFromRosParam(node),
                 getTsdfIntegratorConfigFromRosParam(node),
                 getMeshIntegratorConfigFromRosParam(node)) {}

TsdfServer::TsdfServer(rclcpp::Node* node, const TsdfMap::Config& config,
                       const TsdfIntegratorBase::Config& integrator_config,
                       const MeshIntegratorConfig& mesh_config)
    : node_(CHECK_NOTNULL(node)),
      tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(node_)),
      verbose_(true),
      world_frame_("world"),
      icp_corrected_frame_("icp_corrected"),
      pose_corrected_frame_("pose_corrected"),
      max_block_distance_from_body_(std::numeric_limits<FloatingPoint>::max()),
      slice_level_(0.5),
      use_freespace_pointcloud_(false),
      color_map_(new RainbowColorMap()),
      min_time_between_msgs_(0, 0),
      publish_pointclouds_on_update_(false),
      publish_slices_(false),
      publish_pointclouds_(false),
      publish_tsdf_map_(false),
      cache_mesh_(false),
      enable_icp_(false),
      accumulate_icp_corrections_(true),
      occupancy_min_distance_voxel_size_factor_(1.0f),
      pointcloud_queue_size_(1),
      num_subscribers_tsdf_map_(0),
      transformer_(node_),
      last_msg_time_ptcloud_(0, 0, RCL_ROS_TIME),
      last_msg_time_freespace_ptcloud_(0, 0, RCL_ROS_TIME) {
  getServerConfigFromRosParam(node_);

  rclcpp::QoS latched_qos(1);
  latched_qos.transient_local();

  surface_pointcloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "~/surface_pointcloud", latched_qos);
  tsdf_pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/tsdf_pointcloud", latched_qos);
  occupancy_marker_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "~/occupied_nodes", latched_qos);
  tsdf_slice_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/tsdf_slice", latched_qos);

  pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", pointcloud_queue_size_,
      std::bind(&TsdfServer::insertPointcloud, this, std::placeholders::_1));

  mesh_pub_ =
      node_->create_publisher<voxblox_msgs::msg::Mesh>("~/mesh", latched_qos);

  tsdf_map_pub_ =
      node_->create_publisher<voxblox_msgs::msg::Layer>("~/tsdf_map_out", 1);
  tsdf_map_sub_ = node_->create_subscription<voxblox_msgs::msg::Layer>(
      "~/tsdf_map_in", 1,
      std::bind(&TsdfServer::tsdfMapCallback, this, std::placeholders::_1));
  publish_tsdf_map_ =
      declareAndGetParam<bool>(node_, "publish_tsdf_map", publish_tsdf_map_);

  if (use_freespace_pointcloud_) {
    freespace_pointcloud_sub_ =
        node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "freespace_pointcloud", pointcloud_queue_size_,
            std::bind(&TsdfServer::insertFreespacePointcloud, this,
                      std::placeholders::_1));
  }

  if (enable_icp_) {
    icp_transform_pub_ =
        node_->create_publisher<geometry_msgs::msg::TransformStamped>(
            "~/icp_transform", latched_qos);
    icp_corrected_frame_ = declareAndGetParam<std::string>(
        node_, "icp_corrected_frame", icp_corrected_frame_);
    pose_corrected_frame_ = declareAndGetParam<std::string>(
        node_, "pose_corrected_frame", pose_corrected_frame_);
  }

  tsdf_map_.reset(new TsdfMap(config));

  std::string method =
      declareAndGetParam<std::string>(node_, "method", "merged");
  if (method == "simple") {
    tsdf_integrator_.reset(new SimpleTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method == "merged") {
    tsdf_integrator_.reset(new MergedTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method == "fast") {
    tsdf_integrator_.reset(new FastTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else {
    tsdf_integrator_.reset(new SimpleTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  }

  mesh_layer_.reset(new MeshLayer(tsdf_map_->block_size()));
  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));
  icp_.reset(new ICP(getICPConfigFromRosParam(node_)));

  generate_mesh_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "~/generate_mesh",
      std::bind(&TsdfServer::generateMeshCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  clear_map_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "~/clear_map",
      std::bind(&TsdfServer::clearMapCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  save_map_srv_ = node_->create_service<voxblox_msgs::srv::FilePath>(
      "~/save_map",
      std::bind(&TsdfServer::saveMapCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  load_map_srv_ = node_->create_service<voxblox_msgs::srv::FilePath>(
      "~/load_map",
      std::bind(&TsdfServer::loadMapCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  publish_pointclouds_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "~/publish_pointclouds",
      std::bind(&TsdfServer::publishPointcloudsCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  publish_tsdf_map_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "~/publish_map",
      std::bind(&TsdfServer::publishTsdfMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  double update_mesh_every_n_sec =
      declareAndGetParam<double>(node_, "update_mesh_every_n_sec", 1.0);
  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(update_mesh_every_n_sec),
        std::bind(&TsdfServer::updateMeshEvent, this));
  }

  double publish_map_every_n_sec =
      declareAndGetParam<double>(node_, "publish_map_every_n_sec", 1.0);
  if (publish_map_every_n_sec > 0.0) {
    publish_map_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(publish_map_every_n_sec),
        std::bind(&TsdfServer::publishMapEvent, this));
  }
}

void TsdfServer::getServerConfigFromRosParam(rclcpp::Node* node) {
  double min_time_between_msgs_sec =
      declareAndGetParam<double>(node, "min_time_between_msgs_sec", 0.0);
  min_time_between_msgs_ =
      rclcpp::Duration::from_seconds(min_time_between_msgs_sec);

  max_block_distance_from_body_ = declareAndGetParam<double>(
      node, "max_block_distance_from_body", max_block_distance_from_body_);
  slice_level_ = declareAndGetParam<double>(node, "slice_level", slice_level_);
  world_frame_ =
      declareAndGetParam<std::string>(node, "world_frame", world_frame_);
  publish_pointclouds_on_update_ = declareAndGetParam<bool>(
      node, "publish_pointclouds_on_update", publish_pointclouds_on_update_);
  publish_slices_ =
      declareAndGetParam<bool>(node, "publish_slices", publish_slices_);
  publish_pointclouds_ = declareAndGetParam<bool>(node, "publish_pointclouds",
                                                  publish_pointclouds_);
  use_freespace_pointcloud_ = declareAndGetParam<bool>(
      node, "use_freespace_pointcloud", use_freespace_pointcloud_);
  pointcloud_queue_size_ = static_cast<int>(declareAndGetParam<int64_t>(
      node, "pointcloud_queue_size", pointcloud_queue_size_));
  enable_icp_ = declareAndGetParam<bool>(node, "enable_icp", enable_icp_);
  accumulate_icp_corrections_ = declareAndGetParam<bool>(
      node, "accumulate_icp_corrections", accumulate_icp_corrections_);
  occupancy_min_distance_voxel_size_factor_ = declareAndGetFloatParam(
      node, "occupancy_min_distance_voxel_size_factor",
      occupancy_min_distance_voxel_size_factor_);
  verbose_ = declareAndGetParam<bool>(node, "verbose", verbose_);

  mesh_filename_ =
      declareAndGetParam<std::string>(node, "mesh_filename", mesh_filename_);
  std::string color_mode =
      declareAndGetParam<std::string>(node, "color_mode", "");
  color_mode_ = getColorModeFromString(color_mode);

  std::string intensity_colormap =
      declareAndGetParam<std::string>(node, "intensity_colormap", "rainbow");
  float intensity_max_value = declareAndGetFloatParam(
      node, "intensity_max_value", kDefaultMaxIntensity);

  if (intensity_colormap == "rainbow") {
    color_map_.reset(new RainbowColorMap());
  } else if (intensity_colormap == "inverse_rainbow") {
    color_map_.reset(new InverseRainbowColorMap());
  } else if (intensity_colormap == "grayscale") {
    color_map_.reset(new GrayscaleColorMap());
  } else if (intensity_colormap == "inverse_grayscale") {
    color_map_.reset(new InverseGrayscaleColorMap());
  } else if (intensity_colormap == "ironbow") {
    color_map_.reset(new IronbowColorMap());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Invalid color map: %s",
                 intensity_colormap.c_str());
  }
  color_map_->setMaxValue(intensity_max_value);
}

void TsdfServer::processPointCloudMessageAndInsert(
    const sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_msg,
    const Transformation& T_G_C, const bool is_freespace_pointcloud) {
  bool color_pointcloud = false;
  bool has_intensity = false;
  for (size_t d = 0; d < pointcloud_msg->fields.size(); ++d) {
    if (pointcloud_msg->fields[d].name == "rgb") {
      pointcloud_msg->fields[d].datatype =
          sensor_msgs::msg::PointField::FLOAT32;
      color_pointcloud = true;
    } else if (pointcloud_msg->fields[d].name == "intensity") {
      has_intensity = true;
    }
  }

  Pointcloud points_C;
  Colors colors;
  timing::Timer ptcloud_timer("ptcloud_preprocess");

  if (color_pointcloud) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else if (has_intensity) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  }
  ptcloud_timer.Stop();

  Transformation T_G_C_refined = T_G_C;
  if (enable_icp_) {
    timing::Timer icp_timer("icp");
    if (!accumulate_icp_corrections_) {
      icp_corrected_transform_.setIdentity();
    }
    const size_t num_icp_updates =
        icp_->runICP(tsdf_map_->getTsdfLayer(), points_C,
                     icp_corrected_transform_ * T_G_C, &T_G_C_refined);
    if (verbose_) {
      RCLCPP_INFO(node_->get_logger(),
                  "ICP refinement performed %zu successful update steps",
                  num_icp_updates);
    }
    icp_corrected_transform_ = T_G_C_refined * T_G_C.inverse();

    if (!icp_->refiningRollPitch()) {
      Transformation::Vector6 T_vec = icp_corrected_transform_.log();
      T_vec[3] = 0.0;
      T_vec[4] = 0.0;
      icp_corrected_transform_ = Transformation::exp(T_vec);
    }

    geometry_msgs::msg::TransformStamped icp_tf_msg;
    icp_tf_msg.header.stamp = pointcloud_msg->header.stamp;
    icp_tf_msg.header.frame_id = world_frame_;
    icp_tf_msg.child_frame_id = icp_corrected_frame_;
    transformKindrToMsg(icp_corrected_transform_, &icp_tf_msg.transform);
    tf_broadcaster_->sendTransform(icp_tf_msg);

    geometry_msgs::msg::TransformStamped pose_tf_msg;
    pose_tf_msg.header.stamp = pointcloud_msg->header.stamp;
    pose_tf_msg.header.frame_id = icp_corrected_frame_;
    pose_tf_msg.child_frame_id = pose_corrected_frame_;
    transformKindrToMsg(T_G_C, &pose_tf_msg.transform);
    tf_broadcaster_->sendTransform(pose_tf_msg);

    icp_transform_pub_->publish(icp_tf_msg);
    icp_timer.Stop();
  }

  if (verbose_) {
    RCLCPP_INFO(node_->get_logger(), "Integrating a pointcloud with %zu points.",
                points_C.size());
  }

  auto start = std::chrono::steady_clock::now();
  integratePointcloud(T_G_C_refined, points_C, colors, is_freespace_pointcloud);
  auto end = std::chrono::steady_clock::now();
  if (verbose_) {
    std::chrono::duration<double> elapsed = end - start;
    RCLCPP_INFO(node_->get_logger(),
                "Finished integrating in %f seconds, have %zu blocks.",
                elapsed.count(),
                tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
  }

  timing::Timer block_remove_timer("remove_distant_blocks");
  tsdf_map_->getTsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  mesh_layer_->clearDistantMesh(T_G_C.getPosition(),
                                max_block_distance_from_body_);
  block_remove_timer.Stop();

  newPoseCallback(T_G_C);
}

bool TsdfServer::getNextPointcloudFromQueue(
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>* queue,
    sensor_msgs::msg::PointCloud2::SharedPtr* pointcloud_msg,
    Transformation* T_G_C) {
  constexpr size_t kMaxQueueSize = 10;
  if (queue->empty()) {
    return false;
  }
  *pointcloud_msg = queue->front();
  if (transformer_.lookupTransform((*pointcloud_msg)->header.frame_id,
                                   world_frame_,
                                   rclcpp::Time((*pointcloud_msg)->header.stamp),
                                   T_G_C)) {
    queue->pop();
    return true;
  }
  if (queue->size() >= kMaxQueueSize) {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 60000,
        "Input pointcloud queue getting too long! Dropping some pointclouds.");
    while (queue->size() >= kMaxQueueSize) {
      queue->pop();
    }
  }
  return false;
}

void TsdfServer::insertPointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg_in) {
  rclcpp::Time stamp(pointcloud_msg_in->header.stamp);
  if (stamp - last_msg_time_ptcloud_ > min_time_between_msgs_) {
    last_msg_time_ptcloud_ = stamp;
    pointcloud_queue_.push(pointcloud_msg_in);
  }

  Transformation T_G_C;
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg;
  bool processed_any = false;
  while (
      getNextPointcloudFromQueue(&pointcloud_queue_, &pointcloud_msg, &T_G_C)) {
    processPointCloudMessageAndInsert(pointcloud_msg, T_G_C, false);
    processed_any = true;
  }

  if (!processed_any) {
    return;
  }

  if (publish_pointclouds_on_update_) {
    publishPointclouds();
  }

  if (verbose_) {
    RCLCPP_INFO(node_->get_logger(), "Timings:\n%s",
                timing::Timing::Print().c_str());
    RCLCPP_INFO(node_->get_logger(), "Layer memory: %zu",
                tsdf_map_->getTsdfLayer().getMemorySize());
  }
}

void TsdfServer::insertFreespacePointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg_in) {
  rclcpp::Time stamp(pointcloud_msg_in->header.stamp);
  if (stamp - last_msg_time_freespace_ptcloud_ > min_time_between_msgs_) {
    last_msg_time_freespace_ptcloud_ = stamp;
    freespace_pointcloud_queue_.push(pointcloud_msg_in);
  }

  Transformation T_G_C;
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg;
  while (getNextPointcloudFromQueue(&freespace_pointcloud_queue_,
                                    &pointcloud_msg, &T_G_C)) {
    processPointCloudMessageAndInsert(pointcloud_msg, T_G_C, true);
  }
}

void TsdfServer::integratePointcloud(const Transformation& T_G_C,
                                     const Pointcloud& ptcloud_C,
                                     const Colors& colors,
                                     const bool is_freespace_pointcloud) {
  CHECK_EQ(ptcloud_C.size(), colors.size());
  tsdf_integrator_->integratePointCloud(T_G_C, ptcloud_C, colors,
                                        is_freespace_pointcloud);
}

void TsdfServer::publishAllUpdatedTsdfVoxels() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  createDistancePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(), &pointcloud);
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pointcloud, msg);
  msg.header.frame_id = world_frame_;
  msg.header.stamp = node_->now();
  tsdf_pointcloud_pub_->publish(msg);
}

void TsdfServer::publishTsdfSurfacePoints() {
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  const float surface_distance_thresh =
      tsdf_map_->getTsdfLayer().voxel_size() * 0.75f;
  createSurfacePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(),
                                       surface_distance_thresh, &pointcloud);
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pointcloud, msg);
  msg.header.frame_id = world_frame_;
  msg.header.stamp = node_->now();
  surface_pointcloud_pub_->publish(msg);
}

void TsdfServer::publishTsdfOccupiedNodes() {
  visualization_msgs::msg::MarkerArray marker_array;
  createOccupancyBlocksFromTsdfLayer(
      tsdf_map_->getTsdfLayer(), world_frame_,
      tsdf_map_->getTsdfLayer().voxel_size() *
          occupancy_min_distance_voxel_size_factor_,
      &marker_array);
  occupancy_marker_pub_->publish(marker_array);
}

void TsdfServer::publishSlices() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  createDistancePointcloudFromTsdfLayerSlice(tsdf_map_->getTsdfLayer(), 2,
                                             slice_level_, &pointcloud);
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pointcloud, msg);
  msg.header.frame_id = world_frame_;
  msg.header.stamp = node_->now();
  tsdf_slice_pub_->publish(msg);
}

void TsdfServer::publishMap(bool reset_remote_map) {
  if (!publish_tsdf_map_) {
    return;
  }
  size_t subscribers = tsdf_map_pub_->get_subscription_count();
  if (subscribers > 0) {
    if (num_subscribers_tsdf_map_ < subscribers) {
      reset_remote_map = true;
      RCLCPP_WARN(node_->get_logger(),
                  "New subscriber to tsdf_map_, resetting remote map.");
    }
    const bool only_updated = !reset_remote_map;
    timing::Timer publish_map_timer("map/publish_tsdf");
    voxblox_msgs::msg::Layer layer_msg;
    serializeLayerAsMsg<TsdfVoxel>(tsdf_map_->getTsdfLayer(), only_updated,
                                   &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    tsdf_map_pub_->publish(layer_msg);
    publish_map_timer.Stop();
  }
  num_subscribers_tsdf_map_ = subscribers;
}

void TsdfServer::publishPointclouds() {
  publishAllUpdatedTsdfVoxels();
  publishTsdfSurfacePoints();
  publishTsdfOccupiedNodes();
  if (publish_slices_) {
    publishSlices();
  }
}

void TsdfServer::updateMesh() {
  if (verbose_) {
    RCLCPP_INFO(node_->get_logger(), "Updating mesh.");
  }
  timing::Timer generate_mesh_timer("mesh/update");
  mesh_integrator_->generateMesh(true, true);
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");
  voxblox_msgs::msg::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_msg.header.stamp = node_->now();
  mesh_pub_->publish(mesh_msg);

  if (cache_mesh_) {
    cached_mesh_msg_ = mesh_msg;
  }
  publish_mesh_timer.Stop();

  if (publish_pointclouds_ && !publish_pointclouds_on_update_) {
    publishPointclouds();
  }
}

bool TsdfServer::generateMesh() {
  timing::Timer generate_mesh_timer("mesh/generate");
  mesh_integrator_->generateMesh(false, true);
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");
  voxblox_msgs::msg::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_msg.header.stamp = node_->now();
  mesh_pub_->publish(mesh_msg);
  publish_mesh_timer.Stop();

  if (!mesh_filename_.empty()) {
    timing::Timer output_mesh_timer("mesh/output");
    const bool success = outputMeshLayerAsPly(mesh_filename_, *mesh_layer_);
    output_mesh_timer.Stop();
    RCLCPP_INFO(node_->get_logger(),
                success ? "Output file as PLY: %s"
                        : "Failed to output mesh as PLY: %s",
                mesh_filename_.c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "Mesh Timings:\n%s",
              timing::Timing::Print().c_str());
  return true;
}

bool TsdfServer::saveMap(const std::string& file_path) {
  return io::SaveLayer(tsdf_map_->getTsdfLayer(), file_path);
}

bool TsdfServer::loadMap(const std::string& file_path) {
  constexpr bool kMultipleLayerSupport = true;
  bool success = io::LoadBlocksFromFile(
      file_path, Layer<TsdfVoxel>::BlockMergingStrategy::kReplace,
      kMultipleLayerSupport, tsdf_map_->getTsdfLayerPtr());
  if (success) {
    LOG(INFO) << "Successfully loaded TSDF layer.";
  }
  return success;
}

void TsdfServer::clearMapCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  clear();
}

void TsdfServer::generateMeshCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  generateMesh();
}

void TsdfServer::saveMapCallback(
    const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<voxblox_msgs::srv::FilePath::Response>) {
  saveMap(request->file_path);
}

void TsdfServer::loadMapCallback(
    const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<voxblox_msgs::srv::FilePath::Response>) {
  loadMap(request->file_path);
}

void TsdfServer::publishPointcloudsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  publishPointclouds();
}

void TsdfServer::publishTsdfMapCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  publishMap();
}

void TsdfServer::updateMeshEvent() { updateMesh(); }
void TsdfServer::publishMapEvent() { publishMap(); }

void TsdfServer::clear() {
  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  mesh_layer_->clear();
  if (publish_tsdf_map_) {
    publishMap(true);
  }
}

void TsdfServer::tsdfMapCallback(
    const voxblox_msgs::msg::Layer::SharedPtr layer_msg) {
  timing::Timer receive_map_timer("map/receive_tsdf");
  bool success = deserializeMsgToLayer<TsdfVoxel>(*layer_msg,
                                                  tsdf_map_->getTsdfLayerPtr());
  if (!success) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                          "Got an invalid TSDF map message!");
  } else {
    RCLCPP_INFO_ONCE(node_->get_logger(), "Got a TSDF map from ROS topic!");
    if (publish_pointclouds_on_update_) {
      publishPointclouds();
    }
  }
}

}  // namespace voxblox
