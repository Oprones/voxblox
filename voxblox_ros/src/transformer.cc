#include "voxblox_ros/transformer.h"
#include "voxblox_ros/ros_params.h"

#include <tf2/exceptions.h>

namespace voxblox {

namespace {
bool vectorParamToKindr(const std::vector<double>& v, Transformation* out) {
  if (v.size() == 16) {
    Eigen::Matrix4f m;
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        m(r, c) = static_cast<float>(v[r * 4 + c]);
      }
    }
    *out = Transformation(m);
    return true;
  } else if (v.size() == 7) {
    *out = Transformation(
        Transformation::Rotation(static_cast<FloatingPoint>(v[6]),
                                 static_cast<FloatingPoint>(v[3]),
                                 static_cast<FloatingPoint>(v[4]),
                                 static_cast<FloatingPoint>(v[5])),
        Transformation::Position(static_cast<FloatingPoint>(v[0]),
                                 static_cast<FloatingPoint>(v[1]),
                                 static_cast<FloatingPoint>(v[2])));
    return true;
  }
  return false;
}
}  // namespace

Transformer::Transformer(rclcpp::Node* node)
    : node_(CHECK_NOTNULL(node)),
      world_frame_("world"),
      sensor_frame_(""),
      use_tf_transforms_(true),
      timestamp_tolerance_ns_(1000000),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
  world_frame_ = declareAndGetParam<std::string>(node_, "world_frame", world_frame_);
  sensor_frame_ = declareAndGetParam<std::string>(node_, "sensor_frame", sensor_frame_);

  constexpr double kNanoSecondsInSecond = 1.0e9;
  double timestamp_tolerance_sec = timestamp_tolerance_ns_ / kNanoSecondsInSecond;
  timestamp_tolerance_sec = declareAndGetParam<double>(
      node_, "timestamp_tolerance_sec", timestamp_tolerance_sec);
  timestamp_tolerance_ns_ =
      static_cast<int64_t>(timestamp_tolerance_sec * kNanoSecondsInSecond);

  use_tf_transforms_ =
      declareAndGetParam<bool>(node_, "use_tf_transforms", use_tf_transforms_);

  if (!use_tf_transforms_) {
    transform_sub_ =
        node_->create_subscription<geometry_msgs::msg::TransformStamped>(
            "transform", 40,
            std::bind(&Transformer::transformCallback, this, std::placeholders::_1));

    auto T_B_D_vec =
        declareAndGetParam<std::vector<double>>(node_, "T_B_D", std::vector<double>{});
    if (vectorParamToKindr(T_B_D_vec, &T_B_D_)) {
      if (declareAndGetParam<bool>(node_, "invert_T_B_D", false)) {
        T_B_D_ = T_B_D_.inverse();
      }
    }
    auto T_B_C_vec =
        declareAndGetParam<std::vector<double>>(node_, "T_B_C", std::vector<double>{});
    if (vectorParamToKindr(T_B_C_vec, &T_B_C_)) {
      if (declareAndGetParam<bool>(node_, "invert_T_B_C", false)) {
        T_B_C_ = T_B_C_.inverse();
      }
    }
  }
}

void Transformer::transformCallback(
    const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg) {
  transform_queue_.push_back(*transform_msg);
}

bool Transformer::lookupTransform(const std::string& from_frame,
                                  const std::string& to_frame,
                                  const rclcpp::Time& timestamp,
                                  Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (use_tf_transforms_) {
    return lookupTransformTf(from_frame, to_frame, timestamp, transform);
  }
  return lookupTransformQueue(timestamp, transform);
}

bool Transformer::lookupTransformTf(const std::string& from_frame,
                                    const std::string& to_frame,
                                    const rclcpp::Time& timestamp,
                                    Transformation* transform) {
  CHECK_NOTNULL(transform);
  std::string from_frame_modified =
      sensor_frame_.empty() ? from_frame : sensor_frame_;

  if (!tf_buffer_->canTransform(to_frame, from_frame_modified, timestamp,
                                tf2::durationFromSec(0.0))) {
    return false;
  }

  geometry_msgs::msg::TransformStamped tf_transform;
  try {
    tf_transform = tf_buffer_->lookupTransform(to_frame, from_frame_modified,
                                               timestamp);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Error getting TF transform from sensor data: %s", ex.what());
    return false;
  }

  transformMsgToKindr(tf_transform.transform, transform);
  return true;
}

bool Transformer::lookupTransformQueue(const rclcpp::Time& timestamp,
                                       Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (transform_queue_.empty()) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 30000,
        "No match found for transform timestamp: %ld as transform queue is empty.",
        timestamp.nanoseconds());
    return false;
  }

  bool match_found = false;
  auto it = transform_queue_.begin();
  for (; it != transform_queue_.end(); ++it) {
    rclcpp::Time msg_time(it->header.stamp);
    if (msg_time > timestamp) {
      if ((msg_time - timestamp).nanoseconds() < timestamp_tolerance_ns_) {
        match_found = true;
      }
      break;
    }
    if ((timestamp - msg_time).nanoseconds() < timestamp_tolerance_ns_) {
      match_found = true;
      break;
    }
  }

  Transformation T_G_D;
  if (match_found) {
    transformMsgToKindr(it->transform, &T_G_D);
  } else {
    if (it == transform_queue_.begin() || it == transform_queue_.end()) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 30000,
          "No match found for transform timestamp: %ld", timestamp.nanoseconds());
      return false;
    }
    Transformation T_G_D_newest;
    transformMsgToKindr(it->transform, &T_G_D_newest);
    int64_t offset_newest_ns =
        (rclcpp::Time(it->header.stamp) - timestamp).nanoseconds();
    --it;
    Transformation T_G_D_oldest;
    transformMsgToKindr(it->transform, &T_G_D_oldest);
    int64_t offset_oldest_ns =
        (timestamp - rclcpp::Time(it->header.stamp)).nanoseconds();

    FloatingPoint t_diff_ratio =
        static_cast<FloatingPoint>(offset_oldest_ns) /
        static_cast<FloatingPoint>(offset_newest_ns + offset_oldest_ns);

    Transformation::Vector6 diff_vector =
        (T_G_D_oldest.inverse() * T_G_D_newest).log();
    T_G_D = T_G_D_oldest * Transformation::exp(t_diff_ratio * diff_vector);
  }

  *transform = T_G_D * T_B_D_.inverse() * T_B_C_;
  transform_queue_.erase(transform_queue_.begin(), it);
  return true;
}

}  // namespace voxblox
