#ifndef VOXBLOX_ROS_TRANSFORMER_H_
#define VOXBLOX_ROS_TRANSFORMER_H_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include <voxblox/core/common.h>

namespace voxblox {

inline void transformMsgToKindr(const geometry_msgs::msg::Transform& msg,
                                Transformation* kindr) {
  CHECK_NOTNULL(kindr);
  *kindr = Transformation(
      Transformation::Rotation(static_cast<FloatingPoint>(msg.rotation.w),
                               static_cast<FloatingPoint>(msg.rotation.x),
                               static_cast<FloatingPoint>(msg.rotation.y),
                               static_cast<FloatingPoint>(msg.rotation.z)),
      Transformation::Position(static_cast<FloatingPoint>(msg.translation.x),
                               static_cast<FloatingPoint>(msg.translation.y),
                               static_cast<FloatingPoint>(msg.translation.z)));
}

inline void transformKindrToMsg(const Transformation& kindr,
                                geometry_msgs::msg::Transform* msg) {
  CHECK_NOTNULL(msg);
  msg->translation.x = kindr.getPosition().x();
  msg->translation.y = kindr.getPosition().y();
  msg->translation.z = kindr.getPosition().z();
  msg->rotation.w = kindr.getRotation().w();
  msg->rotation.x = kindr.getRotation().x();
  msg->rotation.y = kindr.getRotation().y();
  msg->rotation.z = kindr.getRotation().z();
}

class Transformer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit Transformer(rclcpp::Node* node);

  bool lookupTransform(const std::string& from_frame,
                       const std::string& to_frame,
                       const rclcpp::Time& timestamp,
                       Transformation* transform);

  void transformCallback(
      const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg);

 private:
  bool lookupTransformTf(const std::string& from_frame,
                         const std::string& to_frame,
                         const rclcpp::Time& timestamp,
                         Transformation* transform);

  bool lookupTransformQueue(const rclcpp::Time& timestamp,
                            Transformation* transform);

  rclcpp::Node* node_;
  std::string world_frame_;
  std::string sensor_frame_;
  bool use_tf_transforms_;
  int64_t timestamp_tolerance_ns_;
  Transformation T_B_C_;
  Transformation T_B_D_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
      transform_sub_;
  AlignedDeque<geometry_msgs::msg::TransformStamped> transform_queue_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_TRANSFORMER_H_
