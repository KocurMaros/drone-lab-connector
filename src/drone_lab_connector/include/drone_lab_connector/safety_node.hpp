#pragma once

#include <Eigen/Dense>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "drone_lab_connector/config_manager.hpp"

class SafetyNode : public rclcpp::Node {
 public:
  using LogCallback = std::function<void(const std::string&)>;

  explicit SafetyNode(std::shared_ptr<ConfigManager> config);

  void activateDrone(int drone_id);
  void deactivateDrone(int drone_id);
  bool isDroneActive(int drone_id) const;

  /// Refresh cached transforms & boundaries from ConfigManager.
  void reloadConfig();

  void setLogCallback(LogCallback cb);

 private:
  struct DroneHandles {
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr student_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    mavros_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr              error_pub;
  };

  void onStudentSetpoint(int drone_id,
                         const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Coordinate math
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);
  Eigen::Vector3d transformPosition(const Eigen::Matrix4d& tf,
                                    const Eigen::Vector3d& pos) const;
  double transformYaw(const Eigen::Matrix4d& tf, double yaw) const;
  bool   isWithinBounds(const Eigen::Vector3d& pos) const;

  void log(const std::string& msg);

  std::shared_ptr<ConfigManager> config_mgr_;

  // Cached copies (updated via reloadConfig)
  Eigen::Matrix4d student_to_real_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d real_to_drone_   = Eigen::Matrix4d::Identity();
  Boundaries      bounds_;

  std::map<int, DroneHandles> handles_;
  mutable std::recursive_mutex mtx_;
  LogCallback                 log_cb_;
};
