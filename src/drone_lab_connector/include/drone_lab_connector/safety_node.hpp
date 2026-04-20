#pragma once

#include <Eigen/Dense>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
    // ── Setpoint validation (student → safety → MAVROS) ─────────────
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr student_setpoint_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    mavros_setpoint_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr              error_pub;

    // ── State tracking (subscribe to MAVROS native topics) ──────────
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    geometry_msgs::msg::PoseStamped::SharedPtr                       last_pose;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr         state_sub;
    std::string current_mode;
    bool is_armed = false;

    // ── Service proxies (student safe/* → safety → MAVROS) ──────────
    rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr  arm_srv;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr   arm_cli;
    rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr      mode_srv;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr       mode_cli;
    rclcpp::Service<mavros_msgs::srv::CommandTOL>::SharedPtr   takeoff_srv;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr    takeoff_cli;
    rclcpp::Service<mavros_msgs::srv::CommandTOL>::SharedPtr   land_srv;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr    land_cli;
  };

  void onStudentSetpoint(int drone_id,
                         const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Service proxy handlers
  void onArmRequest(int drone_id,
                    const mavros_msgs::srv::CommandBool::Request::SharedPtr req,
                    mavros_msgs::srv::CommandBool::Response::SharedPtr res);
  void onSetModeRequest(int drone_id,
                        const mavros_msgs::srv::SetMode::Request::SharedPtr req,
                        mavros_msgs::srv::SetMode::Response::SharedPtr res);
  void onTakeoffRequest(int drone_id,
                        const mavros_msgs::srv::CommandTOL::Request::SharedPtr req,
                        mavros_msgs::srv::CommandTOL::Response::SharedPtr res);
  void onLandRequest(int drone_id,
                     const mavros_msgs::srv::CommandTOL::Request::SharedPtr req,
                     mavros_msgs::srv::CommandTOL::Response::SharedPtr res);

  // Coordinate math
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);
  Eigen::Vector3d transformPosition(const Eigen::Matrix4d& tf,
                                    const Eigen::Vector3d& pos) const;
  double transformYaw(const Eigen::Matrix4d& tf, double yaw) const;
  bool   isWithinBounds(const Eigen::Vector3d& pos) const;

  void log(const std::string& msg);
  void publishMarkers();

  std::shared_ptr<ConfigManager> config_mgr_;

  // Cached copies (updated via reloadConfig)
  Eigen::Matrix4d student_to_real_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d real_to_drone_   = Eigen::Matrix4d::Identity();
  Boundaries      bounds_;

  std::map<int, DroneHandles> handles_;
  mutable std::recursive_mutex mtx_;
  LogCallback                 log_cb_;

  rclcpp::CallbackGroup::SharedPtr service_cbg_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
};
