#include "drone_lab_connector/safety_node.hpp"

#include <cmath>
#include <sstream>

// ─── construction ───────────────────────────────────────────────────────────

SafetyNode::SafetyNode(std::shared_ptr<ConfigManager> config)
    : rclcpp::Node("drone_lab_safety"), config_mgr_(std::move(config)) {
  reloadConfig();
  RCLCPP_INFO(get_logger(), "SafetyNode initialised");
}

// ─── drone lifecycle ────────────────────────────────────────────────────────

void SafetyNode::activateDrone(int drone_id) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (handles_.count(drone_id)) return;  // already active

  const auto& cfg = config_mgr_->config().drones;
  const std::string ns = config_mgr_->droneNamespace(drone_id);

  DroneHandles h;

  // Student setpoint subscriber
  std::string sub_topic = ns + "/" + cfg.student_setpoint_topic;
  h.student_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
      sub_topic, rclcpp::SensorDataQoS(),
      [this, drone_id](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        onStudentSetpoint(drone_id, msg);
      });

  // MAVROS setpoint publisher
  std::string pub_topic = ns + "/" + cfg.mavros_setpoint_topic;
  h.mavros_pub = create_publisher<geometry_msgs::msg::PoseStamped>(pub_topic, 10);

  // Error string publisher
  std::string err_topic = ns + "/" + cfg.error_topic;
  h.error_pub = create_publisher<std_msgs::msg::String>(err_topic, 10);

  handles_[drone_id] = std::move(h);

  log("[SafetyNode] Activated drone EDU" + std::to_string(drone_id) +
      " | sub: " + sub_topic + " | pub: " + pub_topic);
}

void SafetyNode::deactivateDrone(int drone_id) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (auto it = handles_.find(drone_id); it != handles_.end()) {
    handles_.erase(it);
    log("[SafetyNode] Deactivated drone EDU" + std::to_string(drone_id));
  }
}

bool SafetyNode::isDroneActive(int drone_id) const {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  return handles_.count(drone_id) > 0;
}

void SafetyNode::reloadConfig() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  const auto& c  = config_mgr_->config();
  student_to_real_ = c.student_to_real;
  real_to_drone_   = c.real_to_drone;
  bounds_          = c.safety;
}

void SafetyNode::setLogCallback(LogCallback cb) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  log_cb_ = std::move(cb);
}

// ─── core callback ──────────────────────────────────────────────────────────

void SafetyNode::onStudentSetpoint(
    int drone_id,
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // Snapshot cached config under lock
  Eigen::Matrix4d s2r, r2d;
  Boundaries b;
  {
    std::lock_guard<std::recursive_mutex> lock(mtx_);
    s2r = student_to_real_;
    r2d = real_to_drone_;
    b   = bounds_;
  }

  // 1. Extract student position & yaw
  const Eigen::Vector3d student_pos(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  const double student_yaw = yawFromQuaternion(msg->pose.orientation);

  // 2. Student → Real-world
  const Eigen::Vector3d real_pos = transformPosition(s2r, student_pos);
  const double real_yaw          = transformYaw(s2r, student_yaw);

  // 3. Boundary check in real-world frame
  if (!isWithinBounds(real_pos)) {
    std::ostringstream ss;
    ss << "OUT OF BOUNDS  EDU" << drone_id << "  real=("
       << real_pos.x() << ", " << real_pos.y() << ", " << real_pos.z()
       << ")  limits X[" << b.x_min << "," << b.x_max
       << "] Y[" << b.y_min << "," << b.y_max
       << "] Z[" << b.z_min << "," << b.z_max << "]";
    const std::string err_str = ss.str();

    // Publish error
    {
      std::lock_guard<std::recursive_mutex> lock(mtx_);
      if (auto it = handles_.find(drone_id); it != handles_.end()) {
        std_msgs::msg::String err_msg;
        err_msg.data = err_str;
        it->second.error_pub->publish(err_msg);
      }
    }
    log("[BLOCKED] " + err_str);
    return;
  }

  // 4. Real-world → Drone frame
  const Eigen::Vector3d drone_pos = transformPosition(r2d, real_pos);
  const double drone_yaw          = transformYaw(r2d, real_yaw);

  // 5. Build & publish transformed PoseStamped
  geometry_msgs::msg::PoseStamped out;
  out.header.stamp    = now();
  out.header.frame_id = "map";
  out.pose.position.x = drone_pos.x();
  out.pose.position.y = drone_pos.y();
  out.pose.position.z = drone_pos.z();
  out.pose.orientation = quaternionFromYaw(drone_yaw);

  {
    std::lock_guard<std::recursive_mutex> lock(mtx_);
    if (auto it = handles_.find(drone_id); it != handles_.end()) {
      it->second.mavros_pub->publish(out);
    }
  }
}

// ─── math helpers ───────────────────────────────────────────────────────────

double SafetyNode::yawFromQuaternion(
    const geometry_msgs::msg::Quaternion& q) {
  // ZYX Euler yaw extraction
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion SafetyNode::quaternionFromYaw(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

Eigen::Vector3d SafetyNode::transformPosition(
    const Eigen::Matrix4d& tf, const Eigen::Vector3d& pos) const {
  Eigen::Vector4d h;
  h << pos, 1.0;
  Eigen::Vector4d out = tf * h;
  return out.head<3>();
}

double SafetyNode::transformYaw(const Eigen::Matrix4d& tf,
                                double yaw) const {
  // Extract the Z-rotation angle from the upper-left 3×3 rotation block
  double yaw_offset = std::atan2(tf(1, 0), tf(0, 0));
  return yaw + yaw_offset;
}

bool SafetyNode::isWithinBounds(const Eigen::Vector3d& pos) const {
  return pos.x() >= bounds_.x_min && pos.x() <= bounds_.x_max &&
         pos.y() >= bounds_.y_min && pos.y() <= bounds_.y_max &&
         pos.z() >= bounds_.z_min && pos.z() <= bounds_.z_max;
}

void SafetyNode::log(const std::string& msg) {
  // log_cb_ may be invoked from the executor thread – the Qt side must
  // use a queued connection so the slot runs on the GUI thread.
  LogCallback cb;
  {
    std::lock_guard<std::recursive_mutex> lock(mtx_);
    cb = log_cb_;
  }
  if (cb) cb(msg);
  RCLCPP_INFO(get_logger(), "%s", msg.c_str());
}
