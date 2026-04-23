#include "drone_lab_connector/safety_node.hpp"

#include <cmath>
#include <sstream>

using namespace std::chrono_literals;

// ─── construction ───────────────────────────────────────────────────────────

SafetyNode::SafetyNode(std::shared_ptr<ConfigManager> config)
    : rclcpp::Node("drone_lab_safety"), config_mgr_(std::move(config)) {
  reloadConfig();

  // Reentrant group so service proxies can block-wait for MAVROS responses
  service_cbg_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/drone_markers", 10);
  marker_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SafetyNode::publishMarkers, this));

  RCLCPP_INFO(get_logger(), "SafetyNode initialised");
}

// ─── drone lifecycle ────────────────────────────────────────────────────────

void SafetyNode::activateDrone(int drone_id) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (handles_.count(drone_id)) return;  // already active

  const auto& cfg = config_mgr_->config().drones;
  const std::string ns = config_mgr_->droneNamespace(drone_id);

  DroneHandles h;

  // ── Student setpoint → safety check → MAVROS ─────────────────────────
  std::string student_sp = ns + "/" + cfg.student_setpoint_topic;
  h.student_setpoint_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
      student_sp, rclcpp::SensorDataQoS(),
      [this, drone_id](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        onStudentSetpoint(drone_id, msg);
      });

  std::string mavros_sp = ns + "/" + cfg.mavros_setpoint_topic;
  h.mavros_setpoint_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
      mavros_sp, 10);

  // ── Error topic ───────────────────────────────────────────────────────
  h.error_pub = create_publisher<std_msgs::msg::String>(
      ns + "/" + cfg.error_topic, 10);

  // ── Pose tracking (MAVROS native topic, for RViz markers) ─────────────
  h.pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/mavros_node/pose", rclcpp::SensorDataQoS(),
      [this, drone_id](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::recursive_mutex> lk(mtx_);
        auto it = handles_.find(drone_id);
        if (it == handles_.end()) return;
        it->second.last_pose = msg;
      });

  // ── State tracking (MAVROS native topic, for mode/arm checks) ─────────
  h.state_sub = create_subscription<mavros_msgs::msg::State>(
      ns + "/state", 10,
      [this, drone_id](mavros_msgs::msg::State::SharedPtr msg) {
        std::lock_guard<std::recursive_mutex> lk(mtx_);
        auto it = handles_.find(drone_id);
        if (it == handles_.end()) return;
        it->second.current_mode = msg->mode;
        it->second.is_armed     = msg->armed;
      });

  // ── Service proxies (student calls safe/*, safety forwards to MAVROS) ─
  // Arming: student → /safe/cmd/arming → safety → /mavros_node/arming (MAVROS)
  h.arm_cli = create_client<mavros_msgs::srv::CommandBool>(
      ns + "/mavros_node/arming",
      rmw_qos_profile_services_default, service_cbg_);
  h.arm_srv = create_service<mavros_msgs::srv::CommandBool>(
      ns + "/safe/cmd/arming",
      [this, drone_id](
          const mavros_msgs::srv::CommandBool::Request::SharedPtr req,
          mavros_msgs::srv::CommandBool::Response::SharedPtr res) {
        onArmRequest(drone_id, req, res);
      },
      rmw_qos_profile_services_default, service_cbg_);

  // Set mode: student → /safe/set_mode → safety → /set_mode (MAVROS)
  h.mode_cli = create_client<mavros_msgs::srv::SetMode>(
      ns + "/set_mode",
      rmw_qos_profile_services_default, service_cbg_);
  h.mode_srv = create_service<mavros_msgs::srv::SetMode>(
      ns + "/safe/set_mode",
      [this, drone_id](
          const mavros_msgs::srv::SetMode::Request::SharedPtr req,
          mavros_msgs::srv::SetMode::Response::SharedPtr res) {
        onSetModeRequest(drone_id, req, res);
      },
      rmw_qos_profile_services_default, service_cbg_);

  // Takeoff: student → /safe/cmd/takeoff → safety → /mavros_node/takeoff (MAVROS)
  h.takeoff_cli = create_client<mavros_msgs::srv::CommandTOL>(
      ns + "/mavros_node/takeoff",
      rmw_qos_profile_services_default, service_cbg_);
  h.takeoff_srv = create_service<mavros_msgs::srv::CommandTOL>(
      ns + "/safe/cmd/takeoff",
      [this, drone_id](
          const mavros_msgs::srv::CommandTOL::Request::SharedPtr req,
          mavros_msgs::srv::CommandTOL::Response::SharedPtr res) {
        onTakeoffRequest(drone_id, req, res);
      },
      rmw_qos_profile_services_default, service_cbg_);

  // Land: student → /safe/cmd/land → safety → /mavros_node/land (MAVROS)
  h.land_cli = create_client<mavros_msgs::srv::CommandTOL>(
      ns + "/mavros_node/land",
      rmw_qos_profile_services_default, service_cbg_);
  h.land_srv = create_service<mavros_msgs::srv::CommandTOL>(
      ns + "/safe/cmd/land",
      [this, drone_id](
          const mavros_msgs::srv::CommandTOL::Request::SharedPtr req,
          mavros_msgs::srv::CommandTOL::Response::SharedPtr res) {
        onLandRequest(drone_id, req, res);
      },
      rmw_qos_profile_services_default, service_cbg_);

  handles_[drone_id] = std::move(h);

  log("[SafetyNode] Activated EDU" + std::to_string(drone_id) +
      " | student safe services: " + ns + "/safe/{cmd/arming, set_mode, cmd/takeoff, cmd/land}");
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

// ─── setpoint validation callback ───────────────────────────────────────────

void SafetyNode::onStudentSetpoint(
    int drone_id,
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  Eigen::Matrix4d s2r, r2d;
  Boundaries b;
  {
    std::lock_guard<std::recursive_mutex> lock(mtx_);
    s2r = student_to_real_;
    r2d = real_to_drone_;
    b   = bounds_;
  }

  const Eigen::Vector3d student_pos(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  const double student_yaw = yawFromQuaternion(msg->pose.orientation);

  const Eigen::Vector3d real_pos = transformPosition(s2r, student_pos);
  const double real_yaw          = transformYaw(s2r, student_yaw);

  if (!isWithinBounds(real_pos)) {
    std::ostringstream ss;
    ss << "OUT OF BOUNDS  EDU" << drone_id << "  real=("
       << real_pos.x() << ", " << real_pos.y() << ", " << real_pos.z()
       << ")  limits X[" << b.x_min << "," << b.x_max
       << "] Y[" << b.y_min << "," << b.y_max
       << "] Z[" << b.z_min << "," << b.z_max << "]";
    const std::string err_str = ss.str();
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

  const Eigen::Vector3d drone_pos = transformPosition(r2d, real_pos);
  const double drone_yaw          = transformYaw(r2d, real_yaw);

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
      it->second.mavros_setpoint_pub->publish(out);
    }
  }
}

// ─── service proxy handlers ─────────────────────────────────────────────────

void SafetyNode::onArmRequest(
    int drone_id,
    const mavros_msgs::srv::CommandBool::Request::SharedPtr req,
    mavros_msgs::srv::CommandBool::Response::SharedPtr res) {

  std::string mode;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr client;
  {
    std::lock_guard<std::recursive_mutex> lk(mtx_);
    auto it = handles_.find(drone_id);
    if (it == handles_.end()) { res->success = false; return; }
    mode   = it->second.current_mode;
    client = it->second.arm_cli;
  }

  // Safety: can only arm in LOITER (real drones reject arming in GUIDED)
  if (req->value && mode != "LOITER") {
    std::string err = "EDU" + std::to_string(drone_id) +
        " cannot arm in " + mode + " – switch to LOITER first";
    log("[BLOCKED] " + err);
    {
      std::lock_guard<std::recursive_mutex> lk(mtx_);
      auto it = handles_.find(drone_id);
      if (it != handles_.end()) {
        std_msgs::msg::String m; m.data = err;
        it->second.error_pub->publish(m);
      }
    }
    res->success = false;
    return;
  }

  if (!client->wait_for_service(2s)) {
    log("[ERROR] MAVROS arming service unavailable for EDU" +
        std::to_string(drone_id));
    res->success = false;
    return;
  }

  auto future = client->async_send_request(req);
  if (future.wait_for(5s) == std::future_status::ready) {
    auto r = future.get();
    res->success = r->success;
    res->result  = r->result;
    log("[ARM] EDU" + std::to_string(drone_id) + " " +
        (req->value ? "ARM" : "DISARM") + " → " +
        (res->success ? "OK" : "FAILED"));
  } else {
    log("[ERROR] MAVROS arming timeout for EDU" + std::to_string(drone_id));
    res->success = false;
  }
}

void SafetyNode::onSetModeRequest(
    int drone_id,
    const mavros_msgs::srv::SetMode::Request::SharedPtr req,
    mavros_msgs::srv::SetMode::Response::SharedPtr res) {

  bool armed;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client;
  {
    std::lock_guard<std::recursive_mutex> lk(mtx_);
    auto it = handles_.find(drone_id);
    if (it == handles_.end()) { res->mode_sent = false; return; }
    armed  = it->second.is_armed;
    client = it->second.mode_cli;
  }

  // Safety: cannot switch to GUIDED unless armed (arm in LOITER first)
  if (req->custom_mode == "GUIDED" && !armed) {
    std::string err = "EDU" + std::to_string(drone_id) +
        " cannot switch to GUIDED – arm in LOITER first";
    log("[BLOCKED] " + err);
    {
      std::lock_guard<std::recursive_mutex> lk(mtx_);
      auto it = handles_.find(drone_id);
      if (it != handles_.end()) {
        std_msgs::msg::String m; m.data = err;
        it->second.error_pub->publish(m);
      }
    }
    res->mode_sent = false;
    return;
  }

  if (!client->wait_for_service(2s)) {
    log("[ERROR] MAVROS set_mode service unavailable for EDU" +
        std::to_string(drone_id));
    res->mode_sent = false;
    return;
  }

  auto future = client->async_send_request(req);
  if (future.wait_for(5s) == std::future_status::ready) {
    auto r = future.get();
    res->mode_sent = r->mode_sent;
    log("[MODE] EDU" + std::to_string(drone_id) + " → " +
        req->custom_mode + (res->mode_sent ? " OK" : " FAILED"));
  } else {
    log("[ERROR] MAVROS set_mode timeout for EDU" + std::to_string(drone_id));
    res->mode_sent = false;
  }
}

void SafetyNode::onTakeoffRequest(
    int drone_id,
    const mavros_msgs::srv::CommandTOL::Request::SharedPtr req,
    mavros_msgs::srv::CommandTOL::Response::SharedPtr res) {

  bool armed;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr client;
  {
    std::lock_guard<std::recursive_mutex> lk(mtx_);
    auto it = handles_.find(drone_id);
    if (it == handles_.end()) { res->success = false; return; }
    armed  = it->second.is_armed;
    client = it->second.takeoff_cli;
  }

  if (!armed) {
    log("[BLOCKED] EDU" + std::to_string(drone_id) + " takeoff rejected – not armed");
    res->success = false;
    return;
  }

  if (req->altitude > bounds_.z_max) {
    log("[BLOCKED] EDU" + std::to_string(drone_id) + " takeoff alt " +
        std::to_string(req->altitude) + " exceeds limit " +
        std::to_string(bounds_.z_max));
    res->success = false;
    return;
  }

  if (!client->wait_for_service(2s)) {
    log("[ERROR] MAVROS takeoff service unavailable for EDU" +
        std::to_string(drone_id));
    res->success = false;
    return;
  }

  auto future = client->async_send_request(req);
  if (future.wait_for(5s) == std::future_status::ready) {
    auto r = future.get();
    res->success = r->success;
    res->result  = r->result;
    log("[TAKEOFF] EDU" + std::to_string(drone_id) + " alt=" +
        std::to_string(req->altitude) + " → " +
        (res->success ? "OK" : "FAILED"));
  } else {
    log("[ERROR] MAVROS takeoff timeout for EDU" + std::to_string(drone_id));
    res->success = false;
  }
}

void SafetyNode::onLandRequest(
    int drone_id,
    const mavros_msgs::srv::CommandTOL::Request::SharedPtr req,
    mavros_msgs::srv::CommandTOL::Response::SharedPtr res) {

  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr client;
  {
    std::lock_guard<std::recursive_mutex> lk(mtx_);
    auto it = handles_.find(drone_id);
    if (it == handles_.end()) { res->success = false; return; }
    client = it->second.land_cli;
  }

  // Landing is always allowed
  if (!client->wait_for_service(2s)) {
    log("[ERROR] MAVROS land service unavailable for EDU" +
        std::to_string(drone_id));
    res->success = false;
    return;
  }

  auto future = client->async_send_request(req);
  if (future.wait_for(5s) == std::future_status::ready) {
    auto r = future.get();
    res->success = r->success;
    res->result  = r->result;
    log("[LAND] EDU" + std::to_string(drone_id) + " → " +
        (res->success ? "OK" : "FAILED"));
  } else {
    log("[ERROR] MAVROS land timeout for EDU" + std::to_string(drone_id));
    res->success = false;
  }
}

// ─── math helpers ───────────────────────────────────────────────────────────

double SafetyNode::yawFromQuaternion(
    const geometry_msgs::msg::Quaternion& q) {
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
  double yaw_offset = std::atan2(tf(1, 0), tf(0, 0));
  return yaw + yaw_offset;
}

bool SafetyNode::isWithinBounds(const Eigen::Vector3d& pos) const {
  return pos.x() >= bounds_.x_min && pos.x() <= bounds_.x_max &&
         pos.y() >= bounds_.y_min && pos.y() <= bounds_.y_max &&
         pos.z() >= bounds_.z_min && pos.z() <= bounds_.z_max;
}

void SafetyNode::log(const std::string& msg) {
  LogCallback cb;
  {
    std::lock_guard<std::recursive_mutex> lock(mtx_);
    cb = log_cb_;
  }
  if (cb) cb(msg);
  RCLCPP_INFO(get_logger(), "%s", msg.c_str());
}

// ─── RViz marker publishing ────────────────────────────────────────────────

void SafetyNode::publishMarkers() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_all);

  for (const auto& [drone_id, h] : handles_) {
    if (!h.last_pose) continue;

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now();
    m.ns = "drones";
    m.id = drone_id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = h.last_pose->pose;
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.15;
    m.color.r = 0.0f;
    m.color.g = 0.8f;
    m.color.b = 1.0f;
    m.color.a = 0.9f;
    m.lifetime = rclcpp::Duration(0, 500000000);
    marker_array.markers.push_back(m);

    visualization_msgs::msg::Marker label;
    label.header.frame_id = "map";
    label.header.stamp = now();
    label.ns = "drone_labels";
    label.id = drone_id;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose = h.last_pose->pose;
    label.pose.position.z += 0.35;
    label.scale.z = 0.2;
    label.color.r = 1.0f;
    label.color.g = 1.0f;
    label.color.b = 1.0f;
    label.color.a = 1.0f;
    label.text = "EDU" + std::to_string(drone_id);
    label.lifetime = rclcpp::Duration(0, 500000000);
    marker_array.markers.push_back(label);
  }

  marker_pub_->publish(marker_array);
}
