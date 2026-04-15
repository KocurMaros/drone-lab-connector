#include "drone_lab_connector/config_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

namespace fs = std::filesystem;

// ─── helpers ────────────────────────────────────────────────────────────────

static Eigen::Matrix4d loadMatrix4d(const YAML::Node& node) {
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  if (!node || !node.IsSequence() || node.size() != 4) return m;
  for (int r = 0; r < 4; ++r) {
    const auto& row = node[r];
    if (!row.IsSequence() || row.size() != 4) continue;
    for (int c = 0; c < 4; ++c) {
      m(r, c) = row[c].as<double>();
    }
  }
  return m;
}

static YAML::Node matrixToYaml(const Eigen::Matrix4d& m) {
  YAML::Node node;
  for (int r = 0; r < 4; ++r) {
    YAML::Node row;
    for (int c = 0; c < 4; ++c) {
      row.push_back(m(r, c));
    }
    node.push_back(row);
  }
  return node;
}

// ─── ConfigManager ─────────────────────────────────────────────────────────

ConfigManager::ConfigManager() = default;

std::string ConfigManager::defaultConfigPath() const {
  // 1) Check ament install share path via env
  const char* prefix = std::getenv("AMENT_PREFIX_PATH");
  if (prefix) {
    // AMENT_PREFIX_PATH can be colon-separated; check the first entry
    std::string paths(prefix);
    auto pos = paths.find(':');
    std::string first = (pos != std::string::npos) ? paths.substr(0, pos) : paths;
    fs::path candidate = fs::path(first) / "share" / "drone_lab_connector" / "config" / "default_config.yaml";
    if (fs::exists(candidate)) return candidate.string();
  }
  // 2) Fallback: relative to executable working directory
  fs::path local("config/default_config.yaml");
  if (fs::exists(local)) return local.string();
  // 3) Fallback: source tree path (development)
  return "/home/deck/Projects/drone-lab-connector/src/drone_lab_connector/config/default_config.yaml";
}

std::string ConfigManager::userConfigPath() const {
  const char* home = std::getenv("HOME");
  if (!home) home = "/tmp";
  return (fs::path(home) / ".config" / "drone_lab_connector" / "user_param.yaml").string();
}

bool ConfigManager::load() {
  std::lock_guard<std::mutex> lock(mtx_);
  try {
    std::string path = userConfigPath();
    if (!fs::exists(path)) {
      path = defaultConfigPath();
    }
    loadFromFile(path);
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[ConfigManager] load error: " << e.what() << "\n";
    return false;
  }
}

bool ConfigManager::saveUserConfig() const {
  std::lock_guard<std::mutex> lock(mtx_);
  try {
    saveToFile(userConfigPath());
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[ConfigManager] save error: " << e.what() << "\n";
    return false;
  }
}

AppConfig&       ConfigManager::config()       { return config_; }
const AppConfig& ConfigManager::config() const { return config_; }

// ─── drone helpers ──────────────────────────────────────────────────────────

std::string ConfigManager::droneIp(int id) const {
  if (config_.drones.hotspot_mode) return config_.drones.hotspot_ip;
  // ip_prefix + id  → e.g. "192.168.18.1" + "10" = "192.168.18.110"
  return config_.drones.ip_prefix + std::to_string(id);
}

int ConfigManager::localPort(int id) const {
  return config_.drones.local_port_base + (id - config_.drones.id_min);
}

std::string ConfigManager::droneNamespace(int id) const {
  return config_.drones.ns_prefix + std::to_string(id);
}

std::string ConfigManager::fcuUrl(int id) const {
  // udp://[bind_host][:bind_port]@[remote_host][:remote_port]
  std::ostringstream ss;
  ss << "udp://:" << localPort(id) << "@";
  return ss.str();
}

// ─── YAML I/O ───────────────────────────────────────────────────────────────

void ConfigManager::loadFromFile(const std::string& path) {
  YAML::Node root = YAML::LoadFile(path);

  if (auto d = root["drones"]) {
    if (d["hotspot_mode"])     config_.drones.hotspot_mode    = d["hotspot_mode"].as<bool>();
    if (d["hotspot_ip"])       config_.drones.hotspot_ip      = d["hotspot_ip"].as<std::string>();
    if (d["hotspot_sys_id"])   config_.drones.hotspot_sys_id  = d["hotspot_sys_id"].as<int>();
    if (d["id_min"])           config_.drones.id_min          = d["id_min"].as<int>();
    if (d["id_max"])           config_.drones.id_max          = d["id_max"].as<int>();
    if (d["ip_prefix"])        config_.drones.ip_prefix       = d["ip_prefix"].as<std::string>();
    if (d["local_port_base"])  config_.drones.local_port_base = d["local_port_base"].as<int>();
    if (d["fcu_port"])         config_.drones.fcu_port        = d["fcu_port"].as<int>();
    if (d["namespace_prefix"]) config_.drones.ns_prefix       = d["namespace_prefix"].as<std::string>();
    if (auto t = d["topics"]) {
      if (t["student_setpoint"]) config_.drones.student_setpoint_topic = t["student_setpoint"].as<std::string>();
      if (t["mavros_setpoint"])  config_.drones.mavros_setpoint_topic  = t["mavros_setpoint"].as<std::string>();
      if (t["error"])            config_.drones.error_topic             = t["error"].as<std::string>();
    }
  }

  if (auto r = root["ros2_pc"]) {
    if (r["ip"]) config_.ros2_pc_ip = r["ip"].as<std::string>();
  }

  if (auto s = root["safety"]) {
    if (auto b = s["boundaries"]) {
      if (b["x_min"]) config_.safety.x_min = b["x_min"].as<double>();
      if (b["x_max"]) config_.safety.x_max = b["x_max"].as<double>();
      if (b["y_min"]) config_.safety.y_min = b["y_min"].as<double>();
      if (b["y_max"]) config_.safety.y_max = b["y_max"].as<double>();
      if (b["z_min"]) config_.safety.z_min = b["z_min"].as<double>();
      if (b["z_max"]) config_.safety.z_max = b["z_max"].as<double>();
    }
  }

  if (auto tf = root["transforms"]) {
    config_.student_to_real = loadMatrix4d(tf["student_to_real"]);
    config_.real_to_drone   = loadMatrix4d(tf["real_to_drone"]);
  }

  if (auto su = root["setup"]) {
    if (su["script_path"])     config_.setup.script_path     = su["script_path"].as<std::string>();
    if (su["indoor_command"])  config_.setup.indoor_command   = su["indoor_command"].as<std::string>();
    if (su["outdoor_command"]) config_.setup.outdoor_command  = su["outdoor_command"].as<std::string>();
  }
}

void ConfigManager::saveToFile(const std::string& path) const {
  fs::create_directories(fs::path(path).parent_path());

  YAML::Node root;

  // drones
  root["drones"]["hotspot_mode"]     = config_.drones.hotspot_mode;
  root["drones"]["hotspot_ip"]       = config_.drones.hotspot_ip;
  root["drones"]["hotspot_sys_id"]   = config_.drones.hotspot_sys_id;
  root["drones"]["id_min"]           = config_.drones.id_min;
  root["drones"]["id_max"]           = config_.drones.id_max;
  root["drones"]["ip_prefix"]        = config_.drones.ip_prefix;
  root["drones"]["local_port_base"]  = config_.drones.local_port_base;
  root["drones"]["fcu_port"]         = config_.drones.fcu_port;
  root["drones"]["namespace_prefix"] = config_.drones.ns_prefix;
  root["drones"]["topics"]["student_setpoint"] = config_.drones.student_setpoint_topic;
  root["drones"]["topics"]["mavros_setpoint"]  = config_.drones.mavros_setpoint_topic;
  root["drones"]["topics"]["error"]            = config_.drones.error_topic;

  root["ros2_pc"]["ip"] = config_.ros2_pc_ip;

  // safety
  root["safety"]["boundaries"]["x_min"] = config_.safety.x_min;
  root["safety"]["boundaries"]["x_max"] = config_.safety.x_max;
  root["safety"]["boundaries"]["y_min"] = config_.safety.y_min;
  root["safety"]["boundaries"]["y_max"] = config_.safety.y_max;
  root["safety"]["boundaries"]["z_min"] = config_.safety.z_min;
  root["safety"]["boundaries"]["z_max"] = config_.safety.z_max;

  // transforms
  root["transforms"]["student_to_real"] = matrixToYaml(config_.student_to_real);
  root["transforms"]["real_to_drone"]   = matrixToYaml(config_.real_to_drone);

  // setup
  root["setup"]["script_path"]     = config_.setup.script_path;
  root["setup"]["indoor_command"]  = config_.setup.indoor_command;
  root["setup"]["outdoor_command"] = config_.setup.outdoor_command;

  std::ofstream fout(path);
  if (!fout.is_open()) {
    throw std::runtime_error("Cannot write " + path);
  }
  fout << root;
}
