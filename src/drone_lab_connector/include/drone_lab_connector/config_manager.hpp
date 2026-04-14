#pragma once

#include <Eigen/Dense>
#include <mutex>
#include <string>

// ─── Plain-data structs ─────────────────────────────────────────────────────

struct DroneConfig {
  bool hotspot_mode     = false;
  std::string hotspot_ip = "10.42.0.1";
  int hotspot_sys_id    = 1;

  int id_min            = 10;
  int id_max            = 19;
  std::string ip_prefix = "192.168.18.1";
  int local_port_base   = 14510;
  int fcu_port          = 14550;
  std::string ns_prefix = "/drones/edu";

  // Topic suffixes (appended to <ns>/)
  std::string student_setpoint_topic = "student_setpoint";
  std::string mavros_setpoint_topic  = "mavros/setpoint_position/local";
  std::string error_topic            = "error";
};

struct Boundaries {
  double x_min = -3.0, x_max = 3.0;
  double y_min = -3.0, y_max = 3.0;
  double z_min =  0.0, z_max = 3.0;
};

struct SetupConfig {
  std::string script_path;
  std::string indoor_command;
  std::string outdoor_command;
};

struct AppConfig {
  DroneConfig  drones;
  Boundaries   safety;
  Eigen::Matrix4d student_to_real = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d real_to_drone   = Eigen::Matrix4d::Identity();
  SetupConfig  setup;
  std::string  ros2_pc_ip = "192.168.18.100";
};

// ─── ConfigManager ──────────────────────────────────────────────────────────

class ConfigManager {
 public:
  ConfigManager();

  /// Load config: user_param.yaml if present, else default_config.yaml.
  bool load();

  /// Persist the current configuration to user_param.yaml.
  bool saveUserConfig() const;

  AppConfig&       config();
  const AppConfig& config() const;

  // Convenience helpers
  std::string droneIp(int id) const;
  int         localPort(int id) const;
  std::string droneNamespace(int id) const;
  std::string fcuUrl(int id) const;

  std::string defaultConfigPath() const;
  std::string userConfigPath() const;

 private:
  void loadFromFile(const std::string& path);
  void saveToFile(const std::string& path) const;

  AppConfig config_;
  mutable std::mutex mtx_;
};
