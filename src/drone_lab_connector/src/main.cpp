#include <QApplication>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "drone_lab_connector/config_manager.hpp"
#include "drone_lab_connector/main_window.hpp"
#include "drone_lab_connector/safety_node.hpp"

int main(int argc, char* argv[]) {
  // ── Initialise ROS 2 and Qt ───────────────────────────────────────────
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  app.setApplicationName("Drone Lab Connector");

  // ── Load configuration ────────────────────────────────────────────────
  auto config = std::make_shared<ConfigManager>();
  if (!config->load()) {
    qWarning("Failed to load config – using built-in defaults");
  }

  // ── Create the ROS 2 safety node ──────────────────────────────────────
  auto safety_node = std::make_shared<SafetyNode>(config);

  // ── Spin the ROS executor in a background thread ──────────────────────
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(safety_node);

  std::thread ros_thread([executor]() { executor->spin(); });

  // ── Show the Qt GUI ───────────────────────────────────────────────────
  MainWindow window(config, safety_node);
  window.show();

  int result = app.exec();

  // ── Cleanup ───────────────────────────────────────────────────────────
  executor->cancel();
  if (ros_thread.joinable()) ros_thread.join();
  rclcpp::shutdown();

  return result;
}
