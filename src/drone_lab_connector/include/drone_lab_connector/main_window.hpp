#pragma once

#include <QComboBox>
#include <QGroupBox>
#include <QListWidget>
#include <QMainWindow>
#include <QProcess>
#include <QPushButton>
#include <QTextEdit>
#include <QUdpSocket>
#include <QtNetwork>

#include <map>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "drone_lab_connector/config_manager.hpp"
#include "drone_lab_connector/safety_node.hpp"

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(std::shared_ptr<ConfigManager> config,
             std::shared_ptr<SafetyNode>    safety_node,
             QWidget* parent = nullptr);
  ~MainWindow() override;

 signals:
  /// Emitted from the ROS executor thread; connected with Qt::QueuedConnection.
  void rosLogReceived(const QString& text);

 private slots:
  void onSpawnMavros();
  void onKillMavros();
  void onSetupIndoor();
  void onSetupOutdoor();
  void onOpenSettings();
  void appendLog(const QString& text);
  void onProcessFinished(int drone_id, int exitCode,
                         QProcess::ExitStatus status);
  void onHotspotToggled(bool checked);

 private:
  void buildUi();
  void buildMenuBar();
  void connectSignals();
  void populateDroneSelector();
  void refreshActiveList();

  void sendSetupParameters(int drone_id, bool indoor);

  // ── data ──────────────────────────────────────────────────────────────
  std::shared_ptr<ConfigManager> config_;
  std::shared_ptr<SafetyNode>    safety_node_;

  // map drone_id → running QProcess*
  std::map<int, QProcess*> mavros_processes_;

  // ── widgets ───────────────────────────────────────────────────────────
  class QCheckBox* chk_hotspot_mode_ = nullptr;
  QComboBox*    drone_selector_  = nullptr;
  QPushButton*  btn_spawn_       = nullptr;
  QPushButton*  btn_kill_        = nullptr;
  QListWidget*  active_list_     = nullptr;
  QPushButton*  btn_indoor_      = nullptr;
  QPushButton*  btn_outdoor_     = nullptr;
  QTextEdit*    log_output_      = nullptr;
};
