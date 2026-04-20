#include "drone_lab_connector/main_window.hpp"

#include <QApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QMessageBox>
#include <QScrollBar>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QVBoxLayout>

#include <mavlink/common/mavlink.h>
#include <mavlink_udp_interface.hpp>

#include <sstream>
#include <thread>
#include <chrono>

// ─── construction / destruction ─────────────────────────────────────────────

MainWindow::MainWindow(std::shared_ptr<ConfigManager> config,
                       std::shared_ptr<SafetyNode>    safety_node,
                       QWidget* parent)
    : QMainWindow(parent),
      config_(std::move(config)),
      safety_node_(std::move(safety_node)) {
  setWindowTitle("Drone Lab Connector");
  resize(960, 600);

  buildMenuBar();
  buildUi();
  connectSignals();
  populateDroneSelector();

  chk_hotspot_mode_->setChecked(config_->config().drones.hotspot_mode);
  onHotspotToggled(chk_hotspot_mode_->isChecked());

  statusBar()->showMessage("Ready – no drones active");
}

MainWindow::~MainWindow() {
  // Terminate all running MAVROS processes gracefully
  for (auto& [id, proc] : mavros_processes_) {
    if (proc && proc->state() != QProcess::NotRunning) {
      proc->terminate();
      proc->waitForFinished(3000);
      if (proc->state() != QProcess::NotRunning) proc->kill();
    }
    delete proc;
  }
  mavros_processes_.clear();
}

// ─── UI construction ────────────────────────────────────────────────────────

void MainWindow::buildMenuBar() {
  auto* file_menu = menuBar()->addMenu(tr("&File"));
  file_menu->addAction(tr("&Settings…"), this, &MainWindow::onOpenSettings);
  file_menu->addSeparator();
  file_menu->addAction(tr("&Quit"), qApp, &QApplication::quit,
                       QKeySequence::Quit);
}

void MainWindow::buildUi() {
  auto* splitter = new QSplitter(Qt::Horizontal, this);
  setCentralWidget(splitter);

  // ── Left panel ────────────────────────────────────────────────────────
  auto* left_widget = new QWidget;
  auto* left_layout = new QVBoxLayout(left_widget);

  // Drone selector
  auto* spawn_group  = new QGroupBox(tr("Drone Control"));
  auto* spawn_layout = new QVBoxLayout(spawn_group);

  chk_hotspot_mode_ = new QCheckBox(tr("Drone Hotspot Mode (10.42.0.1)"));
  spawn_layout->addWidget(chk_hotspot_mode_);

  auto* sel_row = new QHBoxLayout;
  drone_selector_ = new QComboBox;
  sel_row->addWidget(new QLabel(tr("Drone:")));
  sel_row->addWidget(drone_selector_, 1);
  spawn_layout->addLayout(sel_row);

  auto* btn_row = new QHBoxLayout;
  btn_spawn_ = new QPushButton(tr("Spawn MAVROS"));
  btn_kill_  = new QPushButton(tr("Kill MAVROS"));
  btn_kill_->setEnabled(false);
  btn_row->addWidget(btn_spawn_);
  btn_row->addWidget(btn_kill_);
  spawn_layout->addLayout(btn_row);

  left_layout->addWidget(spawn_group);

  // Active drones list
  auto* active_group  = new QGroupBox(tr("Active Drones"));
  auto* active_layout = new QVBoxLayout(active_group);
  active_list_        = new QListWidget;
  active_layout->addWidget(active_list_);
  left_layout->addWidget(active_group, 1);

  // Setup buttons
  auto* setup_group  = new QGroupBox(tr("Flight Setup"));
  auto* setup_layout = new QHBoxLayout(setup_group);
  btn_indoor_  = new QPushButton(tr("Setup Indoor Flight"));
  btn_outdoor_ = new QPushButton(tr("Setup Outdoor Flight"));
  setup_layout->addWidget(btn_indoor_);
  setup_layout->addWidget(btn_outdoor_);
  left_layout->addWidget(setup_group);

  splitter->addWidget(left_widget);

  // ── Right panel: log window ───────────────────────────────────────────
  auto* log_group  = new QGroupBox(tr("Log Output"));
  auto* log_layout = new QVBoxLayout(log_group);
  log_output_      = new QTextEdit;
  log_output_->setReadOnly(true);
  log_output_->setFontFamily("Monospace");
  log_layout->addWidget(log_output_);
  splitter->addWidget(log_group);

  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 2);
}

void MainWindow::connectSignals() {
  // Qt signal/slot
  connect(btn_spawn_,  &QPushButton::clicked, this, &MainWindow::onSpawnMavros);
  connect(btn_kill_,   &QPushButton::clicked, this, &MainWindow::onKillMavros);
  connect(btn_indoor_, &QPushButton::clicked, this, &MainWindow::onSetupIndoor);
  connect(btn_outdoor_,&QPushButton::clicked, this, &MainWindow::onSetupOutdoor);
  connect(chk_hotspot_mode_, &QCheckBox::toggled, this, &MainWindow::onHotspotToggled);

  // ROS log bridge (queued so slot runs on GUI thread)
  connect(this, &MainWindow::rosLogReceived,
          this, &MainWindow::appendLog, Qt::QueuedConnection);

  // Feed SafetyNode log into the Qt signal
  safety_node_->setLogCallback([this](const std::string& msg) {
    emit rosLogReceived(QString::fromStdString(msg));
  });

  // Update kill button based on list selection
  connect(active_list_, &QListWidget::currentRowChanged, this,
          [this](int row) { btn_kill_->setEnabled(row >= 0); });
}

void MainWindow::populateDroneSelector() {
  drone_selector_->clear();
  const auto& d = config_->config().drones;
  for (int id = d.id_min; id <= d.id_max; ++id) {
    drone_selector_->addItem(
        QString("EDU%1").arg(id), QVariant(id));
  }
}

void MainWindow::refreshActiveList() {
  active_list_->clear();
  for (const auto& [id, proc] : mavros_processes_) {
    QString state = (proc && proc->state() == QProcess::Running)
                        ? "Running"
                        : "Stopped";
    active_list_->addItem(
        QString("EDU%1 – %2").arg(id).arg(state));
  }
  statusBar()->showMessage(
      QString("%1 drone(s) active").arg(mavros_processes_.size()));
}

// ─── MAVROS process management ──────────────────────────────────────────────

void MainWindow::onHotspotToggled(bool checked) {
  if (!mavros_processes_.empty()) {
    QMessageBox::warning(this, "Cannot change mode", "Please kill active MAVROS processes first.");
    chk_hotspot_mode_->blockSignals(true);
    chk_hotspot_mode_->setChecked(!checked);
    chk_hotspot_mode_->blockSignals(false);
    return;
  }
  
  config_->config().drones.hotspot_mode = checked;
  // Let the user keep the drone selector enabled so they can choose the drone ID (e.g. EDU 13) 
  // which is then mapped correctly to port 14513 and target_sys 13
  config_->saveUserConfig();
}

void MainWindow::onSpawnMavros() {
  int drone_id = drone_selector_->currentData().toInt();
  if (mavros_processes_.count(drone_id)) {
    appendLog(QString("[WARN] EDU%1 MAVROS already running").arg(drone_id));
    return;
  }

  const std::string fcu   = config_->fcuUrl(drone_id);
  const std::string ns    = config_->droneNamespace(drone_id);

  QStringList args;
  args << "run" << "mavros" << "mavros_node"
       << "--ros-args"
       << "-p" << QString("fcu_url:=%1").arg(QString::fromStdString(fcu))
       << "-p" << QString("tgt_system:=%1").arg(drone_id)
       << "--remap" << QString("__ns:=%1").arg(QString::fromStdString(ns))
       // ── Remap writable topics to internal names (prevent student bypass) ──
       << "--remap" << "mavros/setpoint_position/local:=mavros/internal_setpoint_position/local"
       << "--remap" << "mavros/setpoint_velocity/cmd_vel_unstamped:=mavros/internal_setpoint_velocity/cmd_vel_unstamped"
       << "--remap" << "mavros/setpoint_raw/local:=mavros/_internal/setpoint_raw/local"
       << "--remap" << "mavros/setpoint_raw/global:=mavros/_internal/setpoint_raw/global"
       << "--remap" << "mavros/setpoint_raw/attitude:=mavros/_internal/setpoint_raw/attitude"
       << "--remap" << "mavros/setpoint_attitude/attitude:=mavros/_internal/setpoint_attitude/attitude"
       << "--remap" << "mavros/setpoint_attitude/cmd_vel:=mavros/_internal/setpoint_attitude/cmd_vel";

  auto* proc = new QProcess(this);
  proc->setProcessChannelMode(QProcess::MergedChannels);

  // Capture stdout/stderr in log
  connect(proc, &QProcess::readyReadStandardOutput, this, [this, proc, drone_id]() {
    QString text = proc->readAllStandardOutput();
    appendLog(QString("[EDU%1] %2").arg(drone_id).arg(text.trimmed()));
  });

  // Handle process exit
  connect(proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, [this, drone_id](int code, QProcess::ExitStatus status) {
            onProcessFinished(drone_id, code, status);
          });

  proc->start("ros2", args);
  if (!proc->waitForStarted(5000)) {
    appendLog(QString("[ERROR] Failed to start MAVROS for EDU%1: %2")
                  .arg(drone_id)
                  .arg(proc->errorString()));
    delete proc;
    return;
  }

  mavros_processes_[drone_id] = proc;

  // Activate safety node subscriptions for this drone
  safety_node_->activateDrone(drone_id);

  appendLog(QString("[INFO] Spawned MAVROS for EDU%1  fcu_url=%2  ns=%3")
                .arg(drone_id)
                .arg(QString::fromStdString(fcu))
                .arg(QString::fromStdString(ns)));
  refreshActiveList();
}

void MainWindow::onKillMavros() {
  auto* item = active_list_->currentItem();
  if (!item) return;

  // Parse drone id from list item text "EDU<id> – …"
  QString text = item->text();
  int id = text.mid(3, text.indexOf(' ') - 3).toInt();
  if (!mavros_processes_.count(id)) return;

  QProcess* proc = mavros_processes_[id];
  if (proc->state() != QProcess::NotRunning) {
    proc->terminate();
    if (!proc->waitForFinished(3000)) proc->kill();
  }
  // DO NOT `delete proc`, `mavros_processes_.erase(id)`, or `safety_node_->deactivateDrone(id)` here.
  // The `QProcess::finished` signal automatically fires and invokes `onProcessFinished()`,
  // which will safely perform memory cleanup and UI roster refresh without triggering a double-free!

  appendLog(QString("[INFO] Requested termination for MAVROS EDU%1").arg(id));
}

void MainWindow::onProcessFinished(int drone_id, int exitCode,
                                   QProcess::ExitStatus status) {
  QString reason = (status == QProcess::CrashExit) ? "CRASHED" : "exited";
  appendLog(QString("[INFO] EDU%1 MAVROS %2 (code %3)")
                .arg(drone_id)
                .arg(reason)
                .arg(exitCode));

  if (mavros_processes_.count(drone_id)) {
    QProcess* proc = mavros_processes_[drone_id];
    mavros_processes_.erase(drone_id);
    proc->deleteLater(); // Defer deletion so Qt loop cleans it up without crashing
  }
  safety_node_->deactivateDrone(drone_id);
  refreshActiveList();
}

// ─── setup scripts ──────────────────────────────────────────────────────────

void MainWindow::onSetupIndoor() {
  int id = drone_selector_->currentData().toInt();
  sendSetupParameters(id, true);
}

void MainWindow::onSetupOutdoor() {
  int id = drone_selector_->currentData().toInt();
  sendSetupParameters(id, false);
}

void MainWindow::sendSetupParameters(int drone_id, bool indoor) {
  std::string ip = config_->droneIp(drone_id);
  int port = config_->config().drones.fcu_port; // 14550
  
  // Always use the selected drone_id as the target System ID.
  // (Using hotspot_sys_id defaults to 1, causing the drone to ignore packets!)
  int target_sys = drone_id;
  
  QUdpSocket udp;
  QHostAddress targetAddress(QString::fromStdString(ip));

  // SEND HEARTBEAT first so mavlink-router accepts this UDP endpoint
  mavlink_message_t hb_msg;
  mavlink_msg_heartbeat_pack(255, 1, &hb_msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
  uint8_t hb_buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t hb_len = mavlink_msg_to_send_buffer(hb_buffer, &hb_msg);
  
  for (int i = 0; i < 3; ++i) {
      udp.writeDatagram(reinterpret_cast<const char*>(hb_buffer), hb_len, targetAddress, port);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  
  auto send_param = [&](const std::string& name, float value, uint8_t param_type = MAV_PARAM_TYPE_REAL32) {
     mavlink_message_t msg;
     mavlink_param_set_t param;
     param.target_system = target_sys;
     param.target_component = 1; // MAV_COMP_ID_AUTOPILOT1
     param.param_value = value;
     param.param_type = param_type;
     
     std::memset(param.param_id, 0, 16);
     std::strncpy(param.param_id, name.c_str(), 16);
     
     mavlink_msg_param_set_encode(255, 1, &msg, &param);
     
     uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
     uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
     
     // Queue message multiple times to ensure UDP delivery
     udp.writeDatagram(reinterpret_cast<const char*>(buffer), len, targetAddress, port);
     std::this_thread::sleep_for(std::chrono::milliseconds(5));
     udp.writeDatagram(reinterpret_cast<const char*>(buffer), len, targetAddress, port);
     std::this_thread::sleep_for(std::chrono::milliseconds(5));
     udp.writeDatagram(reinterpret_cast<const char*>(buffer), len, targetAddress, port);
     std::this_thread::sleep_for(std::chrono::milliseconds(20));
  };
  
  // Wait to let packets flush
  udp.waitForBytesWritten(100);
  
  appendLog(QString("[SETUP EDU%1] Setting params for %2 (IP: %3:%4)...")
                .arg(drone_id)
                .arg(indoor ? "INDOOR (OptiTrack)" : "OUTDOOR (GPS/Baro)")
                .arg(QString::fromStdString(ip))
                .arg(port));
  
  // Revert back to REAL32. ArduPilot parameter sets via Float directly natively handles uint conversions correctly.
  if (indoor) {
    send_param("VISO_TYPE", 1.0f);
    send_param("AHRS_EKF_TYPE", 3.0f);
    send_param("EK3_ENABLE", 1.0f);
    send_param("EK2_ENABLE", 0.0f);
    send_param("COMPASS_USE", 0.0f);
    send_param("COMPASS_USE2", 0.0f);
    send_param("COMPASS_USE3", 0.0f);
    send_param("VISO_POS_M_NSE", 0.3f);
    send_param("VISO_YAW_M_NSE", 0.2f);
    send_param("EK3_SRC1_POSXY", 6.0f); // ExternalNav
    send_param("EK3_SRC1_POSZ",  6.0f); // ExternalNav
    send_param("EK3_SRC1_VELXY", 0.0f); // ExternalNav
    send_param("EK3_SRC1_VELZ",  0.0f); // ExternalNav
    send_param("EK3_SRC1_YAW",   6.0f); // ExternalNav
  } else {
    send_param("VISO_TYPE", 0.0f);
    send_param("COMPASS_USE", 1.0f);
    send_param("COMPASS_USE2", 1.0f);
    send_param("COMPASS_USE3", 1.0f);
    send_param("EK3_SRC1_POSXY", 3.0f, MAV_PARAM_TYPE_REAL32); // GPS
    send_param("EK3_SRC1_POSZ",  1.0f, MAV_PARAM_TYPE_REAL32); // Baro
    send_param("EK3_SRC1_VELXY", 3.0f, MAV_PARAM_TYPE_REAL32); // GPS
    send_param("EK3_SRC1_VELZ",  3.0f, MAV_PARAM_TYPE_REAL32); // GPS
    send_param("EK3_SRC1_YAW",   1.0f, MAV_PARAM_TYPE_REAL32); // Compass
  }
  
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  udp.waitForBytesWritten(100);
  appendLog(QString("[SETUP EDU%1] Parameter send complete.").arg(drone_id));
}

// ─── log ────────────────────────────────────────────────────────────────────

void MainWindow::appendLog(const QString& text) {
  log_output_->append(text);
  // Auto-scroll to bottom
  auto* sb = log_output_->verticalScrollBar();
  if (sb) sb->setValue(sb->maximum());
}

// ─── settings dialog ────────────────────────────────────────────────────────

void MainWindow::onOpenSettings() {
  QDialog dlg(this);
  dlg.setWindowTitle(tr("Settings"));
  dlg.resize(520, 480);
  auto* root = new QVBoxLayout(&dlg);

  auto* tabs = new QTabWidget;
  root->addWidget(tabs);

  // ── Boundaries tab ────────────────────────────────────────────────────
  auto* bnd_page   = new QWidget;
  auto* bnd_layout = new QFormLayout(bnd_page);

  auto makeSpin = [](double val, double lo = -100, double hi = 100) {
    auto* s = new QDoubleSpinBox;
    s->setRange(lo, hi);
    s->setDecimals(2);
    s->setValue(val);
    return s;
  };

  const auto& b = config_->config().safety;
  auto* xmin = makeSpin(b.x_min); bnd_layout->addRow("X min", xmin);
  auto* xmax = makeSpin(b.x_max); bnd_layout->addRow("X max", xmax);
  auto* ymin = makeSpin(b.y_min); bnd_layout->addRow("Y min", ymin);
  auto* ymax = makeSpin(b.y_max); bnd_layout->addRow("Y max", ymax);
  auto* zmin = makeSpin(b.z_min); bnd_layout->addRow("Z min", zmin);
  auto* zmax = makeSpin(b.z_max); bnd_layout->addRow("Z max", zmax);
  tabs->addTab(bnd_page, tr("Safety Boundaries"));

  // ── Transforms tab (student_to_real) ──────────────────────────────────
  auto makeMatrixTable = [](const Eigen::Matrix4d& m) {
    auto* tbl = new QTableWidget(4, 4);
    tbl->horizontalHeader()->setVisible(false);
    tbl->verticalHeader()->setVisible(false);
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        tbl->setItem(r, c, new QTableWidgetItem(QString::number(m(r, c), 'f', 6)));
    return tbl;
  };

  auto* tf_page   = new QWidget;
  auto* tf_layout = new QVBoxLayout(tf_page);
  tf_layout->addWidget(new QLabel(tr("Student → Real-world")));
  auto* tbl_s2r = makeMatrixTable(config_->config().student_to_real);
  tf_layout->addWidget(tbl_s2r);
  tf_layout->addWidget(new QLabel(tr("Real-world → Drone")));
  auto* tbl_r2d = makeMatrixTable(config_->config().real_to_drone);
  tf_layout->addWidget(tbl_r2d);
  tabs->addTab(tf_page, tr("Transforms"));

  // ── Drone / Network tab ───────────────────────────────────────────────
  auto* net_page   = new QWidget;
  auto* net_layout = new QFormLayout(net_page);
  const auto& d = config_->config().drones;
  auto* ip_prefix = new QLineEdit(QString::fromStdString(d.ip_prefix));
  auto* port_base = new QLineEdit(QString::number(d.local_port_base));
  auto* fcu_port  = new QLineEdit(QString::number(d.fcu_port));
  auto* ns_prefix = new QLineEdit(QString::fromStdString(d.ns_prefix));
  net_layout->addRow("IP prefix",       ip_prefix);
  net_layout->addRow("Local port base", port_base);
  net_layout->addRow("FCU port",        fcu_port);
  net_layout->addRow("Namespace prefix",ns_prefix);
  tabs->addTab(net_page, tr("Network"));

  // ── Setup tab ─────────────────────────────────────────────────────────
  auto* setup_page   = new QWidget;
  auto* setup_layout = new QFormLayout(setup_page);
  auto* indoor_cmd   = new QLineEdit(
      QString::fromStdString(config_->config().setup.indoor_command));
  auto* outdoor_cmd  = new QLineEdit(
      QString::fromStdString(config_->config().setup.outdoor_command));
  setup_layout->addRow("Indoor cmd",  indoor_cmd);
  setup_layout->addRow("Outdoor cmd", outdoor_cmd);
  tabs->addTab(setup_page, tr("Setup Commands"));

  // ── Button box ────────────────────────────────────────────────────────
  auto* buttons = new QDialogButtonBox(
      QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  root->addWidget(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

  if (dlg.exec() != QDialog::Accepted) return;

  // ── Apply ─────────────────────────────────────────────────────────────
  auto& cfg = config_->config();
  cfg.safety.x_min = xmin->value();
  cfg.safety.x_max = xmax->value();
  cfg.safety.y_min = ymin->value();
  cfg.safety.y_max = ymax->value();
  cfg.safety.z_min = zmin->value();
  cfg.safety.z_max = zmax->value();

  auto readMatrix = [](QTableWidget* tbl) {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        if (auto* item = tbl->item(r, c))
          m(r, c) = item->text().toDouble();
    return m;
  };
  cfg.student_to_real = readMatrix(tbl_s2r);
  cfg.real_to_drone   = readMatrix(tbl_r2d);

  cfg.drones.ip_prefix       = ip_prefix->text().toStdString();
  cfg.drones.local_port_base = port_base->text().toInt();
  cfg.drones.fcu_port        = fcu_port->text().toInt();
  cfg.drones.ns_prefix       = ns_prefix->text().toStdString();

  cfg.setup.indoor_command   = indoor_cmd->text().toStdString();
  cfg.setup.outdoor_command  = outdoor_cmd->text().toStdString();

  // Persist & update safety node cache
  config_->saveUserConfig();
  safety_node_->reloadConfig();
  populateDroneSelector();

  appendLog("[INFO] Settings updated and saved to user_param.yaml");
}
