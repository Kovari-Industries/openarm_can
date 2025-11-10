#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QWidget>
#include <QThread>
#include <QColor>
#include <QPen>
#include <QDir>
#include <QDateTime>
#include <QDebug>
#include <QFont>
#include <qcustomplot.h>

#include <openarm/can/socket/openarm.hpp>

namespace oa  = openarm;
namespace oacs = openarm::can::socket;
namespace dm  = openarm::damiao_motor;

using std::chrono::milliseconds;

enum class JointID : int {
  J1 = 0,
  J2 = 1,
  J3 = 2,
  J4 = 3,
  J5 = 4,
  J6 = 5,
  J7 = 6,
  GRIPPER = 0
};

static inline double deg2rad(double d) { return d * M_PI / 180.0; }
static inline double rad2deg(double r) { return r * 180.0 / M_PI; }

// radian limits
static const std::pair<double,double> mech_lim[8] = {
  /* J1 */      {deg2rad(-80.0),  deg2rad(200.0)},
  /* J2 */      {deg2rad(-100.0), deg2rad(100.0)},
  /* J3 */      {deg2rad(-90.0),  deg2rad(90.0)},
  /* J4 */      {deg2rad(0.0),    deg2rad(140.0)},
  /* J5 */      {deg2rad(-90.0),  deg2rad(90.0)},
  /* J6 */      {deg2rad(-45.0),  deg2rad(45.0)},
  /* J7 */      {deg2rad(-90.0),  deg2rad(90.0)},
  /* GRIPPER */ {deg2rad(-60.0),  deg2rad(0.0)}
};

static const double JOINT_SIGN[8] = {
/* J1 */ -1.0,
/* J2 */ -1.0,
/* J3 */ +1.0,
/* J4 */ +1.0,
/* J5 */ +1.0,
/* J6 */ +1.0,
/* J7 */ +1.0,
/* G  */ +1.0
};

static std::pair<double,double> hit_thresholds(const dm::DMDeviceCollection& comp, int idx) {
  const auto motors = comp.get_motors();
  const bool is_gripper = (motors.size() == 1);

  if (is_gripper) return {0.3, 0.3};
  if (idx == 0)     return {0.0125, 5.0};
  return {0.1, 2.0};
}

static void interpolate(
  oacs::OpenArm& openarm,
  dm::DMDeviceCollection& comp,
  JointID joint_id,
  double delta_rad,
  double kp = 52.0,
  double kd = 1.5,
  double torque_assist = 0.0,
  double interp_time_sec = 2.0)
{
  const int idx = static_cast<int>(joint_id);
  const auto motors = comp.get_motors();
  if (idx < 0 || idx >= static_cast<int>(motors.size())) return;

  const double q0 = motors[idx].get_position();
  const double q1 = q0 + delta_rad;

  const int n_steps = 500;
  const double dt = interp_time_sec / static_cast<double>(n_steps);
  const double tau = (q1 - q0) >= 0.0 ? std::abs(torque_assist) : -std::abs(torque_assist);

  for (int i = 0; i <= n_steps; ++i) {
    const double alpha = static_cast<double>(i) / static_cast<double>(n_steps);
    const double q = q0 + (q1 - q0) * alpha;

    dm::MITParam p{};
    p.kp  = kp;
    p.kd  = kd;
    p.q   = q;
    p.dq  = 0.0;
    p.tau = tau;

    comp.mit_control_one(idx, p);
    openarm.recv_all();
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
  }
}

static double bump_to_limit(
  oacs::OpenArm& openarm,
  dm::DMDeviceCollection& comp,
  JointID joint_id,
  double step_deg = 0.2,
  double kp = 45.0,
  double kd = 1.2,
  double torque_bias = 0.0)
{
  const int idx = static_cast<int>(joint_id);
  auto motors = comp.get_motors();
  if (idx < 0 || idx >= static_cast<int>(motors.size())) return 0.0;

  const double step_rad = deg2rad(step_deg);
  const double tau_cmd = (step_deg >= 0.0) ? std::abs(torque_bias) : -std::abs(torque_bias);

  const double q_start = motors[idx].get_position();
  double q_target = q_start;

  const auto [dq_th, tau_th] = hit_thresholds(comp, idx);

  while (true) {
    q_target += step_rad;

    dm::MITParam p{};
    p.kp  = kp;
    p.kd  = kd;
    p.q   = q_target;
    p.dq  = 0.0;
    p.tau = tau_cmd;

    comp.mit_control_one(idx, p);
    openarm.recv_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    motors = comp.get_motors();
    const double vel = std::fabs(motors[idx].get_velocity());
    const double tau = std::fabs(motors[idx].get_torque());

    if (vel < dq_th && tau > tau_th) {
      const double delta_rad = motors[idx].get_position() - q_start;
      std::cout << "[INFO] mechanical stop (Joint " << static_cast<int>(joint_id)
                << "): " << delta_rad << " rad / " << rad2deg(delta_rad) << "°\n";
      return delta_rad;
    }
  }
}

static double calc_delta_to_zero_pos_joint(
  double initial_rad,
  double ideal_limit_rad,
  double delta_to_stop_rad,
  JointID joint_id)
{
  const double q_hit = initial_rad + delta_to_stop_rad;
  const double delta_to_ideal = ideal_limit_rad - q_hit;
  const int j = static_cast<int>(joint_id);
  return JOINT_SIGN[j] * delta_to_ideal;
}

struct SequenceResult {
  double ideal[7];
  double deltas[8];
};

static SequenceResult run_right_sequence(oacs::OpenArm& openarm,
                                         dm::DMDeviceCollection& arm,
                                         dm::DMDeviceCollection& grip)
{
  SequenceResult R{};

  const double d_grip = bump_to_limit(openarm, grip, JointID::GRIPPER);               std::this_thread::sleep_for(milliseconds(500));
  const double d_j4   = bump_to_limit(openarm, arm,  JointID::J4, -0.2);              std::this_thread::sleep_for(milliseconds(500));

  interpolate(openarm, arm, JointID::J2,  deg2rad(5),  52.0, 1.5, 0.0, 0.4);          std::this_thread::sleep_for(milliseconds(500));
  const double d_j3   = bump_to_limit(openarm, arm,  JointID::J3);                    std::this_thread::sleep_for(milliseconds(500));

  interpolate(openarm, arm, JointID::J3, -mech_lim[(int)JointID::J3].second, 52.0, 1.5, 0.0, 1.0); std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J2, -deg2rad(5), 52.0, 1.5, 0.0, 0.4);          std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J4,  M_PI/2.0);                                  std::this_thread::sleep_for(milliseconds(500));

  const double d_j5   = bump_to_limit(openarm, arm,  JointID::J5);                    std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J5, -mech_lim[(int)JointID::J5].second);         std::this_thread::sleep_for(milliseconds(500));

  const double d_j6   = bump_to_limit(openarm, arm,  JointID::J6);                    std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J6, -mech_lim[(int)JointID::J6].second);         std::this_thread::sleep_for(milliseconds(500));

  const double d_j7   = bump_to_limit(openarm, arm,  JointID::J7);                    std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J7, -mech_lim[(int)JointID::J7].second);         std::this_thread::sleep_for(milliseconds(500));

  const double d_j2   = bump_to_limit(openarm, arm,  JointID::J2, -0.2);              std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J2,  deg2rad(10), 180.0, 2.0, 0.0, 0.9);         std::this_thread::sleep_for(milliseconds(500));

  const double d_j1   = bump_to_limit(openarm, arm,  JointID::J1, -0.2, 180.0, 2.1);  std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J1,  deg2rad(80));                                std::this_thread::sleep_for(milliseconds(500));

  interpolate(openarm, arm, JointID::J4, -M_PI/2.0);                                   std::this_thread::sleep_for(milliseconds(2500));

  // Ideal angles
  R.ideal[6] =  M_PI/2.0;   // J7
  R.ideal[5] =  M_PI/4.0;   // J6
  R.ideal[4] =  M_PI/2.0;   // J5
  R.ideal[3] =  0.0;        // J4
  R.ideal[2] =  M_PI/2.0;   // J3
  R.ideal[1] =  deg2rad(-10.0); // J2
  R.ideal[0] =  deg2rad(-80.0); // J1

  R.deltas[0] = d_j1; R.deltas[1] = d_j2; R.deltas[2] = d_j3; R.deltas[3] = d_j4;
  R.deltas[4] = d_j5; R.deltas[5] = d_j6; R.deltas[6] = d_j7; R.deltas[7] = d_grip;
  return R;
}

static SequenceResult run_left_sequence(oacs::OpenArm& openarm,
                                        dm::DMDeviceCollection& arm,
                                        dm::DMDeviceCollection& grip)
{
  SequenceResult R{};

  std::cout << "Running LEFT arm zero-pos sequence\n";
  const double d_grip = bump_to_limit(openarm, grip, JointID::GRIPPER);               std::this_thread::sleep_for(milliseconds(500));
  std::cout << "Gripper limit hit\n";
  const double d_j4   = bump_to_limit(openarm, arm,  JointID::J4, -0.2);              std::this_thread::sleep_for(milliseconds(500));

  interpolate(openarm, arm, JointID::J2, -deg2rad(5), 52.0, 1.5, 0.0, 0.4);           std::this_thread::sleep_for(milliseconds(500));
  const double d_j3   = bump_to_limit(openarm, arm,  JointID::J3);                    std::this_thread::sleep_for(milliseconds(500));

  interpolate(openarm, arm, JointID::J3, -mech_lim[(int)JointID::J3].second, 52.0, 1.5, 0.0, 1.0); std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J2,  deg2rad(5),  52.0, 1.5, 0.0, 0.4);          std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J4,  M_PI/2.0);                                  std::this_thread::sleep_for(milliseconds(500));

  const double d_j5   = bump_to_limit(openarm, arm,  JointID::J5);                    std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J5, -mech_lim[(int)JointID::J5].second);         std::this_thread::sleep_for(milliseconds(500));

  const double d_j6   = bump_to_limit(openarm, arm,  JointID::J6);                    std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J6, -mech_lim[(int)JointID::J6].second);         std::this_thread::sleep_for(milliseconds(500));

  const double d_j7   = bump_to_limit(openarm, arm,  JointID::J7);                    std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J7, -mech_lim[(int)JointID::J7].second);         std::this_thread::sleep_for(milliseconds(500));

  const double d_j2   = bump_to_limit(openarm, arm,  JointID::J2);                    std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J2, -deg2rad(10), 180.0, 2.0, 0.0, 0.9);         std::this_thread::sleep_for(milliseconds(500));

  const double d_j1   = bump_to_limit(openarm, arm,  JointID::J1, +0.2, 180.0, 2.1);  std::this_thread::sleep_for(milliseconds(500));
  interpolate(openarm, arm, JointID::J1, -deg2rad(80));                                std::this_thread::sleep_for(milliseconds(500));

  interpolate(openarm, arm, JointID::J4, -M_PI/2.0);                                   std::this_thread::sleep_for(milliseconds(2500));

  // Ideal angles
  R.ideal[6] =  M_PI/2.0;    // J7
  R.ideal[5] =  M_PI/4.0;    // J6
  R.ideal[4] =  M_PI/2.0;    // J5
  R.ideal[3] =  0.0;         // J4
  R.ideal[2] =  M_PI/2.0;    // J3
  R.ideal[1] =  deg2rad(10); // J2
  R.ideal[0] =  deg2rad(80); // J1

  R.deltas[0] = d_j1; R.deltas[1] = d_j2; R.deltas[2] = d_j3; R.deltas[3] = d_j4;
  R.deltas[4] = d_j5; R.deltas[5] = d_j6; R.deltas[6] = d_j7; R.deltas[7] = d_grip;
  return R;
}

// Worker thread to collect torque data from motors
class MotorDataCollector : public QThread {
    Q_OBJECT

public:
    MotorDataCollector(oacs::OpenArm* openarm) : openarm_(openarm) {}

signals:
    void torqueDataReceived(int motorIndex, double time, double torque);

protected:
    void run() override {
        auto start_time = std::chrono::steady_clock::now();
        
        while (!isInterruptionRequested()) {
            openarm_->recv_all();
            auto motors = openarm_->get_arm().get_motors();
            
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time).count();
            
            for (size_t i = 0; i < motors.size(); ++i) {
                double torque = motors[i].get_torque();
                emit torqueDataReceived(static_cast<int>(i), elapsed, torque);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // ~100 Hz
        }
    }

private:
    oacs::OpenArm* openarm_;
};

// Calibration worker thread
class CalibrationWorker : public QThread {
    Q_OBJECT

public:
    CalibrationWorker(oacs::OpenArm* openarm, const std::string& arm_side)
        : openarm_(openarm), arm_side_(arm_side) {}

signals:
    void calibrationFinished(int result);
    void calibrationError(const QString& error);

protected:
    void run() override {
        try {
            dm::DMDeviceCollection& arm = openarm_->get_arm();
            dm::DMDeviceCollection& gripper = openarm_->get_gripper();

            const auto am = arm.get_motors();
            const auto gm = gripper.get_motors();

            // Arm hold params
            std::vector<dm::MITParam> arm_params;
            arm_params.reserve(am.size());

            const double kp_list[7] = {300, 300, 150, 150, 40, 40, 30};
            const double kd_list[7] = {2.5, 2.5, 2.5, 2.5, 0.8, 0.8, 0.8};

            for (size_t i = 0; i < am.size() && i < 7; ++i) {
                dm::MITParam p{};
                p.kp  = kp_list[i];
                p.kd  = kd_list[i];
                p.q   = am[i].get_position();
                p.dq  = 0.0;
                p.tau = 0.0;
                arm_params.push_back(p);
            }
            arm.mit_control_all(arm_params);

            // Gripper hold params
            std::vector<dm::MITParam> gparams;
            {
                dm::MITParam p{};
                p.kp  = 10.0;
                p.kd  = 0.9;
                p.q   = gm[0].get_position();
                p.dq  = 0.0;
                p.tau = 0.0;
                gparams.push_back(p);
            }
            gripper.mit_control_all(gparams);

            openarm_->recv_all();
            std::cout << "Initial positions held.\n";

            std::vector<double> initial_arm_q(7, 0.0);
            for (int i = 0; i < 7; ++i) initial_arm_q[i] = arm.get_motors()[i].get_position();
            const double initial_grip_q = gripper.get_motors()[0].get_position();

            SequenceResult seq{};
            if (arm_side_ == "right_arm") {
                seq = run_right_sequence(*openarm_, arm, gripper);
            } else {
                seq = run_left_sequence(*openarm_, arm, gripper);
            }

            double ideal_delta_map[7] = {0,0,0,0,0,0,0};
            for (int j = 0; j < 7; ++j) {
                const auto jid = static_cast<JointID>(j);
                const double val = calc_delta_to_zero_pos_joint(
                    initial_arm_q[j],
                    seq.ideal[j],
                    seq.deltas[j],
                    jid
                );
                ideal_delta_map[j] = val;
            }

            std::vector<double> arm_goal_abs(7, 0.0);
            for (int i = 0; i < 7; ++i) arm_goal_abs[i] = initial_arm_q[i] + ideal_delta_map[i];

            openarm_->disable_all();
            openarm_->recv_all();
            openarm_->set_zero_all();
            openarm_->recv_all();
            std::cout << "Wrote zero position to arm.\n";

            (void)initial_grip_q;
            emit calibrationFinished(0);
        } catch (const std::exception& e) {
            emit calibrationError(QString::fromStdString(e.what()));
        } catch (...) {
            emit calibrationError("Unknown exception");
        }
    }

private:
    oacs::OpenArm* openarm_;
    std::string arm_side_;
};

// Main window with torque plots
class TorquePlotWindow : public QMainWindow {
    Q_OBJECT

public:
    TorquePlotWindow(oacs::OpenArm* openarm, QWidget* parent = nullptr)
        : QMainWindow(parent), openarm_(openarm), collector_(nullptr) {
        setupUI();
        setupDataCollector();
    }

    ~TorquePlotWindow() {
        if (collector_) {
            collector_->requestInterruption();
            collector_->wait();
            delete collector_;
        }
    }

    void saveAllPlotsToPdf(const QString& basePath = QString()) {
        QString path = basePath.isEmpty() ? "graphs" : basePath;
        
        // Create graphs directory if it doesn't exist
        QDir dir;
        if (!dir.exists(path)) {
            if (!dir.mkpath(path)) {
                qWarning() << "Failed to create directory:" << path;
                return;
            }
        }

        // Generate timestamp for filename
        QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
        
        // Save each plot as a separate PDF
        for (int i = 0; i < static_cast<int>(plots_.size()); ++i) {
            QString filename = QString("%1/motor_%2_torque_%3.pdf")
                                  .arg(path)
                                  .arg(i + 1)
                                  .arg(timestamp);
            
            if (plots_[i]->savePdf(filename, 1200, 400)) {
                qDebug() << "Saved plot to:" << filename;
            } else {
                qWarning() << "Failed to save plot to:" << filename;
            }
        }
        
        // Also save a combined plot (all motors in one PDF)
        QString combinedFilename = QString("%1/all_motors_torque_%2.pdf")
                                      .arg(path)
                                      .arg(timestamp);
        saveCombinedPlot(combinedFilename);
    }

private:
    void saveCombinedPlot(const QString& filename) {
        // Create a temporary plot with all motors
        QCustomPlot* combinedPlot = new QCustomPlot;
        
        // Add all graphs to the combined plot
        for (int i = 0; i < static_cast<int>(plots_.size()); ++i) {
            combinedPlot->addGraph();
            QColor color = QColor::fromHsv(i * 360 / 7, 255, 255);
            combinedPlot->graph(i)->setPen(QPen(color, 2));
            combinedPlot->graph(i)->setName(QString("Motor %1").arg(i + 1));
            
            // Copy data from individual plot
            auto sourceData = plots_[i]->graph(0)->data();
            for (auto it = sourceData->begin(); it != sourceData->end(); ++it) {
                combinedPlot->graph(i)->addData(it->key, it->value);
            }
        }
        
        // Configure axes
        combinedPlot->xAxis->setLabel("Time (s)");
        combinedPlot->yAxis->setLabel("Torque (Nm)");
        combinedPlot->legend->setVisible(true);
        combinedPlot->legend->setFont(QFont("Helvetica", 9));
        
        // Set axis ranges to fit all data
        combinedPlot->rescaleAxes();
        
        // Save to PDF
        if (combinedPlot->savePdf(filename, 1200, 800)) {
            qDebug() << "Saved combined plot to:" << filename;
        } else {
            qWarning() << "Failed to save combined plot to:" << filename;
        }
        
        delete combinedPlot;
    }

private slots:
    void onTorqueData(int motorIndex, double time, double torque) {
        if (motorIndex < 0 || motorIndex >= static_cast<int>(plots_.size())) {
            return;
        }

        QCustomPlot* plot = plots_[motorIndex];
        plot->graph(0)->addData(time, torque);

        // Remove old data beyond 10 seconds
        plot->graph(0)->data()->removeBefore(time - 10.0);

        // Update X-axis range to show last 10 seconds
        plot->xAxis->setRange(time - 10.0, time);

        // Auto-rescale Y-axis
        plot->graph(0)->rescaleValueAxis();

        // Replot
        plot->replot();
    }

private:
    void setupUI() {
        auto* centralWidget = new QWidget;
        auto* layout = new QVBoxLayout(centralWidget);

        // Create a plot for each motor (7 motors)
        for (int i = 0; i < 7; ++i) {
            auto* plot = new QCustomPlot;
            plot->addGraph();
            
            // Set distinct color for each motor using HSV
            QColor color = QColor::fromHsv(i * 360 / 7, 255, 255);
            plot->graph(0)->setPen(QPen(color, 2));
            
            // Configure axes
            plot->xAxis->setLabel("Time (s)");
            plot->yAxis->setLabel(QString("Motor %1 Torque (Nm)").arg(i + 1));
            
            // Enable zoom and pan
            plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
            
            // Set initial X-axis range (will be updated dynamically)
            plot->xAxis->setRange(0, 10);
            
            plots_.push_back(plot);
            layout->addWidget(plot);
        }

        setCentralWidget(centralWidget);
        setWindowTitle("Zero Position Calibration - Torque Monitor");
        resize(1200, 1000);
    }

    void setupDataCollector() {
        collector_ = new MotorDataCollector(openarm_);
        connect(collector_, &MotorDataCollector::torqueDataReceived,
                this, &TorquePlotWindow::onTorqueData);
        collector_->start();
    }

    oacs::OpenArm* openarm_;
    MotorDataCollector* collector_;
    std::vector<QCustomPlot*> plots_;
};

struct Args {
  std::string canport = "can0";
  std::string arm_side = "right_arm"; // or "left_arm"
  bool show_torques = false;
};

static bool parse_args(int argc, char** argv, Args& out) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--canport" && i + 1 < argc) {
      out.canport = argv[++i];
    } else if (a == "--arm-side" && i + 1 < argc) {
      out.arm_side = argv[++i];
    } else if (a == "--show-torques") {
      out.show_torques = true;
    } else if (a == "-h" || a == "--help") {
      std::cout << "usage: " << argv[0] << " [--canport CANPORT] [--arm-side {right_arm,left_arm}] [--show-torques]\n";
      return false;
    } else {
      std::cerr << "unrecognized argument: " << a << "\n";
      return false;
    }
  }
  if (out.arm_side != "right_arm" && out.arm_side != "left_arm") {
    std::cerr << "Invalid --arm-side: " << out.arm_side << " (expected right_arm or left_arm)\n";
    return false;
  }
  return true;
}

int main(int argc, char** argv) {
  Args args;
  if (!parse_args(argc, argv, args)) return 2;

  // Initialize Qt if showing torques
  QApplication* app = nullptr;
  if (args.show_torques) {
    app = new QApplication(argc, argv);
  }

  try {
    oacs::OpenArm openarm(args.canport, /*enable_fd=*/false);

    openarm.init_arm_motors(
      {dm::MotorType::DM8009, dm::MotorType::DM8009, dm::MotorType::DM4340, dm::MotorType::DM4340,
       dm::MotorType::DM4310, dm::MotorType::DM4310, dm::MotorType::DM4310},
      {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},
      {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17}
    );
    openarm.init_gripper_motor(dm::MotorType::DM4310, 0x08, 0x18);

    openarm.set_callback_mode_all(dm::CallbackMode::STATE);

    std::cout << "Enabling...\n";
    openarm.enable_all();
    std::this_thread::sleep_for(milliseconds(100));
    std::cout << "Enabled.\n";

    openarm.recv_all();
    std::cout << "Holding current pose...\n";

    if (args.show_torques) {
      // Qt mode: show torque plots and run calibration in background
      TorquePlotWindow window(&openarm);
      window.show();

      CalibrationWorker* worker = new CalibrationWorker(&openarm, args.arm_side);
      QObject::connect(worker, &CalibrationWorker::calibrationFinished,
                       app, [app, &window](int result) {
                         std::cout << "[INFO] Calibration completed.\n";
                         
                         // Save all plots to PDF before quitting
                         std::cout << "[INFO] Saving torque graphs to PDF...\n";
                         window.saveAllPlotsToPdf("graphs");
                         std::cout << "[INFO] Graphs saved to graphs/ directory.\n";
                         
                         if (result == 0) {
                           app->quit();
                         }
                       });
      QObject::connect(worker, &CalibrationWorker::calibrationError,
                       app, [app, &window](const QString& error) {
                         std::cerr << "[ERROR] " << error.toStdString() << "\n";
                         
                         // Save plots even on error (might be useful for debugging)
                         std::cout << "[INFO] Saving torque graphs to PDF before exit...\n";
                         window.saveAllPlotsToPdf("graphs");
                         
                         app->quit();
                       });
      worker->start();

      // Run Qt event loop
      int result = app->exec();

      // Wait for calibration to finish
      worker->wait();
      delete worker;
      delete app;

      return result;
    } else {
      // Non-Qt mode: run calibration normally
      dm::DMDeviceCollection& arm     = openarm.get_arm();
      dm::DMDeviceCollection& gripper = openarm.get_gripper();

      const auto am = arm.get_motors();
      const auto gm = gripper.get_motors();

      // Arm hold params
      std::vector<dm::MITParam> arm_params;
      arm_params.reserve(am.size());

      const double kp_list[7] = {300, 300, 150, 150, 40, 40, 30};
      const double kd_list[7] = {2.5, 2.5, 2.5, 2.5, 0.8, 0.8, 0.8};

      for (size_t i = 0; i < am.size() && i < 7; ++i) {
        dm::MITParam p{};
        p.kp  = kp_list[i];
        p.kd  = kd_list[i];
        p.q   = am[i].get_position();
        p.dq  = 0.0;
        p.tau = 0.0;
        arm_params.push_back(p);
      }
      arm.mit_control_all(arm_params);

      // Gripper hold params
      std::vector<dm::MITParam> gparams;
      {
        dm::MITParam p{};
        p.kp  = 10.0;
        p.kd  = 0.9;
        p.q   = gm[0].get_position();
        p.dq  = 0.0;
        p.tau = 0.0;
        gparams.push_back(p);
      }
      gripper.mit_control_all(gparams);

      openarm.recv_all();
      std::cout << "Initial positions held.\n";

      std::vector<double> initial_arm_q(7, 0.0);
      for (int i = 0; i < 7; ++i) initial_arm_q[i] = arm.get_motors()[i].get_position();
      const double initial_grip_q = gripper.get_motors()[0].get_position();

      SequenceResult seq{};
      if (args.arm_side == "right_arm") {
        seq = run_right_sequence(openarm, arm, gripper);
      } else {
        seq = run_left_sequence(openarm, arm, gripper);
      }

      double ideal_delta_map[7] = {0,0,0,0,0,0,0};
      for (int j = 0; j < 7; ++j) {
        const auto jid = static_cast<JointID>(j);
        const double val = calc_delta_to_zero_pos_joint(
          initial_arm_q[j],
          seq.ideal[j],
          seq.deltas[j],
          jid
        );
        ideal_delta_map[j] = val;
        // std::cout << "ideal[" << j << "] = " << val << " rad\n";
      }

      std::vector<double> arm_goal_abs(7, 0.0);
      for (int i = 0; i < 7; ++i) arm_goal_abs[i] = initial_arm_q[i] + ideal_delta_map[i];

      openarm.disable_all();
      openarm.recv_all();
      openarm.set_zero_all();
      openarm.recv_all();
      std::cout << "Wrote zero position to arm.\n";

      (void)initial_grip_q;
      std::cout << "[INFO] Motors disabled, exiting safely.\n";
      return 0;
    }

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] " << e.what() << "\n";
    if (app) delete app;
    return 1;
  } catch (...) {
    std::cerr << "[ERROR] Unknown exception\n";
    if (app) delete app;
    return 1;
  }
}

#include "zero_position_calibration.moc"