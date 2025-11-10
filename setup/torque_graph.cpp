#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QWidget>
#include <QThread>
#include <QColor>

#include <qcustomplot.h>

#include <openarm/can/socket/openarm.hpp>

namespace oa  = openarm;
namespace oacs = openarm::can::socket;
namespace dm  = openarm::damiao_motor;

using std::chrono::milliseconds;

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
        setWindowTitle("OpenArm Motor Torque Monitor");
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
};

static bool parse_args(int argc, char** argv, Args& out) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--canport" && i + 1 < argc) {
            out.canport = argv[++i];
        } else if (a == "-h" || a == "--help") {
            std::cout << "usage: " << argv[0] << " [--canport CANPORT]\n";
            return false;
        } else {
            std::cerr << "unrecognized argument: " << a << "\n";
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    Args args;
    if (!parse_args(argc, argv, args)) {
        return 2;
    }

    try {
        // Initialize OpenArm
        oacs::OpenArm openarm(args.canport, /*enable_fd=*/false);

        openarm.init_arm_motors(
            {dm::MotorType::DM8009, dm::MotorType::DM8009, dm::MotorType::DM4340,
             dm::MotorType::DM4340, dm::MotorType::DM4310, dm::MotorType::DM4310,
             dm::MotorType::DM4310},
            {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},
            {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17});

        openarm.init_gripper_motor(dm::MotorType::DM4310, 0x08, 0x18);

        openarm.set_callback_mode_all(dm::CallbackMode::STATE);

        std::cout << "Enabling motors...\n";
        openarm.enable_all();
        std::this_thread::sleep_for(milliseconds(100));
        std::cout << "Motors enabled. Starting torque monitoring...\n";

        // Initial receive to populate motor states
        openarm.recv_all();

        // Create and show the plot window
        TorquePlotWindow window(&openarm);
        window.show();

        // Run Qt event loop
        int result = app.exec();

        // Cleanup: disable motors
        std::cout << "Disabling motors...\n";
        openarm.disable_all();
        openarm.recv_all();

        return result;

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception\n";
        return 1;
    }
}

#include "torque_graph.moc"

