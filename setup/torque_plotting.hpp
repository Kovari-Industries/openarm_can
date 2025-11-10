#pragma once

#include <QMainWindow>
#include <QThread>
#include <QString>
#include <vector>

#include <openarm/can/socket/openarm.hpp>

namespace openarm::can::socket {
    class OpenArm;
}

class QCustomPlot;

// Worker thread to collect torque data from motors
class MotorDataCollector : public QThread {
    Q_OBJECT

public:
    MotorDataCollector(openarm::can::socket::OpenArm* openarm);
    ~MotorDataCollector() = default;

signals:
    void torqueDataReceived(int motorIndex, double time, double torque);

protected:
    void run() override;

private:
    openarm::can::socket::OpenArm* openarm_;
};

// Note: CalibrationWorker is defined in zero_position_calibration.cpp
// because it uses calibration-specific functions

// Main window with torque plots
class TorquePlotWindow : public QMainWindow {
    Q_OBJECT

public:
    TorquePlotWindow(openarm::can::socket::OpenArm* openarm, QWidget* parent = nullptr);
    ~TorquePlotWindow();

    void saveAllPlots(const QString& basePath = QString(), 
                      const QString& exportFormats = "pdf,png,csv");

private slots:
    void onTorqueData(int motorIndex, double time, double torque);

private:
    void setupUI();
    void setupDataCollector();

    openarm::can::socket::OpenArm* openarm_;
    MotorDataCollector* collector_;
    std::vector<QCustomPlot*> plots_;
};

