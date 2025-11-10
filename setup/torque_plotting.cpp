#include "torque_plotting.hpp"
#include "torque_export.hpp"

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QWidget>
#include <QColor>
#include <QPen>
#include <QDir>
#include <QDateTime>
#include <QDebug>
#include <QFont>
#include <qcustomplot.h>

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <cmath>

namespace oacs = openarm::can::socket;
namespace dm = openarm::damiao_motor;

using std::chrono::milliseconds;

// MotorDataCollector implementation
MotorDataCollector::MotorDataCollector(oacs::OpenArm* openarm) : openarm_(openarm) {}

void MotorDataCollector::run() {
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

// TorquePlotWindow implementation
TorquePlotWindow::TorquePlotWindow(oacs::OpenArm* openarm, QWidget* parent)
    : QMainWindow(parent), openarm_(openarm), collector_(nullptr) {
    setupUI();
    setupDataCollector();
}

TorquePlotWindow::~TorquePlotWindow() {
    if (collector_) {
        collector_->requestInterruption();
        collector_->wait();
        delete collector_;
    }
}

void TorquePlotWindow::saveAllPlots(const QString& basePath, const QString& exportFormats) {
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
    
    // Parse export formats
    auto formats = torque_export::parseExportFormats(exportFormats);
    
    // Save plots based on selected formats
    torque_export::saveStackedPlot(plots_, path, timestamp, formats);
    torque_export::saveIndividualPlots(plots_, path, timestamp, formats);
    
    // Handle CSV export separately
    for (auto format : formats) {
        if (format == torque_export::Format::CSV) {
            QString csvFilename = QString("%1/torque_data_%2.csv")
                                     .arg(path)
                                     .arg(timestamp);
            torque_export::saveDataToCsv(plots_, csvFilename);
            break;
        }
    }
}

void TorquePlotWindow::onTorqueData(int motorIndex, double time, double torque) {
    if (motorIndex < 0 || motorIndex >= static_cast<int>(plots_.size())) {
        return;
    }

    QCustomPlot* plot = plots_[motorIndex];
    plot->graph(0)->addData(time, torque);

    // Keep all data (don't remove old data) - user wants full 42 seconds
    // Update X-axis range to show last 10 seconds for live viewing
    // But keep all data in memory for saving
    double xMin = std::max(0.0, time - 10.0);
    plot->xAxis->setRange(xMin, time);

    // Auto-rescale Y-axis
    plot->graph(0)->rescaleValueAxis();

    // Replot
    plot->replot();
}

void TorquePlotWindow::setupUI() {
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

void TorquePlotWindow::setupDataCollector() {
    collector_ = new MotorDataCollector(openarm_);
    connect(collector_, &MotorDataCollector::torqueDataReceived,
            this, &TorquePlotWindow::onTorqueData);
    collector_->start();
}

