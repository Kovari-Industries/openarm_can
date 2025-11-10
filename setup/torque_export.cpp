#include "torque_export.hpp"

#include <qcustomplot.h>
#include <QDir>
#include <QDebug>
#include <QColor>
#include <QPen>
#include <QFont>
#include <QFile>
#include <QTextStream>
#include <QSet>
#include <QList>
#include <algorithm>
#include <cmath>

namespace torque_export {

std::vector<Format> parseExportFormats(const QString& formatString) {
    std::vector<Format> formats;
    QString lower = formatString.toLower();
    
    if (lower == "all") {
        formats = {Format::PDF, Format::PNG, Format::CSV};
        return formats;
    }
    
    QStringList parts = lower.split(',', Qt::SkipEmptyParts);
    for (const QString& part : parts) {
        QString trimmed = part.trimmed();
        if (trimmed == "pdf") {
            formats.push_back(Format::PDF);
        } else if (trimmed == "png") {
            formats.push_back(Format::PNG);
        } else if (trimmed == "csv") {
            formats.push_back(Format::CSV);
        }
    }
    
    return formats;
}

QCustomPlot* createStackedPlot(const std::vector<QCustomPlot*>& plots, int width, int height) {
    QCustomPlot* stackedPlot = new QCustomPlot;
    
    // Find the overall time range from all plots
    double globalXMin = 0.0, globalXMax = 0.0;
    bool hasAnyData = false;
    
    for (size_t i = 0; i < plots.size(); ++i) {
        auto data = plots[i]->graph(0)->data();
        if (data->size() > 0) {
            double xMin = data->constBegin()->key;
            double xMax = (data->constEnd() - 1)->key;
            
            if (!hasAnyData) {
                globalXMin = xMin;
                globalXMax = xMax;
                hasAnyData = true;
            } else {
                globalXMin = std::min(globalXMin, xMin);
                globalXMax = std::max(globalXMax, xMax);
            }
        }
    }
    
    if (!hasAnyData) {
        qWarning() << "No data to save in stacked plot";
        return nullptr;
    }
    
    // Create axis rects for each motor, stacked vertically
    QList<QCPAxisRect*> axisRects;
    for (size_t i = 0; i < plots.size(); ++i) {
        QCPAxisRect* rect = new QCPAxisRect(stackedPlot);
        axisRects.append(rect);
        
        if (i == 0) {
            stackedPlot->plotLayout()->clear();
            stackedPlot->plotLayout()->addElement(0, 0, rect);
        } else {
            stackedPlot->plotLayout()->addElement(static_cast<int>(i), 0, rect);
        }
        
        QCPGraph* graph = stackedPlot->addGraph(rect->axis(QCPAxis::atBottom), 
                                                rect->axis(QCPAxis::atLeft));
        
        auto sourceData = plots[i]->graph(0)->data();
        for (auto it = sourceData->begin(); it != sourceData->end(); ++it) {
            graph->addData(it->key, it->value);
        }
        
        QColor color = QColor::fromHsv(static_cast<int>(i) * 360 / static_cast<int>(plots.size()), 255, 255);
        graph->setPen(QPen(color, 2));
        
        rect->axis(QCPAxis::atBottom)->setLabel(i == plots.size() - 1 ? "Time (s)" : "");
        rect->axis(QCPAxis::atLeft)->setLabel(QString("Motor %1 (Nm)").arg(i + 1));
        rect->axis(QCPAxis::atBottom)->setRange(globalXMin, globalXMax);
        graph->rescaleValueAxis(false, true);
        rect->axis(QCPAxis::atTop)->setVisible(false);
        rect->axis(QCPAxis::atRight)->setVisible(false);
    }
    
    stackedPlot->plotLayout()->setRowSpacing(5);
    stackedPlot->resize(width, height);
    
    return stackedPlot;
}

void saveStackedPlot(const std::vector<QCustomPlot*>& plots, 
                     const QString& basePath,
                     const QString& timestamp,
                     const std::vector<Format>& formats,
                     int width) {
    int height = 400 * static_cast<int>(plots.size());
    QCustomPlot* stackedPlot = createStackedPlot(plots, width, height);
    
    if (!stackedPlot) {
        return;
    }
    
    for (Format format : formats) {
        QString filename;
        bool success = false;
        
        switch (format) {
            case Format::PDF: {
                filename = QString("%1/all_motors_stacked_%2.pdf")
                              .arg(basePath)
                              .arg(timestamp);
                success = stackedPlot->savePdf(filename, width, height);
                break;
            }
            case Format::PNG: {
                filename = QString("%1/all_motors_stacked_%2.png")
                              .arg(basePath)
                              .arg(timestamp);
                success = stackedPlot->savePng(filename, width, height, 2.0, -1, 300);
                break;
            }
            case Format::CSV:
                // CSV is handled separately
                continue;
            case Format::ALL:
                // Should not happen if parsing is correct
                break;
        }
        
        if (success) {
            qDebug() << "Saved stacked plot to:" << filename;
        } else {
            qWarning() << "Failed to save stacked plot to:" << filename;
        }
    }
    
    delete stackedPlot;
}

void saveIndividualPlots(const std::vector<QCustomPlot*>& plots,
                         const QString& basePath,
                         const QString& timestamp,
                         const std::vector<Format>& formats,
                         int width) {
    for (size_t i = 0; i < plots.size(); ++i) {
        // Temporarily set X-axis to show all data
        double xMin = 0.0, xMax = 0.0;
        bool hasData = false;
        auto data = plots[i]->graph(0)->data();
        if (data->size() > 0) {
            xMin = data->constBegin()->key;
            xMax = (data->constEnd() - 1)->key;
            hasData = true;
        }
        
        if (hasData) {
            plots[i]->xAxis->setRange(xMin, xMax);
            plots[i]->replot();
        }
        
        for (Format format : formats) {
            QString filename;
            bool success = false;
            
            switch (format) {
                case Format::PDF: {
                    filename = QString("%1/motor_%2_torque_%3.pdf")
                                  .arg(basePath)
                                  .arg(i + 1)
                                  .arg(timestamp);
                    success = plots[i]->savePdf(filename, width, 400);
                    break;
                }
                case Format::PNG: {
                    filename = QString("%1/motor_%2_torque_%3.png")
                                  .arg(basePath)
                                  .arg(i + 1)
                                  .arg(timestamp);
                    success = plots[i]->savePng(filename, width, 400, 2.0, -1, 300);
                    break;
                }
                case Format::CSV:
                    // CSV is handled separately
                    continue;
                case Format::ALL:
                    break;
            }
            
            if (success) {
                qDebug() << "Saved plot to:" << filename;
            }
        }
    }
}

void saveDataToCsv(const std::vector<QCustomPlot*>& plots, const QString& filename) {
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning() << "Failed to open CSV file for writing:" << filename;
        return;
    }
    
    QTextStream out(&file);
    
    // Write header
    out << "Time (s)";
    for (size_t i = 0; i < plots.size(); ++i) {
        out << ",Motor " << (i + 1) << " Torque (Nm)";
    }
    out << "\n";
    
    // Collect all unique time points from all motors
    QSet<double> allTimes;
    for (size_t i = 0; i < plots.size(); ++i) {
        auto data = plots[i]->graph(0)->data();
        for (auto it = data->constBegin(); it != data->constEnd(); ++it) {
            allTimes.insert(it->key);
        }
    }
    
    // Convert to sorted list
    QList<double> sortedTimes = allTimes.values();
    std::sort(sortedTimes.begin(), sortedTimes.end());
    
    // Write data rows
    for (double time : sortedTimes) {
        out << time;
        for (size_t i = 0; i < plots.size(); ++i) {
            auto data = plots[i]->graph(0)->data();
            auto it = data->findBegin(time, false);
            if (it != data->constEnd() && qAbs(it->key - time) < 0.001) {
                out << "," << it->value;
            } else {
                out << ","; // Empty if no data at this time
            }
        }
        out << "\n";
    }
    
    file.close();
    qDebug() << "Saved CSV data to:" << filename;
}

} // namespace torque_export

