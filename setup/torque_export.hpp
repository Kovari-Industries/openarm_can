#pragma once

#include <QString>
#include <vector>

class QCustomPlot;

namespace torque_export {

// Export formats
enum class Format {
    PDF,
    PNG,
    CSV,
    ALL
};

// Parse export format string (e.g., "pdf,png,csv" or "all")
std::vector<Format> parseExportFormats(const QString& formatString);

// Save stacked plot (all motors vertically stacked)
void saveStackedPlot(const std::vector<QCustomPlot*>& plots, 
                     const QString& basePath,
                     const QString& timestamp,
                     const std::vector<Format>& formats,
                     int width = 2400);

// Save individual motor plots
void saveIndividualPlots(const std::vector<QCustomPlot*>& plots,
                         const QString& basePath,
                         const QString& timestamp,
                         const std::vector<Format>& formats,
                         int width = 2400);

// Save raw data as CSV
void saveDataToCsv(const std::vector<QCustomPlot*>& plots,
                   const QString& filename);

// Helper: Create stacked plot QCustomPlot object
QCustomPlot* createStackedPlot(const std::vector<QCustomPlot*>& plots, 
                               int width, int height);

} // namespace torque_export

