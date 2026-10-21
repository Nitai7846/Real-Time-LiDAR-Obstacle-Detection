#pragma once

#include "lidar/metrics_logger.h"

#include <cstddef>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

namespace lidar {

struct SummaryStats {
    std::size_t count = 0;
    double mean = 0.0;
    double median = 0.0;
    double p95 = 0.0;
    double minimum = 0.0;
    double maximum = 0.0;
};

// Returns nullopt for empty input; p95 uses linear interpolation at index
// (N-1)*0.95 after sorting, as described in the report.
std::optional<SummaryStats> summarize(const std::vector<double>& values);

class PerformanceReport {
public:
    void add(const FrameMetrics& row);
    std::size_t frameCount() const { return frames_.size(); }
    void print(std::ostream& out, double observed_wall_ms = 0.0) const;
    void writeMarkdown(std::ostream& out, const std::string& input_path,
                       double observed_wall_ms = 0.0,
                       const PipelineConfig& config = {},
                       const std::string& invocation = "") const;

private:
    std::vector<FrameMetrics> frames_;
};

} // namespace lidar
