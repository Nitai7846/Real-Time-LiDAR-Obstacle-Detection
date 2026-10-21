#include "lidar/performance_report.h"

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double eps = 1e-9) {
    if (std::abs(actual - expected) > eps) throw std::runtime_error("value differs from expected");
}
void emptySummary() {
    check(!lidar::summarize({}).has_value(), "empty must have no summary");
}
void singleton() {
    auto s = lidar::summarize({12.5}).value();
    check(s.count == 1, "one sample");
    near(s.median,12.5); near(s.p95,12.5); near(s.mean,12.5);
}
void medianEven() {
    auto s = lidar::summarize({7, 1, 3, 5}).value();
    near(s.median, 4); near(s.mean,4); near(s.minimum,1); near(s.maximum,7);
}
void p95Interpolation() {
    auto s = lidar::summarize({1,2,3,4,5}).value();
    near(s.p95,4.8);
}
void rejectNonfinite() {
    bool threw = false;
    try { (void)lidar::summarize({1.0, std::numeric_limits<double>::infinity()}); }
    catch (const std::invalid_argument&) { threw = true; }
    check(threw, "infinite value should be rejected");
}
void emptyReport() {
    lidar::PerformanceReport report;
    std::ostringstream out;
    report.print(out);
    check(out.str().find("No frames processed") != std::string::npos, "empty report message");
}
void summaryAndMarkdown() {
    lidar::PerformanceReport report;
    for (std::size_t i=0; i<5; ++i) {
        lidar::FrameMetrics row;
        row.frame_index = i;
        row.load_ms = 1.0;
        row.timings.total_ms = double(i+1);
        report.add(row);
    }
    check(report.frameCount() == 5, "frame count");
    std::ostringstream out, markdown;
    report.print(out, 1000.0);
    report.writeMarkdown(markdown, "kit ti/dataset", 1000.0);
    check(out.str().find("Observed wall throughput") != std::string::npos, "wall throughput");
    check(markdown.str().find("| Pipeline total (ms) |") != std::string::npos, "table missing");
    check(markdown.str().find("KITTI object-detection") != std::string::npos, "limits missing");
}
void comparisonSummary() {
    lidar::PerformanceReport report;
    for (int i=0; i<2; ++i) {
        lidar::FrameMetrics row;
        lidar::PclComparison compare;
        compare.custom_ransac_ms = 1.0;
        compare.pcl_ransac_ms = 2.0;
        compare.custom_cluster_count = 3;
        compare.pcl_cluster_count = i ? 2 : 3;
        compare.ground_inlier_jaccard = 0.9;
        row.comparison = compare;
        report.add(row);
    }
    std::ostringstream out;
    report.print(out);
    check(out.str().find("Equal custom/PCL cluster counts: 1/2") != std::string::npos,
          "comparison count not reported");
}
}

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests = {
        {"empty stats",emptySummary}, {"one sample",singleton},
        {"even median",medianEven}, {"p95 interpolation",p95Interpolation},
        {"nonfinite rejected",rejectNonfinite}, {"empty report",emptyReport},
        {"markdown and throughput",summaryAndMarkdown}, {"comparison summary",comparisonSummary}
    };
    std::size_t pass = 0;
    for (const auto& t : tests) {
        try { t.second(); std::cout << "[PASS] " << t.first << '\n'; ++pass; }
        catch (const std::exception& e) { std::cerr << "[FAIL] " << t.first << ": " << e.what() << '\n'; }
    }
    std::cout << pass << "/" << tests.size() << " PerformanceReport tests passed\n";
    return pass == tests.size() ? 0 : 1;
}
