#include "lidar/performance_report.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lidar {
namespace {

double percentile(const std::vector<double>& sorted, double q) {
    const double position = (sorted.size() - 1) * q;
    const auto low = static_cast<std::size_t>(position);
    const auto high = std::min(low + 1, sorted.size() - 1);
    const double frac = position - low;
    return sorted[low] * (1.0 - frac) + sorted[high] * frac;
}

template <typename Access>
std::optional<SummaryStats> statsFor(const std::vector<FrameMetrics>& frames, Access get) {
    std::vector<double> values;
    values.reserve(frames.size());
    for (const auto& frame : frames) {
        auto value = get(frame);
        if (value && std::isfinite(*value)) values.push_back(*value);
    }
    return summarize(values);
}

void reportRow(std::ostream& out, const std::string& name,
               const std::optional<SummaryStats>& s, bool markdown) {
    if (!s) return;
    out << std::fixed << std::setprecision(3);
    if (markdown) {
        out << "| " << name << " | " << s->count << " | " << s->mean << " | "
            << s->median << " | " << s->p95 << " | " << s->maximum << " |\n";
    } else {
        out << "  " << name << ": n=" << s->count << " mean=" << s->mean
            << " median=" << s->median << " p95=" << s->p95
            << " max=" << s->maximum << '\n';
    }
}

} // namespace

std::optional<SummaryStats> summarize(const std::vector<double>& values) {
    if (values.empty()) return std::nullopt;
    for (double v : values) {
        if (!std::isfinite(v)) throw std::invalid_argument("Statistics require finite values");
    }
    auto sorted = values;
    std::sort(sorted.begin(), sorted.end());
    SummaryStats result;
    result.count = sorted.size();
    result.minimum = sorted.front();
    result.maximum = sorted.back();
    result.mean = std::accumulate(sorted.begin(), sorted.end(), 0.0) / sorted.size();
    result.median = percentile(sorted, 0.5);
    result.p95 = percentile(sorted, 0.95);
    return result;
}

void PerformanceReport::add(const FrameMetrics& row) { frames_.push_back(row); }

void PerformanceReport::print(std::ostream& out, double observed_wall_ms) const {
    out << "\n--- Milestone 11 Performance Summary ---\n";
    if (frames_.empty()) { out << "No frames processed.\n"; return; }
    out << "Frames processed: " << frames_.size() << '\n';
    const auto emit = [&](const std::string& name, auto getter) {
        reportRow(out, name, statsFor(frames_, getter), false);
    };
    emit("Load (ms)", [](const auto& r) { return std::optional<double>(r.load_ms); });
    emit("Preprocessing (ms)", [](const auto& r) { return std::optional<double>(r.timings.preprocessing_ms); });
    emit("Custom RANSAC (pipeline ms)", [](const auto& r) { return std::optional<double>(r.timings.ransac_ms); });
    emit("Ground separation (ms)", [](const auto& r) { return std::optional<double>(r.timings.ground_separation_ms); });
    emit("Custom clustering (pipeline ms)", [](const auto& r) { return std::optional<double>(r.timings.clustering_ms); });
    emit("Bounding boxes (ms)", [](const auto& r) { return std::optional<double>(r.timings.bounding_box_ms); });
    emit("Pipeline total (ms)", [](const auto& r) { return std::optional<double>(r.timings.total_ms); });
    emit("Load + pipeline (ms)", [](const auto& r) { return std::optional<double>(r.load_and_pipeline_ms()); });
    const auto combined = statsFor(frames_, [](const auto& r) { return std::optional<double>(r.load_and_pipeline_ms()); });
    if (combined && combined->mean > 0.0) {
        out << "Estimated sequential load+pipeline throughput: "
            << 1000.0 / combined->mean << " frames/s (no GUI, comparison, CSV or console overhead)\n";
    }
    if (combined) {
        out << "100 ms deadline: load+pipeline p95 "
            << (combined->p95 <= 100.0 ? "MEETS" : "EXCEEDS")
            << " target (not a guarantee of sustained 10 Hz).\n";
    }
    if (observed_wall_ms > 0) {
        out << "Observed wall throughput for this run: "
            << frames_.size() * 1000.0 / observed_wall_ms << " frames/s\n";
    }
    emit("Custom RANSAC (isolated ms)", [](const auto& r) { return r.comparison ? std::optional<double>(r.comparison->custom_ransac_ms) : std::nullopt; });
    emit("PCL RANSAC (isolated ms)", [](const auto& r) { return r.comparison ? std::optional<double>(r.comparison->pcl_ransac_ms) : std::nullopt; });
    emit("Custom clustering (isolated ms)", [](const auto& r) { return r.comparison ? std::optional<double>(r.comparison->custom_clustering_ms) : std::nullopt; });
    emit("PCL clustering (isolated ms)", [](const auto& r) { return r.comparison ? std::optional<double>(r.comparison->pcl_clustering_ms) : std::nullopt; });
    emit("Ground inlier Jaccard", [](const auto& r) { return r.comparison ? r.comparison->ground_inlier_jaccard : std::nullopt; });
    emit("Plane normal angle (degrees)", [](const auto& r) { return r.comparison ? r.comparison->plane_normal_angle_deg : std::nullopt; });
    emit("Mean best cluster Jaccard", [](const auto& r) { return r.comparison ? r.comparison->mean_best_cluster_jaccard : std::nullopt; });
    std::size_t matching_cluster_counts = 0, compared = 0;
    for (const auto& r : frames_) if (r.comparison) {
        ++compared;
        if (r.comparison->custom_cluster_count == r.comparison->pcl_cluster_count) {
            ++matching_cluster_counts;
        }
    }
    if (compared) out << "Equal custom/PCL cluster counts: " << matching_cluster_counts << '/' << compared << " frames\n";
    out << "Note: 10 FPS requires <= 100 ms/frame sustained. Set a Release build for credible benchmarks.\n";
}

void PerformanceReport::writeMarkdown(std::ostream& out, const std::string& path,
                                      double observed_wall_ms,
                                      const PipelineConfig& config,
                                      const std::string& invocation) const {
    out << "# LiDAR Pipeline Performance — Milestone 11\n\n"
           "Input: `" << path << "`  \n"
           "Frames: " << frames_.size() << "  \n"
           "Record compiler optimization flags and OS/PCL versions alongside these measurements.\n\n";
    if (!invocation.empty()) {
        out << "Invocation: `" << invocation << "`\n\n";
    }
    out << "## Configuration\n\n"
        << "- Preprocessing voxel size (m): " << config.preprocessing.voxel_size << "\n"
        << "- ROI x/y/z (m): [" << config.preprocessing.min_x << ", " << config.preprocessing.max_x
        << "] / [" << config.preprocessing.min_y << ", " << config.preprocessing.max_y
        << "] / [" << config.preprocessing.min_z << ", " << config.preprocessing.max_z << "]\n"
        << "- RANSAC max iterations: " << config.ransac.max_iterations
        << "; distance threshold (m): " << config.ransac.distance_threshold
        << "; random seed: " << config.ransac.random_seed
        << "; max ground tilt (deg): " << config.ransac.max_tilt_degrees << "\n"
        << "- Expected ground Z (m): ";
    if (config.ransac.expected_ground_z) out << *config.ransac.expected_ground_z;
    else out << "unset";
    out << "; height tolerance (m): " << config.ransac.ground_height_tolerance << "\n"
        << "- Cluster tolerance (m): " << config.clustering.distance_tolerance
        << "; min/max points: " << config.clustering.min_cluster_size << '/'
        << config.clustering.max_cluster_size << "\n\n";
    if (frames_.empty()) return;
    out << "## Per-frame latency statistics\n\n"
           "All values below are milliseconds except overlap/angle rows. "
           "Skipped stages are recorded as zero. p95 uses linear interpolation at (N-1)*0.95.\n\n"
           "| Measurement | n | Mean | Median | p95 | Max |\n"
           "|---|---:|---:|---:|---:|---:|\n";
    const auto emit = [&](const std::string& name, auto getter) {
        reportRow(out, name, statsFor(frames_, getter), true);
    };
    emit("Disk load (ms)", [](const auto& r) { return std::optional<double>(r.load_ms); });
    emit("Preprocessing (ms)", [](const auto& r) { return std::optional<double>(r.timings.preprocessing_ms); });
    emit("RANSAC (pipeline ms)", [](const auto& r) { return std::optional<double>(r.timings.ransac_ms); });
    emit("Ground separation (ms)", [](const auto& r) { return std::optional<double>(r.timings.ground_separation_ms); });
    emit("Clustering (pipeline ms)", [](const auto& r) { return std::optional<double>(r.timings.clustering_ms); });
    emit("Bounding boxes (ms)", [](const auto& r) { return std::optional<double>(r.timings.bounding_box_ms); });
    emit("Pipeline total (ms)", [](const auto& r) { return std::optional<double>(r.timings.total_ms); });
    emit("Load + pipeline (ms)", [](const auto& r) { return std::optional<double>(r.load_and_pipeline_ms()); });
    const auto s = statsFor(frames_, [](const auto& r) { return std::optional<double>(r.load_and_pipeline_ms()); });
    if (s && s->mean > 0.0) {
        out << "\nEstimated load+pipeline-only throughput: **" << 1000.0 / s->mean << " FPS**.\n\n";
    }
    if (s) {
        out << "100 ms deadline using p95 load+pipeline: **"
            << (s->p95 <= 100.0 ? "meets" : "exceeds")
            << "** (not proof of sustained real-time operation).\n\n";
    }
    if (observed_wall_ms > 0.0) {
        out << "Observed wall throughput (includes optional comparisons, CSV, console): **"
            << frames_.size() * 1000.0 / observed_wall_ms << " FPS**.\n\n";
    }
    out << "## Custom algorithm vs PCL references\n\n"
           "Same filtered input for both plane fits, same CUSTOM-RANSAC non-ground input for both clusterers, "
           "identical numeric iteration/distance/cluster-size thresholds. "
           "PCL uses SACMODEL_PERPENDICULAR_PLANE constrained to the Z axis; "
           "random sample sequences and implementation details differ. "
           "The custom expected-ground-height option (if enabled) has no exact PCL counterpart. "
           "Times are separate benchmark reruns, NOT additional stages of the normal pipeline.\n\n";
    out << "| Measurement | n | Mean | Median | p95 | Max |\n"
           "|---|---:|---:|---:|---:|---:|\n";
    emit("Custom RANSAC benchmark (ms)", [](const auto& r) { return r.comparison ? std::optional<double>(r.comparison->custom_ransac_ms) : std::nullopt; });
    emit("PCL RANSAC benchmark (ms)", [](const auto& r) { return r.comparison ? std::optional<double>(r.comparison->pcl_ransac_ms) : std::nullopt; });
    emit("Custom clustering benchmark (ms)", [](const auto& r) { return r.comparison ? std::optional<double>(r.comparison->custom_clustering_ms) : std::nullopt; });
    emit("PCL clustering benchmark (ms)", [](const auto& r) { return r.comparison ? std::optional<double>(r.comparison->pcl_clustering_ms) : std::nullopt; });
    emit("Ground inlier Jaccard (0–1)", [](const auto& r) { return r.comparison ? r.comparison->ground_inlier_jaccard : std::nullopt; });
    emit("Plane normal difference (deg)", [](const auto& r) { return r.comparison ? r.comparison->plane_normal_angle_deg : std::nullopt; });
    emit("Cluster mean-best Jaccard (0–1)", [](const auto& r) { return r.comparison ? r.comparison->mean_best_cluster_jaccard : std::nullopt; });
    std::size_t same_counts = 0, compared = 0, same_inliers = 0;
    for (const auto& r : frames_) if (r.comparison) {
        ++compared;
        same_counts += (r.comparison->custom_cluster_count == r.comparison->pcl_cluster_count);
        same_inliers += (r.comparison->custom_ground_inliers == r.comparison->pcl_ground_inliers);
    }
    out << "\nFrames with matching cluster counts: " << same_counts << '/' << compared
        << ". Matching ground inlier counts: " << same_inliers << '/' << compared << ".\n\n"
           "## Interpretation and limitations\n\n"
           "- 'Load + pipeline' excludes rendering and logging; do **not** equate target `--fps 10` with measured throughput.\n"
           "- Observed wall throughput is measured for the executed headless command; when `--compare-pcl` is enabled, extra reference work reduces it.\n"
           "- Ground inlier Jaccard is intersection/union of inlier indices; undefined for two empty inlier sets.\n"
           "- Cluster overlap averages each custom cluster's best PCL Jaccard match; it is order-invariant, but not a one-to-one matching metric.\n"
           "- The KITTI object-detection training folder is not a temporally ordered drive sequence.\n"
           "- One run is not a reliable hardware benchmark: repeat warmed-up Release-build runs, control load and background work, and report variance.\n";
}

} // namespace lidar
