#pragma once

#include "lidar/pcl_comparison.h"
#include "lidar/pipeline.h"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace lidar {

// Small scalar-only summary; retaining it does not retain point clouds.
struct FrameMetrics {
    std::size_t frame_index = 0;
    std::string filename;
    std::size_t raw_points = 0;
    std::size_t filtered_points = 0;
    std::size_t ground_points = 0;
    std::size_t nonground_points = 0;
    std::size_t cluster_count = 0;
    std::size_t bounding_box_count = 0;
    bool found_ground = false;
    double load_ms = 0.0;
    PipelineTimings timings;
    std::optional<PclComparison> comparison;

    // Deliberately excludes plotting, console logging and optional reference
    // comparisons; this is NOT GUI or end-to-end wall-clock latency.
    double load_and_pipeline_ms() const { return load_ms + timings.total_ms; }
};

inline FrameMetrics makeFrameMetrics(std::size_t index,
                                    const std::filesystem::path& filename,
                                    std::size_t raw_points,
                                    double load_ms,
                                    const PipelineResult& result,
                                    std::optional<PclComparison> comparison = std::nullopt) {
    if (!result.filtered || !result.separated.ground || !result.separated.nonground) {
        throw std::invalid_argument("makeFrameMetrics: incomplete pipeline result");
    }
    FrameMetrics row;
    row.frame_index = index;
    row.filename = filename.filename().string();
    row.raw_points = raw_points;
    row.filtered_points = result.filtered->size();
    row.ground_points = result.separated.ground->size();
    row.nonground_points = result.separated.nonground->size();
    row.cluster_count = result.clusters.size();
    row.bounding_box_count = static_cast<std::size_t>(std::count_if(
        result.boxes.begin(), result.boxes.end(),
        [](const auto& box) { return box.has_value(); }));
    row.found_ground = result.ransac.found_plane;
    row.load_ms = load_ms;
    row.timings = result.timings;
    row.comparison = std::move(comparison);
    return row;
}

class MetricsLogger {
public:
    explicit MetricsLogger(const std::filesystem::path& path) : path_(path) {
        if (path_.empty()) throw std::invalid_argument("CSV path is empty");
        if (std::filesystem::exists(path_)) {
            throw std::runtime_error("Refusing to overwrite existing CSV: " + path_.string());
        }
        stream_.exceptions(std::ios::badbit | std::ios::failbit);
        try {
            stream_.open(path_, std::ios::out | std::ios::trunc);
            stream_ << "frame_index,filename,raw_points,filtered_points,ground_points,"
                       "nonground_points,clusters,bounding_boxes,found_ground,load_ms,"
                       "preprocessing_ms,ransac_ms,ground_separation_ms,clustering_ms,"
                       "bounding_box_ms,pipeline_total_ms,load_plus_pipeline_ms,"
                       "custom_ransac_benchmark_ms,pcl_ransac_ms,custom_ground_inliers,"
                       "pcl_ground_inliers,ground_inlier_jaccard,plane_normal_angle_deg,"
                       "custom_clustering_benchmark_ms,pcl_clustering_ms,"
                       "custom_cluster_count,pcl_cluster_count,custom_clustered_points,"
                       "pcl_clustered_points,mean_best_cluster_jaccard\n";
            stream_.flush();
        } catch (const std::ios_base::failure& e) {
            throw std::runtime_error("Could not initialize CSV: " + path_.string() + ": " + e.what());
        }
    }

    void write(const FrameMetrics& r) {
        try {
            stream_ << r.frame_index << ',' << quoted(r.filename) << ','
                    << r.raw_points << ',' << r.filtered_points << ','
                    << r.ground_points << ',' << r.nonground_points << ','
                    << r.cluster_count << ',' << r.bounding_box_count << ','
                    << (r.found_ground ? 1 : 0) << ',' << std::fixed << std::setprecision(6)
                    << r.load_ms << ',' << r.timings.preprocessing_ms << ','
                    << r.timings.ransac_ms << ',' << r.timings.ground_separation_ms << ','
                    << r.timings.clustering_ms << ',' << r.timings.bounding_box_ms << ','
                    << r.timings.total_ms << ',' << r.load_and_pipeline_ms() << ',';
            if (r.comparison) {
                const auto& c = *r.comparison;
                stream_ << c.custom_ransac_ms << ',' << c.pcl_ransac_ms << ','
                        << c.custom_ground_inliers << ',' << c.pcl_ground_inliers << ',';
                optionalValue(c.ground_inlier_jaccard); stream_ << ',';
                optionalValue(c.plane_normal_angle_deg); stream_ << ',';
                stream_ << c.custom_clustering_ms << ',' << c.pcl_clustering_ms << ','
                        << c.custom_cluster_count << ',' << c.pcl_cluster_count << ','
                        << c.custom_clustered_points << ',' << c.pcl_clustered_points << ',';
                optionalValue(c.mean_best_cluster_jaccard);
            } else {
                // 13 missing comparison columns, always preserve CSV schema.
                for (int i = 0; i < 12; ++i) stream_ << ',';
            }
            stream_ << '\n';
            stream_.flush();
        } catch (const std::ios_base::failure& e) {
            throw std::runtime_error("CSV write failed: " + path_.string() + ": " + e.what());
        }
    }

private:
    static std::string quoted(const std::string& value) {
        std::string output = "\"";
        for (char c : value) output += (c == '"' ? "\"\"" : std::string(1, c));
        return output + '"';
    }
    void optionalValue(const std::optional<double>& value) {
        if (value) stream_ << *value;
    }
    std::filesystem::path path_;
    std::ofstream stream_;
};

} // namespace lidar
