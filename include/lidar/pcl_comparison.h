#pragma once

#include <cstddef>
#include <optional>

#include "lidar/pipeline.h"

namespace lidar {

// Measures reference algorithms on EXACTLY the same filtered cloud and
// custom-ground-separated non-ground cloud used by the current pipeline.
// Reference comparisons are optional and never alter PipelineResult.
struct PclComparison {
    double custom_ransac_ms = 0.0;
    double pcl_ransac_ms = 0.0;
    std::size_t custom_ground_inliers = 0;
    std::size_t pcl_ground_inliers = 0;
    std::optional<double> ground_inlier_jaccard;
    std::optional<double> plane_normal_angle_deg;

    double custom_clustering_ms = 0.0;
    double pcl_clustering_ms = 0.0;
    std::size_t custom_cluster_count = 0;
    std::size_t pcl_cluster_count = 0;
    std::size_t custom_clustered_points = 0;
    std::size_t pcl_clustered_points = 0;
    std::optional<double> mean_best_cluster_jaccard;
};

class PclComparator {
public:
    static PclComparison compare(const PipelineResult& processed,
                                 const PipelineConfig& config = {});
};

} // namespace lidar
