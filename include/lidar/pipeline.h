#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar/bounding_box.h"
#include "lidar/euclidean_clustering.h"
#include "lidar/ground_separator.h"
#include "lidar/preprocessing.h"
#include "lidar/ransac_plane.h"

namespace lidar {

// Configures the existing algorithms without changing their implementations.
struct PipelineConfig {
    PreprocessingConfig preprocessing;
    RansacConfig ransac;
    ClusteringConfig clustering;
};

// Execution times in milliseconds, measured using std::chrono::steady_clock.
// A stage that was skipped has a time of 0.0 ms.
// total_ms covers Pipeline::process(), excluding disk I/O and visualization.
struct PipelineTimings {
    double preprocessing_ms = 0.0;
    double ransac_ms = 0.0;
    double ground_separation_ms = 0.0;
    double clustering_ms = 0.0;
    double bounding_box_ms = 0.0;
    double total_ms = 0.0;
};

// Results for ONE frame. All cluster indices refer to separated.nonground.
// boxes[i] corresponds to clusters[i].
struct PipelineResult {
    using Cloud = pcl::PointCloud<pcl::PointXYZI>;
    using CloudPtr = Cloud::Ptr;

    CloudPtr filtered;
    RansacResult ransac;
    GroundSplit separated;
    EuclideanClustering::Clusters clusters;
    BoundingBox::Boxes boxes;
    PipelineTimings timings;
};

class Pipeline {
public:
    using Cloud = pcl::PointCloud<pcl::PointXYZI>;
    using CloudConstPtr = Cloud::ConstPtr;

    explicit Pipeline(PipelineConfig config = {});

    // Processes a single cloud without changing the caller-owned cloud.
    // Throws std::invalid_argument for a null input pointer.
    // If no plane is found, all filtered points are treated as non-ground.
    PipelineResult process(const CloudConstPtr& input) const;

private:
    PipelineConfig config_;
};

}  // namespace lidar
