
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <optional>
#include <vector>

namespace lidar {

struct RansacConfig {
    int max_iterations = 100;
    float distance_threshold = 0.2f;
    unsigned int random_seed = 42;

    float max_tilt_degrees = 25.0f;

    std::optional<float> expected_ground_z = std::nullopt;
    float ground_height_tolerance = 1.0f;
};

struct PlaneModel {
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float d = 0.0f;
};

struct RansacResult {
    bool found_plane = false;
    PlaneModel plane;
    std::vector<std::size_t> inlier_indices;
};

class RansacPlane {
public:
    using Cloud = pcl::PointCloud<pcl::PointXYZI>;
    using CloudConstPtr = Cloud::ConstPtr;

    static RansacResult fit(
        const CloudConstPtr& cloud,
        const RansacConfig& config
    );
};

} // namespace lidar
