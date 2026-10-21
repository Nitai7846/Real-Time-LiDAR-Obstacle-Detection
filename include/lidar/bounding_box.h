
#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar {

struct BoundingBox3D {

    // Minimum coordinates
    float min_x = 0.0f;
    float min_y = 0.0f;
    float min_z = 0.0f;

    // Maximum coordinates
    float max_x = 0.0f;
    float max_y = 0.0f;
    float max_z = 0.0f;

    // Center coordinates
    float center_x = 0.0f;
    float center_y = 0.0f;
    float center_z = 0.0f;

    // Box dimensions
    float length = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
};

class BoundingBox {

public:

    using Point = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<Point>;

    using Cluster = std::vector<std::size_t>;
    using Clusters = std::vector<Cluster>;

    using Box = BoundingBox3D;
    using Boxes = std::vector<std::optional<Box>>;

    // Compute the bounding box for one cluster.
    static std::optional<Box> compute(
        const Cloud& cloud,
        const Cluster& cluster
    );

    // Compute bounding boxes for all clusters.
    static Boxes computeAll(
        const Cloud& cloud,
        const Clusters& clusters
    );
};

} // namespace lidar
