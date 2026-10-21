
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <vector>

namespace lidar {

// Stores the two resulting point clouds
struct GroundSplit {
    using Cloud = pcl::PointCloud<pcl::PointXYZI>;

    Cloud::Ptr ground;
    Cloud::Ptr nonground;
};

class GroundSeparator {
public:
    using Cloud = pcl::PointCloud<pcl::PointXYZI>;
    using CloudConstPtr = Cloud::ConstPtr;

    static GroundSplit split(
        const CloudConstPtr& cloud,
        const std::vector<std::size_t>& inlier_indices
    );
};

} // namespace lidar
