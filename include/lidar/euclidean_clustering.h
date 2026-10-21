
#pragma once

#include <cstddef>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar {

// Configuration for Euclidean clustering.
struct ClusteringConfig {
    float distance_tolerance = 0.5f;
    std::size_t min_cluster_size = 20;
    std::size_t max_cluster_size = 10000;
};

class EuclideanClustering {
public:
    using Point = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<Point>;

    using Cluster = std::vector<std::size_t>;
    using Clusters = std::vector<Cluster>;

    explicit EuclideanClustering(ClusteringConfig config = {});

    Clusters extract(const Cloud& cloud) const;

private:
    ClusteringConfig config_;
};

}  // namespace lidar
