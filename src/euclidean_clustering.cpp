
#include "lidar/euclidean_clustering.h"
#include "lidar/kdtree3d.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lidar {

// Constructor
EuclideanClustering::EuclideanClustering(ClusteringConfig config)
    : config_(config) {

    if (!std::isfinite(config_.distance_tolerance) ||
        config_.distance_tolerance <= 0.0f) {
        throw std::invalid_argument(
            "Distance tolerance must be finite and positive."
        );
    }

    if (config_.min_cluster_size == 0) {
        throw std::invalid_argument(
            "Minimum cluster size must be greater than zero."
        );
    }

    if (config_.max_cluster_size < config_.min_cluster_size) {
        throw std::invalid_argument(
            "Maximum cluster size must be >= minimum cluster size."
        );
    }
}


// Extract Euclidean clusters
EuclideanClustering::Clusters
EuclideanClustering::extract(const Cloud& cloud) const {

    Clusters clusters;

    if (cloud.empty()) {
        return clusters;
    }

    // Step 1: Build our custom KD-Tree
    KdTree3D tree;

    for (std::size_t i = 0; i < cloud.points.size(); ++i) {

        const auto& point = cloud.points[i];

        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            throw std::invalid_argument(
                "Point cloud contains non-finite coordinates."
            );
        }

        tree.insert(point, i);
    }

    // Step 2: Track visited points
    std::vector<bool> visited(cloud.points.size(), false);

    // Step 3: Iterate through every point
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {

        if (visited[i]) {
            continue;
        }

        Cluster cluster;
        std::queue<std::size_t> queue;

        queue.push(i);
        visited[i] = true;

        // Step 4: Expand the cluster using BFS
        while (!queue.empty()) {

            std::size_t current_index = queue.front();
            queue.pop();

            cluster.push_back(current_index);

            // Find neighboring points using our KD-Tree
            const auto neighbors = tree.radiusSearch(
                cloud.points[current_index],
                config_.distance_tolerance
            );

            for (std::size_t neighbor_index : neighbors) {

                if (!visited[neighbor_index]) {

                    visited[neighbor_index] = true;
                    queue.push(neighbor_index);
                }
            }
        }

        // Step 5: Apply cluster-size filtering
        if (cluster.size() >= config_.min_cluster_size &&
            cluster.size() <= config_.max_cluster_size) {

            std::sort(cluster.begin(), cluster.end());
            clusters.push_back(std::move(cluster));
        }
    }

    // Step 6: Return accepted clusters
    return clusters;
}

}  // namespace lidar
