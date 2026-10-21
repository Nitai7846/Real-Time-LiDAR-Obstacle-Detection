
#include "lidar/bounding_box.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lidar {

// Compute AABB for a single cluster
std::optional<BoundingBox::Box> BoundingBox::compute(
    const Cloud& cloud,
    const Cluster& cluster
) {

    // An empty cluster has no bounding box
    if (cluster.empty()) {
        return std::nullopt;
    }

    Box box;
    bool initialized = false;

    // Iterate through all points in the cluster
    for (std::size_t index : cluster) {

        // Validate the point index
        if (index >= cloud.size()) {
            throw std::out_of_range(
                "BoundingBox: invalid point index"
            );
        }

        const Point& point = cloud.points[index];

        // Reject non-finite coordinates
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            return std::nullopt;
        }

        // Initialize boundaries using the first point
        if (!initialized) {

            box.min_x = box.max_x = point.x;
            box.min_y = box.max_y = point.y;
            box.min_z = box.max_z = point.z;

            initialized = true;
            continue;
        }

        // Update minimum coordinates
        box.min_x = std::min(box.min_x, point.x);
        box.min_y = std::min(box.min_y, point.y);
        box.min_z = std::min(box.min_z, point.z);

        // Update maximum coordinates
        box.max_x = std::max(box.max_x, point.x);
        box.max_y = std::max(box.max_y, point.y);
        box.max_z = std::max(box.max_z, point.z);
    }

    // Calculate dimensions
    box.length = box.max_x - box.min_x;
    box.width  = box.max_y - box.min_y;
    box.height = box.max_z - box.min_z;

    // Reject dimensions that overflow to non-finite values
    if (!std::isfinite(box.length) ||
        !std::isfinite(box.width) ||
        !std::isfinite(box.height)) {
        return std::nullopt;
    }

    // Calculate box center
    box.center_x = box.min_x + box.length / 2.0f;
    box.center_y = box.min_y + box.width  / 2.0f;
    box.center_z = box.min_z + box.height / 2.0f;

    return box;
}


// Compute bounding boxes for all clusters
BoundingBox::Boxes BoundingBox::computeAll(
    const Cloud& cloud,
    const Clusters& clusters
) {

    Boxes boxes;

    // Reserve space for one result per cluster
    boxes.reserve(clusters.size());

    for (const Cluster& cluster : clusters) {

        boxes.push_back(
            compute(cloud, cluster)
        );
    }

    return boxes;
}

} // namespace lidar
