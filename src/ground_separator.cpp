
#include "lidar/ground_separator.h"

#include <memory>
#include <stdexcept>
#include <vector>

namespace lidar {

GroundSplit GroundSeparator::split(
    const CloudConstPtr& cloud,
    const std::vector<std::size_t>& inlier_indices
) {
    // 1. Validate input
    if (!cloud) {
        throw std::invalid_argument("Input cloud is null");
    }

    // 2. Create two empty point clouds
    GroundSplit result;

    result.ground = std::make_shared<Cloud>();
    result.nonground = std::make_shared<Cloud>();

    // 3. Create a mask to identify ground points
    std::vector<bool> is_ground(cloud->size(), false);

    // 4. Mark all RANSAC inliers as ground
    for (std::size_t index : inlier_indices) {

        if (index >= cloud->size()) {
            throw std::out_of_range("Invalid inlier index");
        }

        is_ground[index] = true;
    }

    // 5. Separate ground and non-ground points
    for (std::size_t i = 0; i < cloud->size(); i++) {

        const auto& point = cloud->points[i];

        if (is_ground[i]) {
            result.ground->push_back(point);
        }
        else {
            result.nonground->push_back(point);
        }
    }

    // 6. Return both point clouds
    return result;
}

} // namespace lidar
