
#include "lidar/ransac_plane.h"

#include <cmath>
#include <random>
#include <stdexcept>
#include <utility>

namespace lidar {

RansacResult RansacPlane::fit(
    const CloudConstPtr& cloud,
    const RansacConfig& config
) {
    RansacResult result;

    // 1. Check input cloud
    if (!cloud || cloud->size() < 3) {
        return result;
    }

    // 2. Validate configuration
    if (config.max_iterations <= 0 ||
        !std::isfinite(config.distance_threshold) ||
        config.distance_threshold <= 0.0f ||
        !std::isfinite(config.max_tilt_degrees) ||
        config.max_tilt_degrees < 0.0f ||
        config.max_tilt_degrees >= 90.0f ||
        !std::isfinite(config.ground_height_tolerance) ||
        config.ground_height_tolerance < 0.0f) {

        throw std::invalid_argument(
            "Invalid RANSAC parameters"
        );
    }

    if (config.expected_ground_z.has_value() &&
        !std::isfinite(*config.expected_ground_z)) {
        throw std::invalid_argument(
            "Invalid expected ground height"
        );
    }

    // 3. Initialize random number generator
    std::mt19937 rng(config.random_seed);

    std::uniform_int_distribution<std::size_t> dist(
        0, cloud->size() - 1
    );

    // Maximum allowed plane tilt
    constexpr float PI = 3.14159265358979323846f;

    float max_tilt_radians =
        config.max_tilt_degrees * PI / 180.0f;

    float min_alignment = std::cos(max_tilt_radians);

    // 4. RANSAC iterations
    for (int i = 0; i < config.max_iterations; i++) {

        // Select three random indices
        std::size_t i1 = dist(rng);
        std::size_t i2 = dist(rng);
        std::size_t i3 = dist(rng);

        // Reject duplicate indices
        if (i1 == i2 || i1 == i3 || i2 == i3) {
            continue;
        }

        // Retrieve the three points
        const auto& p1 = cloud->points[i1];
        const auto& p2 = cloud->points[i2];
        const auto& p3 = cloud->points[i3];

        // Reject invalid sample coordinates
        auto valid = [](const auto& p) {
            return std::isfinite(p.x) &&
                   std::isfinite(p.y) &&
                   std::isfinite(p.z);
        };

        if (!valid(p1) || !valid(p2) || !valid(p3)) {
            continue;
        }

        // 5. Construct two vectors
        float v1x = p2.x - p1.x;
        float v1y = p2.y - p1.y;
        float v1z = p2.z - p1.z;

        float v2x = p3.x - p1.x;
        float v2y = p3.y - p1.y;
        float v2z = p3.z - p1.z;

        // 6. Cross product gives A, B, C
        float a = v1y * v2z - v1z * v2y;
        float b = v1z * v2x - v1x * v2z;
        float c = v1x * v2y - v1y * v2x;

        // 7. Calculate normal vector magnitude
        float norm = std::sqrt(a*a + b*b + c*c);

        // Reject collinear or degenerate points
        if (!std::isfinite(norm) || norm < 1e-6f) {
            continue;
        }

        // 8. Calculate D
        float d = -(a*p1.x + b*p1.y + c*p1.z);

        if (!std::isfinite(d)) {
            continue;
        }

        // 9. Check ground plane orientation
        float alignment = std::abs(c) / norm;

        if (alignment < min_alignment) {
            continue;
        }

        // 10. Optional ground height check
        if (config.expected_ground_z.has_value()) {

            if (std::abs(c) < 1e-6f) {
                continue;
            }

            float ground_z = -d / c;

            if (!std::isfinite(ground_z) ||
                std::abs(
                    ground_z - *config.expected_ground_z
                ) > config.ground_height_tolerance) {

                continue;
            }
        }

        // 11. Find all inliers for this plane
        std::vector<std::size_t> current_inliers;

        for (std::size_t j = 0; j < cloud->size(); j++) {

            const auto& point = cloud->points[j];

            if (!valid(point)) {
                continue;
            }

            // Perpendicular distance to plane
            float distance = std::abs(
                a * point.x +
                b * point.y +
                c * point.z +
                d
            ) / norm;

            // Add point if within tolerance
            if (distance <= config.distance_threshold) {
                current_inliers.push_back(j);
            }
        }

        // 12. Keep the plane with the most inliers
        if (current_inliers.size() >
            result.inlier_indices.size()) {

            result.found_plane = true;

            // Store normalized plane coefficients
            result.plane.a = a / norm;
            result.plane.b = b / norm;
            result.plane.c = c / norm;
            result.plane.d = d / norm;

            result.inlier_indices = std::move(current_inliers);
        }
    }

    // 13. Return the best plane and its inlier indices
    return result;
}

} // namespace lidar
