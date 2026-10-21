
#include "lidar/preprocessing.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <Eigen/Core>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace lidar {

Preprocessor::CloudPtr Preprocessor::filter(
    const CloudConstPtr& input,
    const PreprocessingConfig& config
) {

    // 1. Validate the input
    if (!input) {
        throw std::invalid_argument("Input cloud is null");
    }

    if (!std::isfinite(config.voxel_size) ||
        config.voxel_size <= 0.0f) {
        throw std::invalid_argument("Invalid voxel size");
    }

    // Validate ROI boundaries
    auto valid_range = [](float min, float max) {
        return std::isfinite(min) &&
               std::isfinite(max) &&
               min <= max;
    };

    if (!valid_range(config.min_x, config.max_x) ||
        !valid_range(config.min_y, config.max_y) ||
        !valid_range(config.min_z, config.max_z)) {
        throw std::invalid_argument("Invalid ROI boundaries");
    }

    // 2. Remove invalid XYZ points
    CloudPtr clean_cloud = std::make_shared<Cloud>();

    clean_cloud->reserve(input->size());

    for (const auto& point : input->points) {

        if (std::isfinite(point.x) &&
            std::isfinite(point.y) &&
            std::isfinite(point.z)) {

            clean_cloud->push_back(point);
        }
    }

    // Prepare a new output cloud
    CloudPtr output = std::make_shared<Cloud>();

    if (clean_cloud->empty()) {
        return output;
    }

    // 3. VoxelGrid downsampling
    CloudPtr downsampled = std::make_shared<Cloud>();

    pcl::VoxelGrid<pcl::PointXYZI> voxel;

    voxel.setInputCloud(clean_cloud);

    voxel.setLeafSize(
        config.voxel_size,
        config.voxel_size,
        config.voxel_size
    );

    voxel.filter(*downsampled);

    // 4. CropBox ROI filtering
    pcl::CropBox<pcl::PointXYZI> crop;

    crop.setInputCloud(downsampled);

    crop.setMin(Eigen::Vector4f(
        config.min_x,
        config.min_y,
        config.min_z,
        1.0f
    ));

    crop.setMax(Eigen::Vector4f(
        config.max_x,
        config.max_y,
        config.max_z,
        1.0f
    ));

    crop.filter(*output);

    return output;
}

} // namespace lidar
