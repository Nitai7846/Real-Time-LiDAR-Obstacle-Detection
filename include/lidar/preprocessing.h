
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar {

// Configuration for point-cloud preprocessing
struct PreprocessingConfig {

    // VoxelGrid leaf size in meters
    float voxel_size = 0.2f;

    // Region of interest boundaries
    float min_x = -15.0f;
    float max_x = 35.0f;

    float min_y = -10.0f;
    float max_y = 10.0f;

    float min_z = -3.0f;
    float max_z = 4.0f;
};


// Point-cloud preprocessing class
class Preprocessor {

public:

    using Cloud = pcl::PointCloud<pcl::PointXYZI>;
    using CloudPtr = Cloud::Ptr;
    using CloudConstPtr = Cloud::ConstPtr;

    static CloudPtr filter(
        const CloudConstPtr& input,
        const PreprocessingConfig& config
    );

};

} // namespace lidar
