
#include <iostream>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "lidar/preprocessing.h"

int main() {

    using Cloud = pcl::PointCloud<pcl::PointXYZI>;

    // Create a synthetic point cloud
    auto cloud = std::make_shared<Cloud>();

    pcl::PointXYZI p1{};
    p1.x = 1.01f;
    p1.y = 1.01f;
    p1.z = 0.01f;
    p1.intensity = 0.5f;

    // Point close to p1 (same voxel)
    auto p2 = p1;
    p2.x = 1.04f;

    // Point in a different voxel
    auto p3 = p1;
    p3.x = 2.1f;
    p3.y = 2.1f;

    // Point outside our ROI
    auto p4 = p1;
    p4.x = 100.0f;

    // Point with invalid X coordinate
    auto p5 = p1;
    p5.x = std::numeric_limits<float>::quiet_NaN();

    cloud->push_back(p1);
    cloud->push_back(p2);
    cloud->push_back(p3);
    cloud->push_back(p4);
    cloud->push_back(p5);

    lidar::PreprocessingConfig config;

    // Run preprocessing
    auto filtered = lidar::Preprocessor::filter(cloud, config);

    // Expected: p1 and p2 combine, p3 remains
    if (filtered->size() != 2) {
        std::cerr << "FAILED: Expected 2 filtered points\n";
        return 1;
    }

    // Original cloud should remain unchanged
    if (cloud->size() != 5) {
        std::cerr << "FAILED: Original cloud was modified\n";
        return 1;
    }

    // Test empty cloud
    auto empty = std::make_shared<Cloud>();
    auto empty_result = lidar::Preprocessor::filter(empty, config);

    if (!empty_result->empty()) {
        std::cerr << "FAILED: Empty cloud test\n";
        return 1;
    }

    // Test invalid voxel size
    config.voxel_size = -0.2f;

    bool exception_caught = false;

    try {
        lidar::Preprocessor::filter(cloud, config);
    }
    catch (const std::invalid_argument&) {
        exception_caught = true;
    }

    if (!exception_caught) {
        std::cerr << "FAILED: Invalid voxel size test\n";
        return 1;
    }

    std::cout << "All preprocessing tests passed!\n";

    return 0;
}
