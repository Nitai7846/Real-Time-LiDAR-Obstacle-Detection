
#include "lidar/ground_separator.h"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace lidar;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

// Simple test assertion
void check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

// Helper to add points
void addPoint(Cloud& cloud, float x) {
    pcl::PointXYZI point{};
    point.x = x;
    point.y = 0.0f;
    point.z = 0.0f;
    point.intensity = 1.0f;

    cloud.push_back(point);
}

// Test 1: Ground and non-ground separation
void testSeparation() {
    auto cloud = std::make_shared<Cloud>();

    for (int i = 0; i < 5; i++) {
        addPoint(*cloud, static_cast<float>(i));
    }

    std::vector<std::size_t> inliers = {1, 3};

    auto result = GroundSeparator::split(cloud, inliers);

    check(result.ground->size() == 2,
          "Incorrect ground count");

    check(result.nonground->size() == 3,
          "Incorrect non-ground count");

    check(result.ground->points[0].x == 1.0f &&
          result.ground->points[1].x == 3.0f,
          "Incorrect ground points");

    check(result.nonground->points[0].x == 0.0f &&
          result.nonground->points[1].x == 2.0f &&
          result.nonground->points[2].x == 4.0f,
          "Incorrect non-ground points");

    check(cloud->size() == 5,
          "Original cloud was modified");

    std::cout << "SeparationTest PASSED\n";
}

// Test 2: Empty cloud
void testEmptyCloud() {
    auto cloud = std::make_shared<Cloud>();

    auto result = GroundSeparator::split(cloud, {});

    check(result.ground->empty(),
          "Ground should be empty");

    check(result.nonground->empty(),
          "Non-ground should be empty");

    std::cout << "EmptyCloudTest PASSED\n";
}

// Test 3: No ground inliers
void testNoInliers() {
    auto cloud = std::make_shared<Cloud>();

    addPoint(*cloud, 1.0f);
    addPoint(*cloud, 2.0f);
    addPoint(*cloud, 3.0f);

    auto result = GroundSeparator::split(cloud, {});

    check(result.ground->empty(),
          "Ground should be empty");

    check(result.nonground->size() == 3,
          "All points should be non-ground");

    std::cout << "NoInliersTest PASSED\n";
}

// Test 4: Duplicate inlier indices
void testDuplicateIndices() {
    auto cloud = std::make_shared<Cloud>();

    addPoint(*cloud, 1.0f);
    addPoint(*cloud, 2.0f);
    addPoint(*cloud, 3.0f);

    auto result = GroundSeparator::split(cloud, {0, 0, 2});

    check(result.ground->size() == 2,
          "Duplicate indices caused duplicate points");

    check(result.nonground->size() == 1,
          "Incorrect non-ground count");

    std::cout << "DuplicateIndicesTest PASSED\n";
}

// Test 5: Invalid inlier index
void testInvalidIndex() {
    auto cloud = std::make_shared<Cloud>();

    addPoint(*cloud, 1.0f);

    bool caught = false;

    try {
        GroundSeparator::split(cloud, {5});
    }
    catch (const std::out_of_range&) {
        caught = true;
    }

    check(caught, "Invalid index was not rejected");

    std::cout << "InvalidIndexTest PASSED\n";
}

// Test 6: Null cloud
void testNullCloud() {
    GroundSeparator::CloudConstPtr cloud;

    bool caught = false;

    try {
        GroundSeparator::split(cloud, {});
    }
    catch (const std::invalid_argument&) {
        caught = true;
    }

    check(caught, "Null cloud was not rejected");

    std::cout << "NullCloudTest PASSED\n";
}

int main() {
    try {
        testSeparation();
        testEmptyCloud();
        testNoInliers();
        testDuplicateIndices();
        testInvalidIndex();
        testNullCloud();

        std::cout << "\nAll Ground Separator tests PASSED!\n";
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "TEST FAILED: " << e.what() << "\n";
        return 1;
    }
}
