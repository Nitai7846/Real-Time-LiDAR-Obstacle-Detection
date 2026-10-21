
#include "lidar/ransac_plane.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

using namespace lidar;

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

// Simple test assertion
void check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

// Helper function to add a 3D point
void addPoint(Cloud& cloud, float x, float y, float z) {
    pcl::PointXYZI point{};
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = 1.0f;

    cloud.push_back(point);
}

// Test 1: Horizontal ground + obstacles
void testHorizontalPlane() {
    auto cloud = std::make_shared<Cloud>();

    // Generate 100 ground points at z = 0
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            addPoint(*cloud, x, y, 0.0f);
        }
    }

    // Add 10 obstacle points above ground
    for (int i = 0; i < 10; i++) {
        addPoint(*cloud, i, 2.0f, 2.0f);
    }

    RansacConfig config;
    config.expected_ground_z = 0.0f;
    config.ground_height_tolerance = 0.3f;

    RansacResult result = RansacPlane::fit(cloud, config);

    check(result.found_plane, "Horizontal plane not found");

    check(result.inlier_indices.size() == 100,
          "Incorrect horizontal ground inlier count");

    std::cout << "HorizontalPlaneTest PASSED\n";
}

// Test 2: Sloped ground + obstacles
void testSlopedPlane() {
    auto cloud = std::make_shared<Cloud>();

    // Ground equation: z = 0.05x + 0.03y - 1.5
    for (int x = 0; x < 12; x++) {
        for (int y = 0; y < 12; y++) {
            float z = 0.05f*x + 0.03f*y - 1.5f;
            addPoint(*cloud, x, y, z);
        }
    }

    // Obstacle points
    for (int i = 0; i < 12; i++) {
        addPoint(*cloud, i, 3.0f, 2.0f);
    }

    RansacConfig config;
    config.expected_ground_z = -1.5f;
    config.ground_height_tolerance = 0.3f;

    RansacResult result = RansacPlane::fit(cloud, config);

    check(result.found_plane, "Sloped plane not found");

    check(result.inlier_indices.size() == 144,
          "Incorrect sloped ground inlier count");

    // Check estimated ground height at x=0, y=0
    float ground_z = -result.plane.d / result.plane.c;

    check(std::abs(ground_z + 1.5f) < 0.1f,
          "Incorrect estimated ground height");

    std::cout << "SlopedPlaneTest PASSED\n";
}

// Test 3: Empty and insufficient clouds
void testEmptyCloud() {
    auto cloud = std::make_shared<Cloud>();

    RansacConfig config;

    auto result = RansacPlane::fit(cloud, config);

    check(!result.found_plane,
          "Empty cloud should not produce a plane");

    addPoint(*cloud, 0.0f, 0.0f, 0.0f);
    addPoint(*cloud, 1.0f, 0.0f, 0.0f);

    result = RansacPlane::fit(cloud, config);

    check(!result.found_plane,
          "Two points cannot define a unique plane");

    std::cout << "EmptyCloudTest PASSED\n";
}

// Test 4: Collinear points
void testCollinearPoints() {
    auto cloud = std::make_shared<Cloud>();

    // All points lie on one straight line
    for (int i = 0; i < 10; i++) {
        addPoint(*cloud, i, 0.0f, 0.0f);
    }

    RansacConfig config;
    auto result = RansacPlane::fit(cloud, config);

    check(!result.found_plane,
          "Collinear points should be rejected");

    std::cout << "CollinearPointsTest PASSED\n";
}

// Test 5: Reject vertical plane
void testVerticalPlane() {
    auto cloud = std::make_shared<Cloud>();

    addPoint(*cloud, 2.0f, 0.0f, 0.0f);
    addPoint(*cloud, 2.0f, 1.0f, 0.0f);
    addPoint(*cloud, 2.0f, 0.0f, 1.0f);

    RansacConfig config;
    auto result = RansacPlane::fit(cloud, config);

    check(!result.found_plane,
          "Vertical plane should not be ground");

    std::cout << "VerticalPlaneTest PASSED\n";
}

// Test 6: Deterministic random seed
void testRepeatability() {
    auto cloud = std::make_shared<Cloud>();

    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            addPoint(*cloud, x, y, 0.0f);
        }
    }

    addPoint(*cloud, 2.0f, 3.0f, 2.0f);

    RansacConfig config;
    config.random_seed = 42;

    auto result1 = RansacPlane::fit(cloud, config);
    auto result2 = RansacPlane::fit(cloud, config);

    check(result1.found_plane && result2.found_plane,
          "Repeatability plane not found");

    check(result1.inlier_indices == result2.inlier_indices,
          "Same seed gave different inliers");

    check(result1.plane.a == result2.plane.a &&
          result1.plane.b == result2.plane.b &&
          result1.plane.c == result2.plane.c &&
          result1.plane.d == result2.plane.d,
          "Same seed gave different coefficients");

    std::cout << "RepeatabilityTest PASSED\n";
}

// Run all tests
int main() {
    try {
        testHorizontalPlane();
        testSlopedPlane();
        testEmptyCloud();
        testCollinearPoints();
        testVerticalPlane();
        testRepeatability();

        std::cout << "\nAll RANSAC tests PASSED!\n";
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "TEST FAILED: " << e.what() << "\n";
        return 1;
    }
}
