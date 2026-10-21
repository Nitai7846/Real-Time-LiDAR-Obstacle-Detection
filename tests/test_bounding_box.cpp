
#include "lidar/bounding_box.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using Cloud = lidar::BoundingBox::Cloud;
using Cluster = lidar::BoundingBox::Cluster;
using Clusters = lidar::BoundingBox::Clusters;
using Box = lidar::BoundingBox3D;


// Helper: add a 3D point to a cloud
void addPoint(Cloud& cloud, float x, float y, float z) {
    pcl::PointXYZI point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = 0.0f;

    cloud.push_back(point);
}


// Helper: check a test condition
void check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}


// Helper: compare floating-point values
void checkNear(float actual, float expected) {
    constexpr float epsilon = 1e-5f;

    check(
        std::abs(actual - expected) <= epsilon,
        "Floating-point comparison failed"
    );
}


// Test 1: Simple cube
void testCube() {
    Cloud cloud;
    addPoint(cloud, 0, 0, 0);
    addPoint(cloud, 2, 2, 2);

    auto box = lidar::BoundingBox::compute(cloud, {0, 1});

    check(box.has_value(), "Cube should have a box");
    checkNear(box->length, 2);
    checkNear(box->width, 2);
    checkNear(box->height, 2);
}


// Test 2: Rectangular cuboid
void testCuboid() {
    Cloud cloud;
    addPoint(cloud, 0, 0, 0);
    addPoint(cloud, 4, 3, 2);

    auto box = lidar::BoundingBox::compute(cloud, {0, 1});

    check(box.has_value(), "Cuboid should have a box");
    checkNear(box->length, 4);
    checkNear(box->width, 3);
    checkNear(box->height, 2);
}


// Test 3: Negative coordinates
void testNegativeCoordinates() {
    Cloud cloud;
    addPoint(cloud, -5, -4, -3);
    addPoint(cloud, -1, -2, -1);

    auto box = lidar::BoundingBox::compute(cloud, {0, 1});

    check(box.has_value(), "Negative coordinates failed");
    checkNear(box->min_x, -5);
    checkNear(box->max_x, -1);
    checkNear(box->min_y, -4);
    checkNear(box->max_y, -2);
    checkNear(box->min_z, -3);
    checkNear(box->max_z, -1);
}


// Test 4: Single point
void testSinglePoint() {
    Cloud cloud;
    addPoint(cloud, 3, 4, 5);

    auto box = lidar::BoundingBox::compute(cloud, {0});

    check(box.has_value(), "Single point should have a box");
    checkNear(box->length, 0);
    checkNear(box->width, 0);
    checkNear(box->height, 0);
}


// Test 5: Empty cluster
void testEmptyCluster() {
    Cloud cloud;

    auto box = lidar::BoundingBox::compute(cloud, {});

    check(!box.has_value(), "Empty cluster should have no box");
}


// Test 6: Flat cluster
void testFlatCluster() {
    Cloud cloud;
    addPoint(cloud, 0, 0, 2);
    addPoint(cloud, 5, 3, 2);

    auto box = lidar::BoundingBox::compute(cloud, {0, 1});

    check(box.has_value(), "Flat cluster should have a box");
    checkNear(box->height, 0);
}


// Test 7: Center calculation
void testCenter() {
    Cloud cloud;
    addPoint(cloud, -1, 2, 0);
    addPoint(cloud, 3, 5, 2);

    auto box = lidar::BoundingBox::compute(cloud, {0, 1});

    check(box.has_value(), "Center test failed");
    checkNear(box->center_x, 1);
    checkNear(box->center_y, 3.5f);
    checkNear(box->center_z, 1);
}


// Test 8: Dimensions
void testDimensions() {
    Cloud cloud;
    addPoint(cloud, -2, 1, -1);
    addPoint(cloud, 5, 4, 3);

    auto box = lidar::BoundingBox::compute(cloud, {0, 1});

    check(box.has_value(), "Dimensions test failed");
    checkNear(box->length, 7);
    checkNear(box->width, 3);
    checkNear(box->height, 4);
}


// Test 9: All cluster points must be enclosed
void testPointEnclosure() {
    Cloud cloud;
    addPoint(cloud, -2, 1, 0);
    addPoint(cloud, 4, 5, 3);
    addPoint(cloud, 1, 2, 1);
    addPoint(cloud, 0, 4, 2);

    Cluster cluster = {0, 1, 2, 3};

    auto box = lidar::BoundingBox::compute(cloud, cluster);

    check(box.has_value(), "Point enclosure test failed");

    for (std::size_t index : cluster) {
        const auto& p = cloud.points[index];

        check(p.x >= box->min_x && p.x <= box->max_x,
              "Point outside X boundaries");

        check(p.y >= box->min_y && p.y <= box->max_y,
              "Point outside Y boundaries");

        check(p.z >= box->min_z && p.z <= box->max_z,
              "Point outside Z boundaries");
    }
}


// Test 10: Invalid point index
void testInvalidIndex() {
    Cloud cloud;
    addPoint(cloud, 0, 0, 0);

    bool caught = false;

    try {
        lidar::BoundingBox::compute(cloud, {0, 10});
    }
    catch (const std::out_of_range&) {
        caught = true;
    }

    check(caught, "Invalid index should throw");
}


// Test 11: Non-finite coordinates
void testNonFinite() {
    Cloud cloud;
    addPoint(cloud, 0, 0, 0);

    addPoint(
        cloud,
        std::numeric_limits<float>::quiet_NaN(),
        1,
        2
    );

    auto box = lidar::BoundingBox::compute(cloud, {0, 1});

    check(!box.has_value(), "NaN point should be rejected");
}


// Test 12: Multiple clusters
void testMultipleClusters() {
    Cloud cloud;
    addPoint(cloud, 0, 0, 0);
    addPoint(cloud, 2, 2, 2);
    addPoint(cloud, 10, 10, 10);
    addPoint(cloud, 14, 13, 12);

    Clusters clusters = {
        {0, 1},
        {},
        {2, 3}
    };

    auto boxes = lidar::BoundingBox::computeAll(cloud, clusters);

    check(boxes.size() == 3, "Incorrect number of results");

    check(boxes[0].has_value(), "First box missing");
    check(!boxes[1].has_value(), "Empty cluster should have no box");
    check(boxes[2].has_value(), "Third box missing");

    checkNear(boxes[0]->length, 2);
    checkNear(boxes[2]->length, 4);
}


// Test runner
int main() {

    struct TestCase {
        const char* name;
        void (*function)();
    };

    std::vector<TestCase> tests = {
        {"Simple cube", testCube},
        {"Rectangular cuboid", testCuboid},
        {"Negative coordinates", testNegativeCoordinates},
        {"Single point", testSinglePoint},
        {"Empty cluster", testEmptyCluster},
        {"Flat cluster", testFlatCluster},
        {"Center calculation", testCenter},
        {"Dimensions", testDimensions},
        {"Point enclosure", testPointEnclosure},
        {"Invalid indices", testInvalidIndex},
        {"Non-finite coordinates", testNonFinite},
        {"Multiple clusters", testMultipleClusters}
    };

    for (const auto& test : tests) {
        try {
            test.function();
            std::cout << "[PASS] " << test.name << '\n';
        }
        catch (const std::exception& e) {
            std::cerr << "[FAIL] " << test.name
                      << ": " << e.what() << '\n';
            return 1;
        }
    }

    std::cout << "\nAll " << tests.size()
              << " bounding box tests passed!\n";

    return 0;
}
