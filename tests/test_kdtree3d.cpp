#include "lidar/kdtree3d.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using Tree = lidar::KdTree3D;
using Point = pcl::PointXYZI;

Point makePoint(float x, float y, float z) {
    Point p{};
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = 0.0f;
    return p;
}

void check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::string showIndices(const std::vector<std::size_t>& indices) {
    std::ostringstream out;
    out << "[";
    for (std::size_t i = 0; i < indices.size(); ++i) {
        if (i > 0) out << ", ";
        out << indices[i];
    }
    out << "]";
    return out.str();
}

void checkNeighbors(const std::vector<std::size_t>& actual,
                    const std::vector<std::size_t>& expected,
                    const std::string& testName) {
    if (actual != expected) {
        throw std::runtime_error(
            testName + ": expected " + showIndices(expected) +
            ", got " + showIndices(actual)
        );
    }
}

std::vector<std::size_t> bruteForce(
    const std::vector<Point>& points,
    const Point& target,
    float radius
) {
    const double r = static_cast<double>(radius);
    const double r2 = r * r;
    std::vector<std::size_t> result;

    for (std::size_t i = 0; i < points.size(); ++i) {
        const double dx = static_cast<double>(points[i].x) - target.x;
        const double dy = static_cast<double>(points[i].y) - target.y;
        const double dz = static_cast<double>(points[i].z) - target.z;
        if (dx * dx + dy * dy + dz * dz <= r2) {
            result.push_back(i);
        }
    }
    return result;
}

template <typename Function>
void checkInvalidArgument(Function action, const std::string& testName) {
    try {
        action();
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error(testName + ": expected std::invalid_argument");
}

void testEmptyTree() {
    Tree tree;
    check(tree.size() == 0, "Empty tree size should be zero");
    checkNeighbors(tree.radiusSearch(makePoint(0, 0, 0), 1.0f),
                   {}, "Empty tree query");
}

void testSinglePoint() {
    Tree tree;
    const Point p = makePoint(2, 3, 4);
    tree.insert(p, 17);

    check(tree.size() == 1, "Single point size should be one");
    checkNeighbors(tree.radiusSearch(p, 0.0f), {17}, "Zero radius");
    checkNeighbors(tree.radiusSearch(makePoint(20, 30, 40), 1.0f),
                   {}, "No neighbors");
}

void testThreeDimensionalDistance() {
    Tree tree;
    tree.insert(makePoint(0, 0, 0), 0);
    tree.insert(makePoint(1, 1, 1), 1);
    tree.insert(makePoint(-1, -1, -1), 2);
    tree.insert(makePoint(2, 2, 2), 3);
    tree.insert(makePoint(0, 0, 2), 4);

    checkNeighbors(tree.radiusSearch(makePoint(0, 0, 0), 2.0f),
                   {0, 1, 2, 4}, "3D radius search");
}

void testPointsOnBoundary() {
    Tree tree;
    tree.insert(makePoint(0, 0, 0), 10);
    tree.insert(makePoint(3, 4, 0), 20);   // exactly 5 meters away
    tree.insert(makePoint(0, 0, 5), 30);   // exactly 5 meters away
    tree.insert(makePoint(-5, 0, 0), 40);  // exactly 5 meters away
    tree.insert(makePoint(6, 0, 0), 50);   // outside

    checkNeighbors(tree.radiusSearch(makePoint(0, 0, 0), 5.0f),
                   {10, 20, 30, 40}, "Inclusive radius boundary");
}

void testDuplicateCoordinatesAndIndices() {
    Tree tree;
    const Point p = makePoint(1, 2, 3);
    tree.insert(p, 11);
    tree.insert(p, 12);  // same position, different index
    tree.insert(p, 11);  // repeated index must not repeat in output
    tree.insert(makePoint(9, 9, 9), 13);

    check(tree.size() == 4, "All four insertions should create nodes");
    checkNeighbors(tree.radiusSearch(p, 0.0f), {11, 12},
                   "Duplicate coordinates and indices");
}

void testSearchAcrossSplittingPlane() {
    Tree tree;
    tree.insert(makePoint(0, 0, 0), 0);       // root splits on X = 0
    tree.insert(makePoint(-0.2f, 0, 0), 1);   // left branch
    tree.insert(makePoint(0.2f, 0, 0), 2);    // right branch
    tree.insert(makePoint(-10, 0, 0), 3);     // far left
    tree.insert(makePoint(10, 0, 0), 4);      // far right

    checkNeighbors(tree.radiusSearch(makePoint(0.1f, 0, 0), 0.35f),
                   {0, 1, 2}, "Search must check both sides of split");
}

void testInvalidInputs() {
    Tree tree;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    checkInvalidArgument(
        [&]() { tree.insert(makePoint(nan, 0, 0), 0); },
        "Reject NaN insertion"
    );
    checkInvalidArgument(
        [&]() { tree.insert(makePoint(0, inf, 0), 1); },
        "Reject infinity insertion"
    );
    checkInvalidArgument(
        [&]() { tree.radiusSearch(makePoint(0, 0, 0), -1.0f); },
        "Reject negative radius"
    );
    checkInvalidArgument(
        [&]() { tree.radiusSearch(makePoint(0, 0, 0), inf); },
        "Reject infinite radius"
    );
    checkInvalidArgument(
        [&]() { tree.radiusSearch(makePoint(0, 0, nan), 1.0f); },
        "Reject invalid target"
    );
    check(tree.size() == 0, "Rejected insertions must not change size");
}

void testRandomizedAgainstBruteForceAndPCL() {
    std::mt19937 rng(42); // reproducible random tests
    std::uniform_real_distribution<float> position(-20.0f, 20.0f);

    Tree tree;
    std::vector<Point> points;
    auto cloud = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);

    for (std::size_t i = 0; i < 250; ++i) {
        Point p = makePoint(position(rng), position(rng), position(rng));
        points.push_back(p);
        cloud->push_back(p);
        tree.insert(p, i);
    }

    check(tree.size() == points.size(), "Random cloud insertion count");

    pcl::KdTreeFLANN<Point> pclTree;
    pclTree.setInputCloud(cloud);

    const std::vector<float> radii = {0.5f, 2.0f, 5.0f, 10.0f};
    for (int query = 0; query < 60; ++query) {
        // Half of the queries use real cloud points; half use new positions.
        Point target = (query % 2 == 0)
            ? points[static_cast<std::size_t>(query)]
            : makePoint(position(rng), position(rng), position(rng));

        for (float radius : radii) {
            const auto actual = tree.radiusSearch(target, radius);
            const auto expected = bruteForce(points, target, radius);
            checkNeighbors(actual, expected, "Randomized vs brute force");

            std::vector<int> pclIndices;
            std::vector<float> pclSquaredDistances;
            pclTree.radiusSearch(target, radius, pclIndices,
                                 pclSquaredDistances);

            std::vector<std::size_t> pclExpected;
            for (int index : pclIndices) {
                pclExpected.push_back(static_cast<std::size_t>(index));
            }
            std::sort(pclExpected.begin(), pclExpected.end());
            checkNeighbors(actual, pclExpected, "Randomized vs PCL KD-Tree");
        }
    }
}

template <typename Function>
void runTest(const std::string& name, Function test) {
    test();
    std::cout << "[PASS] " << name << '\n';
}

int main() {
    try {
        runTest("Empty tree", testEmptyTree);
        runTest("Single point and no neighbors", testSinglePoint);
        runTest("3D distances", testThreeDimensionalDistance);
        runTest("Exact radius boundary", testPointsOnBoundary);
        runTest("Duplicate coordinates and indices", testDuplicateCoordinatesAndIndices);
        runTest("Search across splitting plane", testSearchAcrossSplittingPlane);
        runTest("Invalid inputs", testInvalidInputs);
        runTest("Randomized vs brute force and PCL", testRandomizedAgainstBruteForceAndPCL);

        std::cout << "\nAll 8 KD-Tree tests passed!\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[FAIL] " << error.what() << '\n';
        return 1;
    }
}
