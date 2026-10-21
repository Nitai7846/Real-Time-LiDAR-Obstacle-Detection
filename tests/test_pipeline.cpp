#include "lidar/pipeline.h"

#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using CloudPtr = Cloud::Ptr;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

CloudPtr makeCloud() {
    return CloudPtr(new Cloud);
}

void addPoint(Cloud& cloud, float x, float y, float z) {
    pcl::PointXYZI point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = 1.0f;
    cloud.push_back(point);
}

void addGround(Cloud& cloud) {
    // Dense, flat plane at z = -1.5 m.
    for (int ix = -12; ix <= 12; ++ix) {
        for (int iy = -12; iy <= 12; ++iy) {
            addPoint(cloud, ix * 0.5f, iy * 0.5f, -1.5f);
        }
    }
}

void addObstacle(Cloud& cloud, float center_x, float center_y) {
    // 18 distinct points, compact enough to form one Euclidean cluster.
    for (int ix = 0; ix < 3; ++ix) {
        for (int iy = 0; iy < 3; ++iy) {
            for (int iz = 0; iz < 2; ++iz) {
                addPoint(cloud,
                         center_x + ix * 0.12f,
                         center_y + iy * 0.12f,
                         -0.2f + iz * 0.15f);
            }
        }
    }
}

CloudPtr makeScene(int obstacle_count) {
    CloudPtr cloud = makeCloud();
    addGround(*cloud);
    if (obstacle_count >= 1) {
        addObstacle(*cloud, -2.0f, 2.0f);
    }
    if (obstacle_count >= 2) {
        addObstacle(*cloud, 3.0f, -2.0f);
    }
    return cloud;
}

lidar::PipelineConfig config() {
    lidar::PipelineConfig cfg;
    cfg.preprocessing.voxel_size = 0.05f;
    cfg.preprocessing.min_x = -10.0f;
    cfg.preprocessing.max_x = 10.0f;
    cfg.preprocessing.min_y = -10.0f;
    cfg.preprocessing.max_y = 10.0f;
    cfg.preprocessing.min_z = -3.0f;
    cfg.preprocessing.max_z = 3.0f;

    cfg.ransac.max_iterations = 200;
    cfg.ransac.distance_threshold = 0.06f;
    cfg.ransac.random_seed = 42;
    cfg.ransac.max_tilt_degrees = 15.0f;
    cfg.ransac.expected_ground_z = -1.5f;
    cfg.ransac.ground_height_tolerance = 0.3f;

    cfg.clustering.distance_tolerance = 0.5f;
    cfg.clustering.min_cluster_size = 5;
    cfg.clustering.max_cluster_size = 500;
    return cfg;
}

void requirePartition(const lidar::PipelineResult& result) {
    require(result.filtered != nullptr, "null filtered cloud");
    require(result.separated.ground != nullptr, "null ground cloud");
    require(result.separated.nonground != nullptr, "null non-ground cloud");
    require(result.separated.ground->size() +
                result.separated.nonground->size() == result.filtered->size(),
            "ground/non-ground point conservation failed");
    require(result.boxes.size() == result.clusters.size(),
            "cluster-to-box correspondence failed");
}

void requireBoxesEncloseClusters(const lidar::PipelineResult& result) {
    for (std::size_t i = 0; i < result.clusters.size(); ++i) {
        require(result.boxes[i].has_value(), "missing box for valid cluster");
        const auto& box = *result.boxes[i];
        require(box.length >= 0 && box.width >= 0 && box.height >= 0,
                "negative box dimensions");
        for (std::size_t index : result.clusters[i]) {
            require(index < result.separated.nonground->size(),
                    "cluster index out of range");
            const auto& p = result.separated.nonground->points[index];
            const float eps = 0.0001f;
            require(p.x >= box.min_x - eps && p.x <= box.max_x + eps &&
                    p.y >= box.min_y - eps && p.y <= box.max_y + eps &&
                    p.z >= box.min_z - eps && p.z <= box.max_z + eps,
                    "bounding box fails to enclose a cluster point");
        }
    }
}

void testNullInput() {
    lidar::Pipeline pipeline(config());
    bool threw = false;
    try {
        pipeline.process(Cloud::ConstPtr{});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, "null input should throw std::invalid_argument");
}

void testEmptyInput() {
    lidar::Pipeline pipeline(config());
    auto result = pipeline.process(makeCloud());
    requirePartition(result);
    require(result.filtered->empty(), "filtered must be empty");
    require(result.separated.ground->empty(), "ground must be empty");
    require(result.separated.nonground->empty(), "non-ground must be empty");
    require(!result.ransac.found_plane, "empty cloud cannot define a plane");
    require(result.clusters.empty(), "empty cloud cannot contain clusters");
}

void testOnlyGround() {
    lidar::Pipeline pipeline(config());
    auto result = pipeline.process(makeScene(0));
    requirePartition(result);
    require(result.ransac.found_plane, "ground plane was not detected");
    require(result.separated.ground->size() == result.filtered->size(),
            "flat ground points were not all classified as ground");
    require(result.clusters.empty(), "ground-only scene yielded clusters");
}

void testOneObstacle() {
    lidar::Pipeline pipeline(config());
    auto result = pipeline.process(makeScene(1));
    requirePartition(result);
    require(result.ransac.found_plane, "ground plane not detected");
    require(result.clusters.size() == 1, "expected one obstacle cluster");
    requireBoxesEncloseClusters(result);
}

void testTwoObstacles() {
    lidar::Pipeline pipeline(config());
    auto result = pipeline.process(makeScene(2));
    requirePartition(result);
    require(result.clusters.size() == 2, "expected two obstacle clusters");
    requireBoxesEncloseClusters(result);
}

void testRoiExclusion() {
    lidar::Pipeline pipeline(config());
    auto cloud = makeScene(1);
    addPoint(*cloud, 100.0f, 0.0f, 0.0f);
    addPoint(*cloud, 0.0f, -100.0f, 0.0f);
    auto result = pipeline.process(cloud);
    requirePartition(result);
    for (const auto& p : result.filtered->points) {
        require(p.x >= -10.0f && p.x <= 10.0f &&
                p.y >= -10.0f && p.y <= 10.0f,
                "point outside ROI survived preprocessing");
    }
    require(result.filtered->size() < cloud->size(),
            "ROI filtering did not remove distant points");
}

void testAllFilteredOut() {
    lidar::Pipeline pipeline(config());
    auto cloud = makeCloud();
    addPoint(*cloud, 100.0f, 100.0f, 100.0f);
    auto result = pipeline.process(cloud);
    requirePartition(result);
    require(result.filtered->empty(), "ROI should reject every point");
    require(!result.ransac.found_plane, "no plane possible after empty ROI");
    require(result.clusters.empty(), "no clusters possible after empty ROI");
}

void testInsufficientPointsForPlane() {
    lidar::Pipeline pipeline(config());
    auto cloud = makeCloud();
    addPoint(*cloud, 0.0f, 0.0f, -0.2f);
    addPoint(*cloud, 0.5f, 0.5f, -0.1f);
    auto result = pipeline.process(cloud);
    requirePartition(result);
    require(!result.ransac.found_plane, "two points cannot define a plane");
    require(result.separated.ground->empty(),
            "without a plane, points must not be classified as ground");
    require(result.separated.nonground->size() == result.filtered->size(),
            "no-plane policy must preserve all filtered points as non-ground");
}

void testNoAccumulationAcrossFrames() {
    lidar::Pipeline pipeline(config());
    const auto two = pipeline.process(makeScene(2));
    const auto none = pipeline.process(makeScene(0));
    const auto one = pipeline.process(makeScene(1));
    require(two.clusters.size() == 2, "first frame should have two clusters");
    require(none.clusters.empty(), "second frame has stale clusters");
    require(none.boxes.empty(), "second frame has stale boxes");
    require(one.clusters.size() == 1, "third frame should have one cluster");
    require(one.boxes.size() == 1, "third frame should have one box");
    require(two.boxes.size() == 2, "prior results were mutated");
}

void testOriginalCloudUnchanged() {
    lidar::Pipeline pipeline(config());
    auto cloud = makeScene(1);
    const auto before = cloud->points;
    (void)pipeline.process(cloud);
    require(cloud->size() == before.size(), "input point count was modified");
    for (std::size_t i = 0; i < before.size(); ++i) {
        const auto& a = cloud->points[i];
        const auto& b = before[i];
        require(a.x == b.x && a.y == b.y && a.z == b.z &&
                    a.intensity == b.intensity, "input cloud was modified");
    }
}

void testClusterSizeConfiguration() {
    auto cfg = config();
    cfg.clustering.min_cluster_size = 50;  // Synthetic obstacle has 18 points.
    lidar::Pipeline pipeline(cfg);
    auto result = pipeline.process(makeScene(1));
    requirePartition(result);
    require(result.clusters.empty(), "minimum cluster size was ignored");
    require(result.boxes.empty(), "boxes should be empty without clusters");
}

void testRepeatability() {
    lidar::Pipeline pipeline(config());
    auto input = makeScene(2);
    const auto first = pipeline.process(input);
    const auto second = pipeline.process(input);
    requirePartition(first);
    requirePartition(second);
    require(first.filtered->size() == second.filtered->size(),
            "non-deterministic filtered count");
    require(first.ransac.inlier_indices == second.ransac.inlier_indices,
            "non-deterministic RANSAC ground inliers");
    require(first.clusters == second.clusters,
            "non-deterministic clusters");
    require(first.boxes.size() == second.boxes.size(),
            "non-deterministic box count");
}

}  // namespace

int main() {
    using Test = std::pair<std::string, std::function<void()>>;
    const std::vector<Test> tests = {
        {"null input", testNullInput},
        {"empty input", testEmptyInput},
        {"flat ground only", testOnlyGround},
        {"one obstacle", testOneObstacle},
        {"two obstacles and box enclosure", testTwoObstacles},
        {"ROI exclusion", testRoiExclusion},
        {"all points filtered out", testAllFilteredOut},
        {"too few points for a plane", testInsufficientPointsForPlane},
        {"no results accumulate across frames", testNoAccumulationAcrossFrames},
        {"original cloud unchanged", testOriginalCloudUnchanged},
        {"clustering configuration", testClusterSizeConfiguration},
        {"deterministic repeated processing", testRepeatability},
    };

    std::size_t passed = 0;
    for (const auto& test : tests) {
        try {
            test.second();
            ++passed;
            std::cout << "[PASS] " << test.first << '\n';
        } catch (const std::exception& e) {
            std::cerr << "[FAIL] " << test.first << ": " << e.what() << '\n';
        } catch (...) {
            std::cerr << "[FAIL] " << test.first << ": unknown error\n";
        }
    }

    std::cout << "\n" << passed << " / " << tests.size()
              << " Pipeline tests passed.\n";
    return passed == tests.size() ? 0 : 1;
}
