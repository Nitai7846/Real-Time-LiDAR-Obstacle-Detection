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
    pcl::PointXYZI p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = 1.0f;
    cloud.push_back(p);
}

CloudPtr makeScene(bool with_obstacle) {
    auto cloud = makeCloud();

    // The same synthetic geometry/configuration already exercised by the
    // existing Milestone 10 pipeline tests.
    for (int ix = -12; ix <= 12; ++ix) {
        for (int iy = -12; iy <= 12; ++iy) {
            addPoint(*cloud, ix * 0.5f, iy * 0.5f, -1.5f);
        }
    }
    if (with_obstacle) {
        for (int ix = 0; ix < 3; ++ix) {
            for (int iy = 0; iy < 3; ++iy) {
                for (int iz = 0; iz < 2; ++iz) {
                    addPoint(*cloud,
                             -2.0f + ix * 0.12f,
                             2.0f + iy * 0.12f,
                             -0.2f + iz * 0.15f);
                }
            }
        }
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

void verifyNonnegativeFinite(const lidar::PipelineTimings& t) {
    const double values[] = {
        t.preprocessing_ms, t.ransac_ms, t.ground_separation_ms,
        t.clustering_ms, t.bounding_box_ms, t.total_ms
    };
    for (double value : values) {
        require(std::isfinite(value), "timing was not finite");
        require(value >= 0.0, "timing was negative");
    }
}

void verifyTotalCoversStages(const lidar::PipelineTimings& t) {
    const double stage_sum = t.preprocessing_ms + t.ransac_ms +
        t.ground_separation_ms + t.clustering_ms + t.bounding_box_ms;
    // Clock readings and floating-point rounding can introduce tiny errors;
    // do not compare timing measurements for exact equality.
    require(stage_sum <= t.total_ms + 0.01,
            "sum of stage times exceeds total processing time");
}

void verifyResultsValid(const lidar::PipelineResult& result) {
    require(result.filtered != nullptr, "filtered cloud is null");
    require(result.separated.ground != nullptr, "ground cloud is null");
    require(result.separated.nonground != nullptr, "nonground cloud is null");
    require(result.filtered->size() == result.separated.ground->size() +
                result.separated.nonground->size(),
            "point count conservation failed");
    require(result.boxes.size() == result.clusters.size(),
            "cluster/box correspondence changed");
    verifyNonnegativeFinite(result.timings);
    verifyTotalCoversStages(result.timings);
}

void testNullInputStillThrows() {
    lidar::Pipeline pipeline(config());
    bool caught = false;
    try {
        (void)pipeline.process(Cloud::ConstPtr{});
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    require(caught, "null input should throw invalid_argument");
}

void testEmptyInputSkipsAllStages() {
    lidar::Pipeline pipeline(config());
    const auto result = pipeline.process(makeCloud());
    verifyResultsValid(result);
    const auto& t = result.timings;
    require(result.filtered->empty(), "empty input should remain empty");
    require(t.preprocessing_ms == 0.0, "preprocessing should be skipped");
    require(t.ransac_ms == 0.0, "RANSAC should be skipped");
    require(t.ground_separation_ms == 0.0, "separation should be skipped");
    require(t.clustering_ms == 0.0, "clustering should be skipped");
    require(t.bounding_box_ms == 0.0, "boxes should be skipped");
}

void testEntireCloudRemovedByROI() {
    lidar::Pipeline pipeline(config());
    auto cloud = makeCloud();
    addPoint(*cloud, 100.0f, 100.0f, 100.0f);
    const auto result = pipeline.process(cloud);
    verifyResultsValid(result);
    require(result.filtered->empty(), "ROI should remove out-of-range point");
    require(result.timings.ransac_ms == 0.0, "RANSAC should be skipped");
    require(result.timings.ground_separation_ms == 0.0,
            "separation should be skipped");
    require(result.timings.clustering_ms == 0.0,
            "clustering should be skipped");
    require(result.timings.bounding_box_ms == 0.0,
            "boxes should be skipped");
}

void testInsufficientPointsSkipsRansac() {
    lidar::Pipeline pipeline(config());
    auto cloud = makeCloud();
    addPoint(*cloud, 0.0f, 0.0f, 0.0f);
    addPoint(*cloud, 1.0f, 0.0f, 0.0f);
    const auto result = pipeline.process(cloud);
    verifyResultsValid(result);
    require(result.filtered->size() == 2, "both valid points should survive");
    require(!result.ransac.found_plane, "two points cannot define a plane");
    require(result.timings.ransac_ms == 0.0, "RANSAC should be skipped");
    require(result.separated.ground->empty(), "all points should be non-ground");
}

void testGroundOnlyScene() {
    lidar::Pipeline pipeline(config());
    const auto result = pipeline.process(makeScene(false));
    verifyResultsValid(result);
    require(result.ransac.found_plane, "known flat ground not detected");
    require(result.clusters.empty(), "ground-only scene has clusters");
}

void testObstacleSceneRetainsBoxes() {
    lidar::Pipeline pipeline(config());
    const auto result = pipeline.process(makeScene(true));
    verifyResultsValid(result);
    require(result.ransac.found_plane, "ground plane not detected");
    require(result.clusters.size() == 1, "expected one obstacle cluster");
    require(result.boxes.size() == 1 && result.boxes[0].has_value(),
            "expected one valid obstacle bounding box");
}

void testRepeatedFramesHaveIndependentTiming() {
    lidar::Pipeline pipeline(config());
    const auto first = pipeline.process(makeScene(true));
    const auto empty = pipeline.process(makeCloud());
    const auto third = pipeline.process(makeScene(true));
    verifyResultsValid(first);
    verifyResultsValid(empty);
    verifyResultsValid(third);
    require(first.clusters.size() == 1, "first result was modified");
    require(empty.clusters.empty(), "empty frame retained clusters");
    require(empty.timings.ransac_ms == 0.0, "empty frame retained RANSAC time");
    require(empty.timings.clustering_ms == 0.0, "empty frame retained cluster time");
    require(third.clusters.size() == 1, "third frame lacked obstacle");
}

void testTotalIsAtLeastEachStage() {
    lidar::Pipeline pipeline(config());
    const auto result = pipeline.process(makeScene(true));
    const auto& t = result.timings;
    verifyNonnegativeFinite(t);
    verifyTotalCoversStages(t);
    require(t.total_ms >= t.preprocessing_ms, "total < preprocessing");
    require(t.total_ms >= t.ransac_ms, "total < RANSAC");
    require(t.total_ms >= t.ground_separation_ms, "total < separation");
    require(t.total_ms >= t.clustering_ms, "total < clustering");
    require(t.total_ms >= t.bounding_box_ms, "total < bounding box");
}

void testInputUnchanged() {
    lidar::Pipeline pipeline(config());
    auto cloud = makeScene(true);
    const auto original = cloud->points;
    (void)pipeline.process(cloud);
    require(cloud->size() == original.size(), "input point count changed");
    for (std::size_t i = 0; i < original.size(); ++i) {
        const auto& before = original[i];
        const auto& after = cloud->points[i];
        require(before.x == after.x && before.y == after.y &&
                before.z == after.z && before.intensity == after.intensity,
                "input point changed");
    }
}

}  // namespace

int main() {
    using Test = std::pair<std::string, std::function<void()>>;
    const std::vector<Test> tests = {
        {"null input unchanged", testNullInputStillThrows},
        {"empty input skips stages", testEmptyInputSkipsAllStages},
        {"all points removed by ROI", testEntireCloudRemovedByROI},
        {"two points skip RANSAC", testInsufficientPointsSkipsRansac},
        {"ground-only timings", testGroundOnlyScene},
        {"obstacle and box timings", testObstacleSceneRetainsBoxes},
        {"independent per-frame timings", testRepeatedFramesHaveIndependentTiming},
        {"total time covers stage times", testTotalIsAtLeastEachStage},
        {"input cloud unchanged", testInputUnchanged},
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
              << " PipelineTiming tests passed.\n";
    return passed == tests.size() ? 0 : 1;
}
