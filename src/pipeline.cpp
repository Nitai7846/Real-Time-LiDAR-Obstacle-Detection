#include "lidar/pipeline.h"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace lidar {

namespace {

using Clock = std::chrono::steady_clock;

// Return elapsed time in milliseconds; no truncation to integer milliseconds.
double millisecondsSince(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(
        Clock::now() - start
    ).count();
}

}  // namespace

Pipeline::Pipeline(PipelineConfig config)
    : config_(std::move(config)) {}

PipelineResult Pipeline::process(const CloudConstPtr& input) const {
    if (!input) {
        throw std::invalid_argument("Pipeline::process: input cloud is null");
    }

    const auto total_start = Clock::now();
    PipelineResult result;

    // An empty input has a well-defined, empty result. Avoid invoking
    // algorithms that require at least one point.
    if (input->empty()) {
        result.filtered.reset(new Cloud);
        result.separated.ground.reset(new Cloud);
        result.separated.nonground.reset(new Cloud);
        result.timings.total_ms = millisecondsSince(total_start);
        return result;
    }

    const auto preprocessing_start = Clock::now();
    result.filtered = Preprocessor::filter(input, config_.preprocessing);
    result.timings.preprocessing_ms = millisecondsSince(preprocessing_start);

    if (!result.filtered) {
        throw std::runtime_error("Pipeline::process: preprocessing returned null");
    }

    // All points may have been removed by the ROI / finite-point filters.
    if (result.filtered->empty()) {
        result.separated.ground.reset(new Cloud);
        result.separated.nonground.reset(new Cloud);
        result.timings.total_ms = millisecondsSince(total_start);
        return result;
    }

    // Three points are required for a 3D plane hypothesis.
    if (result.filtered->size() >= 3) {
        const auto ransac_start = Clock::now();
        result.ransac = RansacPlane::fit(result.filtered, config_.ransac);
        result.timings.ransac_ms = millisecondsSince(ransac_start);
    }

    // No valid ground plane: do not accidentally label any points as ground.
    if (!result.ransac.found_plane) {
        result.ransac.inlier_indices.clear();
    }

    const auto separation_start = Clock::now();
    result.separated = GroundSeparator::split(
        result.filtered, result.ransac.inlier_indices
    );
    result.timings.ground_separation_ms = millisecondsSince(separation_start);

    if (!result.separated.ground || !result.separated.nonground) {
        throw std::runtime_error("Pipeline::process: ground separation returned null");
    }

    if (!result.separated.nonground->empty()) {
        const auto clustering_start = Clock::now();
        const EuclideanClustering clustering(config_.clustering);
        result.clusters = clustering.extract(*result.separated.nonground);
        result.timings.clustering_ms = millisecondsSince(clustering_start);

        const auto boxes_start = Clock::now();
        result.boxes = BoundingBox::computeAll(
            *result.separated.nonground, result.clusters
        );
        result.timings.bounding_box_ms = millisecondsSince(boxes_start);
    }

    result.timings.total_ms = millisecondsSince(total_start);
    return result;
}

}  // namespace lidar
