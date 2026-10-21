#include "lidar/pcl_comparison.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace lidar {
namespace {
using Clock = std::chrono::steady_clock;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Cluster = EuclideanClustering::Cluster;
using Clusters = EuclideanClustering::Clusters;

constexpr double kPi = 3.14159265358979323846;

double elapsedMs(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

std::optional<double> normalAngleDegrees(const PlaneModel& a,
                                         const pcl::ModelCoefficients& b) {
    if (b.values.size() < 4) return std::nullopt;
    const double aa = std::sqrt(double(a.a)*a.a + double(a.b)*a.b + double(a.c)*a.c);
    const double bb = std::sqrt(double(b.values[0])*b.values[0] +
                                double(b.values[1])*b.values[1] +
                                double(b.values[2])*b.values[2]);
    if (!(aa > 0.0 && bb > 0.0)) return std::nullopt;
    const double dot = (double(a.a)*b.values[0] + double(a.b)*b.values[1] +
                        double(a.c)*b.values[2]) / (aa * bb);
    return std::acos(std::clamp(std::abs(dot), 0.0, 1.0)) * 180.0 / kPi;
}

std::optional<double> inlierJaccard(const std::vector<std::size_t>& custom,
                                   const std::vector<int>& reference,
                                   std::size_t cloud_size) {
    if (custom.empty() && reference.empty()) return std::nullopt;
    std::vector<unsigned char> mask(cloud_size, 0);
    for (std::size_t i : custom) {
        if (i >= cloud_size) throw std::out_of_range("Custom RANSAC index out of range");
        mask[i] = 1;
    }
    std::size_t union_count = 0, intersection = 0;
    // Mask values: 0 neither, 1 custom, 2 reference only, 3 both.
    for (int i : reference) {
        if (i < 0 || static_cast<std::size_t>(i) >= cloud_size) {
            throw std::out_of_range("PCL RANSAC index out of range");
        }
        mask[static_cast<std::size_t>(i)] |= 2;
    }
    for (unsigned char v : mask) {
        if (v) ++union_count;
        if (v == 3) ++intersection;
    }
    return union_count ? std::optional<double>(double(intersection) / union_count)
                       : std::nullopt;
}

std::size_t pointCount(const Clusters& clusters) {
    std::size_t count = 0;
    for (const auto& cluster : clusters) count += cluster.size();
    return count;
}

// Index-label-free cluster match: each custom cluster chooses the PCL cluster
// with the largest Jaccard intersection-over-union. Not a one-to-one assignment.
std::optional<double> meanBestJaccard(const Clusters& custom,
                                      const Clusters& reference,
                                      std::size_t cloud_size) {
    if (custom.empty() && reference.empty()) return 1.0;
    if (custom.empty() || reference.empty()) return 0.0;
    std::vector<int> reference_label(cloud_size, -1);
    for (std::size_t j = 0; j < reference.size(); ++j) {
        for (std::size_t idx : reference[j]) {
            if (idx >= cloud_size) throw std::out_of_range("PCL cluster index out of range");
            reference_label[idx] = static_cast<int>(j);
        }
    }
    double sum = 0.0;
    for (const auto& a : custom) {
        std::vector<std::size_t> intersection(reference.size(), 0);
        for (std::size_t idx : a) {
            if (idx >= cloud_size) throw std::out_of_range("Custom cluster index out of range");
            const int j = reference_label[idx];
            if (j >= 0) ++intersection[static_cast<std::size_t>(j)];
        }
        double best = 0.0;
        for (std::size_t j = 0; j < reference.size(); ++j) {
            const auto denom = a.size() + reference[j].size() - intersection[j];
            if (denom) best = std::max(best, double(intersection[j]) / double(denom));
        }
        sum += best;
    }
    return sum / static_cast<double>(custom.size());
}

} // namespace

PclComparison PclComparator::compare(const PipelineResult& processed,
                                     const PipelineConfig& config) {
    if (!processed.filtered || !processed.separated.nonground) {
        throw std::invalid_argument("PclComparator: pipeline result lacks point clouds");
    }
    PclComparison out;
    const auto& filtered = processed.filtered;

    if (filtered->size() >= 3) {
        const auto start_custom = Clock::now();
        const auto custom = RansacPlane::fit(filtered, config.ransac);
        out.custom_ransac_ms = elapsedMs(start_custom);
        out.custom_ground_inliers = custom.inlier_indices.size();

        pcl::SACSegmentation<Point> reference;
        reference.setOptimizeCoefficients(false);
        reference.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        reference.setMethodType(pcl::SAC_RANSAC);
        reference.setAxis(Eigen::Vector3f::UnitZ());
        reference.setEpsAngle(config.ransac.max_tilt_degrees * kPi / 180.0);
        reference.setMaxIterations(config.ransac.max_iterations);
        reference.setDistanceThreshold(config.ransac.distance_threshold);
        reference.setInputCloud(filtered);
        pcl::PointIndices indices;
        pcl::ModelCoefficients coefficients;
        const auto start_pcl = Clock::now();
        reference.segment(indices, coefficients);
        out.pcl_ransac_ms = elapsedMs(start_pcl);
        out.pcl_ground_inliers = indices.indices.size();
        out.ground_inlier_jaccard = inlierJaccard(custom.inlier_indices,
                                                indices.indices, filtered->size());
        if (custom.found_plane && !indices.indices.empty()) {
            out.plane_normal_angle_deg = normalAngleDegrees(custom.plane, coefficients);
        }
    }

    // BOTH clustering implementations receive the same points. These points
    // come from CUSTOM RANSAC; the PCL plane result does not alter them.
    const auto& nonground = processed.separated.nonground;
    if (!nonground->empty()) {
        const auto start_custom = Clock::now();
        const auto custom = EuclideanClustering(config.clustering).extract(*nonground);
        out.custom_clustering_ms = elapsedMs(start_custom);
        out.custom_cluster_count = custom.size();
        out.custom_clustered_points = pointCount(custom);

        pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
        pcl::EuclideanClusterExtraction<Point> clustering;
        clustering.setClusterTolerance(config.clustering.distance_tolerance);
        clustering.setMinClusterSize(static_cast<int>(config.clustering.min_cluster_size));
        clustering.setMaxClusterSize(static_cast<int>(config.clustering.max_cluster_size));
        clustering.setSearchMethod(tree);
        clustering.setInputCloud(nonground);
        std::vector<pcl::PointIndices> pcl_clusters;
        const auto start_pcl = Clock::now();
        clustering.extract(pcl_clusters);
        out.pcl_clustering_ms = elapsedMs(start_pcl);
        Clusters reference;
        reference.reserve(pcl_clusters.size());
        for (const auto& item : pcl_clusters) {
            Cluster cluster;
            cluster.reserve(item.indices.size());
            for (int index : item.indices) {
                if (index < 0) throw std::runtime_error("PCL produced negative cluster index");
                cluster.push_back(static_cast<std::size_t>(index));
            }
            reference.push_back(std::move(cluster));
        }
        out.pcl_cluster_count = reference.size();
        out.pcl_clustered_points = pointCount(reference);
        out.mean_best_cluster_jaccard = meanBestJaccard(custom, reference, nonground->size());
    } else {
        out.mean_best_cluster_jaccard = 1.0;
    }
    return out;
}

} // namespace lidar
