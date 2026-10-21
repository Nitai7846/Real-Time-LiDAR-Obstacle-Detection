
#include "lidar/euclidean_clustering.h"

#include <array>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace lidar;

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Clusters = EuclideanClustering::Clusters;

// Helper function to create synthetic point clouds
Cloud makeCloud(const std::vector<std::array<float, 3>>& coordinates) {

    Cloud cloud;

    for (const auto& xyz : coordinates) {
        Point point{};

        point.x = xyz[0];
        point.y = xyz[1];
        point.z = xyz[2];

        cloud.push_back(point);
    }

    return cloud;
}

// Throw an error if a test fails
void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

int main() {

    try {

        ClusteringConfig config;
        config.distance_tolerance = 0.5f;
        config.min_cluster_size = 1;
        config.max_cluster_size = 100;

        EuclideanClustering clustering(config);


        // Test 1: Empty point cloud
        {
            Cloud cloud;

            auto clusters = clustering.extract(cloud);

            check(clusters.empty(),
                  "Empty cloud test failed");

            std::cout << "[PASS] Empty cloud\n";
        }


        // Test 2: Single point
        {
            Cloud cloud = makeCloud({
                {0.0f, 0.0f, 0.0f}
            });

            auto clusters = clustering.extract(cloud);

            check(clusters == Clusters{{0}},
                  "Single point test failed");

            std::cout << "[PASS] Single point\n";
        }


        // Test 3: Two separated clusters
        {
            Cloud cloud = makeCloud({
                {0.0f, 0.0f, 0.0f},
                {0.2f, 0.0f, 0.0f},
                {5.0f, 0.0f, 0.0f},
                {5.2f, 0.0f, 0.0f}
            });

            auto clusters = clustering.extract(cloud);

            check(clusters == (Clusters{{0, 1}, {2, 3}}),
                  "Two clusters test failed");

            std::cout << "[PASS] Two separated clusters\n";
        }


        // Test 4: Connected chain of points
        {
            Cloud cloud = makeCloud({
                {0.0f, 0.0f, 0.0f},
                {0.3f, 0.0f, 0.0f},
                {0.6f, 0.0f, 0.0f},
                {0.9f, 0.0f, 0.0f}
            });

            auto clusters = clustering.extract(cloud);

            check(clusters == Clusters{{0, 1, 2, 3}},
                  "Connected chain test failed");

            std::cout << "[PASS] Connected chain\n";
        }


        // Test 5: Duplicate point coordinates
        {
            Cloud cloud = makeCloud({
                {1.0f, 1.0f, 1.0f},
                {1.0f, 1.0f, 1.0f},
                {5.0f, 5.0f, 5.0f}
            });

            auto clusters = clustering.extract(cloud);

            check(clusters == (Clusters{{0, 1}, {2}}),
                  "Duplicate coordinates test failed");

            std::cout << "[PASS] Duplicate coordinates\n";
        }


        // Test 6: Cluster-size filtering
        {
            ClusteringConfig filter_config;

            filter_config.distance_tolerance = 0.5f;
            filter_config.min_cluster_size = 2;
            filter_config.max_cluster_size = 2;

            EuclideanClustering filtered(filter_config);

            Cloud cloud = makeCloud({
                {0.0f, 0.0f, 0.0f},
                {0.2f, 0.0f, 0.0f},
                {5.0f, 0.0f, 0.0f},
                {5.2f, 0.0f, 0.0f},
                {5.4f, 0.0f, 0.0f},
                {10.0f, 0.0f, 0.0f}
            });

            auto clusters = filtered.extract(cloud);

            check(clusters == Clusters{{0, 1}},
                  "Cluster-size filtering test failed");

            std::cout << "[PASS] Cluster-size filtering\n";
        }


        // Test 7: Exact distance threshold
        {
            Cloud cloud = makeCloud({
                {0.0f, 0.0f, 0.0f},
                {0.5f, 0.0f, 0.0f},
                {2.0f, 0.0f, 0.0f}
            });

            auto clusters = clustering.extract(cloud);

            check(clusters == (Clusters{{0, 1}, {2}}),
                  "Exact distance threshold test failed");

            std::cout << "[PASS] Exact distance threshold\n";
        }


        // Test 8: Isolated points
        {
            Cloud cloud = makeCloud({
                {0.0f, 0.0f, 0.0f},
                {3.0f, 0.0f, 0.0f},
                {6.0f, 0.0f, 0.0f}
            });

            auto clusters = clustering.extract(cloud);

            check(clusters == (Clusters{{0}, {1}, {2}}),
                  "Isolated points test failed");

            std::cout << "[PASS] Isolated points\n";
        }


        // Test 9: No overlapping cluster assignments
        {
            Cloud cloud = makeCloud({
                {0.0f, 0.0f, 0.0f},
                {0.2f, 0.0f, 0.0f},
                {0.4f, 0.0f, 0.0f},
                {5.0f, 0.0f, 0.0f},
                {5.2f, 0.0f, 0.0f}
            });

            auto clusters = clustering.extract(cloud);

            std::vector<bool> seen(cloud.size(), false);

            for (const auto& cluster : clusters) {

                for (std::size_t index : cluster) {

                    check(index < cloud.size(),
                          "Invalid point index");

                    check(!seen[index],
                          "Point assigned to multiple clusters");

                    seen[index] = true;
                }
            }

            for (bool assigned : seen) {

                check(assigned,
                      "Point was not assigned to any cluster");
            }

            std::cout << "[PASS] No overlapping assignments\n";
        }


        // Test 10: Deterministic results
        {
            Cloud cloud = makeCloud({
                {0.0f, 0.0f, 0.0f},
                {0.3f, 0.0f, 0.0f},
                {4.0f, 0.0f, 0.0f},
                {4.3f, 0.0f, 0.0f}
            });

            auto first = clustering.extract(cloud);
            auto second = clustering.extract(cloud);

            check(first == second,
                  "Deterministic results test failed");

            std::cout << "[PASS] Deterministic results\n";
        }


        std::cout << "\nAll 10 clustering tests passed!\n";

    }
    catch (const std::exception& error) {

        std::cerr << "[FAIL] " << error.what() << '\n';
        return 1;
    }

    return 0;
}
