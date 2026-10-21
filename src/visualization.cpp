
#include "lidar/visualization.h"
#include <iostream>

#include <pcl/visualization/point_cloud_color_handlers.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lidar {

namespace {

// Generate a different color for each cluster
std::array<int, 3> clusterColor(std::size_t index) {

    // Golden-angle spacing distributes colors around
    // the color wheel.
    const double hue = std::fmod(
        static_cast<double>(index) * 137.508,
        360.0
    );

    const double saturation = 0.85;
    const double value = 0.95;

    const double chroma = value * saturation;

    const double x = chroma * (
        1.0 - std::abs(std::fmod(hue / 60.0, 2.0) - 1.0)
    );

    const double m = value - chroma;

    double r = 0.0;
    double g = 0.0;
    double b = 0.0;

    if (hue < 60.0) {
        r = chroma;
        g = x;
    }
    else if (hue < 120.0) {
        r = x;
        g = chroma;
    }
    else if (hue < 180.0) {
        g = chroma;
        b = x;
    }
    else if (hue < 240.0) {
        g = x;
        b = chroma;
    }
    else if (hue < 300.0) {
        r = x;
        b = chroma;
    }
    else {
        r = chroma;
        b = x;
    }

    return {
        static_cast<int>(std::lround((r + m) * 255.0)),
        static_cast<int>(std::lround((g + m) * 255.0)),
        static_cast<int>(std::lround((b + m) * 255.0))
    };
}

} // anonymous namespace


// Constructor
Visualizer::Visualizer() {

    viewer_ =
        std::make_shared<pcl::visualization::PCLVisualizer>(
            "LiDAR Point Cloud Viewer"
        );

    viewer_->setBackgroundColor(0.05, 0.05, 0.05);

    viewer_->addCoordinateSystem(1.0);

    viewer_->initCameraParameters();

    viewer_->registerKeyboardCallback(
    [this](const pcl::visualization::KeyboardEvent& event) {
        handleKeyboard(event);
    }
);
}


// Install an optional callback for playback-specific keys.
// The callback must only update playback state; the app drives rendering.
void Visualizer::setPlaybackHandler(PlaybackHandler handler) {
    playback_handler_ = std::move(handler);
}


// Remove previously drawn bounding boxes
void Visualizer::clearBoundingBoxes() {

    for (std::size_t i = 0; i < box_present_.size(); ++i) {

        if (box_present_[i]) {
            viewer_->removeShape(
                "bbox_" + std::to_string(i)
            );
        }
    }

    box_present_.clear();
}


// Clear only scene actors; do not recreate the PCLVisualizer or
// remove the persistent coordinate system.
void Visualizer::clearFrameActors() {
    clearBoundingBoxes();
    viewer_->removeAllPointClouds();

    ground_actor_present_ = false;
    excluded_actor_present_ = false;
    cluster_actor_present_.clear();
}


// The original single-frame viewer automatically frames the first scene.
// Never reset after that: playback should preserve the user's camera.
void Visualizer::resetCameraOnce() {
    if (!camera_initialized_) {
        viewer_->resetCamera();
        camera_initialized_ = true;
    }
}


// Display a single point cloud
void Visualizer::showCloud(const CloudConstPtr& cloud) {

    if (!cloud) {
        throw std::invalid_argument(
            "Point cloud pointer is null"
        );
    }

    // Clear previous point-cloud actors
    clearFrameActors();

    cluster_count_ = 0;
    selected_cluster_ = 0;
    isolate_cluster_ = false;
    ground_visible_ = true;

    if (cloud->empty()) {
        return;
    }

    pcl::visualization::PointCloudColorHandlerCustom<Point>
        color_handler(cloud, 255, 255, 255);

    viewer_->addPointCloud<Point>(
        cloud,
        color_handler,
        "kitti_cloud"
    );

    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        2,
        "kitti_cloud"
    );

    resetCameraOnce();
}


// Display ground and non-ground point clouds
void Visualizer::showGroundAndNonGround(
    const CloudConstPtr& ground,
    const CloudConstPtr& nonground
) {

    if (!ground || !nonground) {
        throw std::invalid_argument(
            "Point cloud pointer is null"
        );
    }

    clearFrameActors();

    cluster_count_ = 0;
    selected_cluster_ = 0;
    isolate_cluster_ = false;
    ground_visible_ = true;

    // Ground: Green
    if (!ground->empty()) {

        pcl::visualization::PointCloudColorHandlerCustom<Point>
            ground_color(ground, 0, 255, 0);

        viewer_->addPointCloud<Point>(
            ground,
            ground_color,
            "ground_cloud"
        );

        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            2,
            "ground_cloud"
        );
        ground_actor_present_ = true;
    }

    // Non-ground: Orange
    if (!nonground->empty()) {

        pcl::visualization::PointCloudColorHandlerCustom<Point>
            nonground_color(nonground, 255, 165, 0);

        viewer_->addPointCloud<Point>(
            nonground,
            nonground_color,
            "nonground_cloud"
        );

        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            2,
            "nonground_cloud"
        );
    }

    if (!ground->empty() || !nonground->empty()) {
        resetCameraOnce();
    }
}


// Display ground and individually colored clusters
void Visualizer::showGroundAndClusters(
    const CloudConstPtr& ground,
    const CloudConstPtr& nonground,
    const Clusters& clusters,
    const Boxes& boxes
) {

    if (!ground || !nonground) {
        throw std::invalid_argument(
            "Point cloud pointer is null"
        );
    }

    // Each supplied box must correspond to the same indexed cluster
    if (!boxes.empty() && boxes.size() != clusters.size()) {
        throw std::invalid_argument(
            "Number of bounding boxes must match clusters"
        );
    }

    // Track points that belong to accepted clusters
    std::vector<bool> assigned(
        nonground->size(),
        false
    );

    // Validate cluster indices
    for (const auto& cluster : clusters) {

        for (std::size_t index : cluster) {

            if (index >= nonground->size()) {
                throw std::out_of_range(
                    "Cluster index exceeds cloud size"
                );
            }

            if (assigned[index]) {
                throw std::invalid_argument(
                    "Point belongs to multiple clusters"
                );
            }

            assigned[index] = true;
        }
    }

    // Clear previous point clouds and bounding boxes
    clearFrameActors();
    box_present_.assign(clusters.size(), false);
    cluster_actor_present_.assign(clusters.size(), false);

    // Preserve the user's ground toggle and cluster inspection mode as
    // frames advance. Clamp the selected index if a frame has fewer clusters.
    cluster_count_ = clusters.size();
    if (cluster_count_ == 0) {
        isolate_cluster_ = false;
        selected_cluster_ = 0;
    } else if (isolate_cluster_) {
        selected_cluster_ = std::min(selected_cluster_, cluster_count_ - 1);
    } else {
        selected_cluster_ = 0;
    }

    // Do not flood the terminal with instructions on every frame.
    if (!controls_printed_) {
        std::cout << "\n--- Visualization Controls ---\n";
        std::cout << "G: Toggle ground\n";
        std::cout << "N: Next cluster\n";
        std::cout << "P: Previous cluster\n";
        std::cout << "0-9: Select cluster by index\n";
        std::cout << "A: Show all clusters\n";
        controls_printed_ = true;
    }


    // STEP 1: Display ground in green
    if (!ground->empty()) {

        pcl::visualization::PointCloudColorHandlerCustom<Point>
            ground_color(ground, 0, 255, 0);

        viewer_->addPointCloud<Point>(
            ground,
            ground_color,
            "ground_cloud"
        );

        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            2,
            "ground_cloud"
        );
        ground_actor_present_ = true;
    }


    // STEP 2: Collect points excluded by clustering
    auto excluded_cloud = std::make_shared<Cloud>();

    for (std::size_t i = 0; i < nonground->size(); ++i) {

        if (!assigned[i]) {
            excluded_cloud->push_back(
                nonground->points[i]
            );
        }
    }

    // Display excluded points in gray
    if (!excluded_cloud->empty()) {

        pcl::visualization::PointCloudColorHandlerCustom<Point>
            excluded_color(excluded_cloud, 110, 110, 110);

        viewer_->addPointCloud<Point>(
            excluded_cloud,
            excluded_color,
            "excluded_cloud"
        );

        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            2,
            "excluded_cloud"
        );
        excluded_actor_present_ = true;
    }


    // STEP 3: Display each cluster in a unique color
    for (std::size_t i = 0; i < clusters.size(); ++i) {

        const auto& indices = clusters[i];

        if (indices.empty()) {
            continue;
        }

        // Create a cloud for this cluster
        auto cluster_cloud = std::make_shared<Cloud>();

        cluster_cloud->points.reserve(indices.size());

        // Copy points using original non-ground indices
        for (std::size_t index : indices) {

            cluster_cloud->push_back(
                nonground->points[index]
            );
        }

        // Generate a deterministic RGB color
        auto color = clusterColor(i);

        // Create a unique visualization ID
        std::string cluster_id =
            "cluster_" + std::to_string(i);

        pcl::visualization::PointCloudColorHandlerCustom<Point>
            cluster_color(
                cluster_cloud,
                color[0],
                color[1],
                color[2]
            );

        // Add cluster to the viewer
        viewer_->addPointCloud<Point>(
            cluster_cloud,
            cluster_color,
            cluster_id
        );
        cluster_actor_present_[i] = true;

        // Make clusters slightly more visible
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            3,
            cluster_id
        );

        // Draw a matching wireframe bounding box, if available
        if (!boxes.empty() && boxes[i].has_value()) {

            const auto& box = *boxes[i];
            const std::string box_id =
                "bbox_" + std::to_string(i);

            const double r = color[0] / 255.0;
            const double g = color[1] / 255.0;
            const double b = color[2] / 255.0;

            const bool added = viewer_->addCube(
                box.min_x, box.max_x,
                box.min_y, box.max_y,
                box.min_z, box.max_z,
                r, g, b,
                box_id
            );

            if (added) {
                viewer_->setShapeRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                    box_id
                );

                viewer_->setShapeRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                    2.0,
                    box_id
                );

                box_present_[i] = true;
            } else {
                std::cerr << "Failed to visualize bounding box "
                          << i << "\n";
            }
        }
    }

    // New actors start visible. Reapply persistent keyboard visibility
    // settings (ground toggle, cluster isolation) to the new frame.
    updateVisibility();

    if (!ground->empty() || !nonground->empty()) {
        resetCameraOnce();
    }
}


void Visualizer::updateVisibility() {

    // Show or hide ground
    if (ground_actor_present_) {
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY,
            ground_visible_ ? 1.0 : 0.0,
            "ground_cloud"
        );
    }

    // Show or hide each cluster
    for (std::size_t i = 0; i < cluster_count_; ++i) {

        bool visible =
            !isolate_cluster_ || i == selected_cluster_;

        std::string cluster_id =
            "cluster_" + std::to_string(i);

        if (i < cluster_actor_present_.size() && cluster_actor_present_[i]) {
            viewer_->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_OPACITY,
                visible ? 1.0 : 0.0,
                cluster_id
            );
        }

        // Keep each wireframe box in sync with its cluster
        if (i < box_present_.size() && box_present_[i]) {

            viewer_->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_OPACITY,
                visible ? 1.0 : 0.0,
                "bbox_" + std::to_string(i)
            );
        }
    }

    // Hide excluded points during individual inspection
    if (excluded_actor_present_) {
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY,
            isolate_cluster_ ? 0.0 : 1.0,
            "excluded_cloud"
        );
    }
}


// Handle keyboard input
void Visualizer::handleKeyboard(
    const pcl::visualization::KeyboardEvent& event
) {

    // Only respond when a key is pressed
    if (!event.keyDown()) {
        return;
    }

    const std::string key = event.getKeySym();

    // These keys are active only when sequence playback installs a handler.
    // Existing cluster keys (G/N/P/A/0-9) remain handled below.
    if (playback_handler_) {
        if (key == "space" || key == " ") {
            playback_handler_(PlaybackCommand::TogglePause);
            return;
        }
        if (key == "f" || key == "F") {
            playback_handler_(PlaybackCommand::StepForward);
            return;
        }
        if (key == "b" || key == "B") {
            playback_handler_(PlaybackCommand::StepBackward);
            return;
        }
        if (key == "q" || key == "Q") {
            playback_handler_(PlaybackCommand::Quit);
            return;
        }
    }

    // G: Toggle ground visibility
    if (key == "g" || key == "G") {

        ground_visible_ = !ground_visible_;

        updateVisibility();

        std::cout << "Ground: "
                  << (ground_visible_ ? "Visible" : "Hidden")
                  << "\n";

        return;
    }

    // No clusters available
    if (cluster_count_ == 0) {
        return;
    }

    // A: Show all clusters
    if (key == "a" || key == "A") {

        isolate_cluster_ = false;

        updateVisibility();

        std::cout << "Showing all "
                  << cluster_count_
                  << " clusters\n";

        return;
    }

    // N: Next cluster
    if (key == "n" || key == "N") {

        if (!isolate_cluster_) {
            selected_cluster_ = 0;
        } else {
            selected_cluster_ =
                (selected_cluster_ + 1) % cluster_count_;
        }

        isolate_cluster_ = true;
    }

    // P: Previous cluster
    else if (key == "p" || key == "P") {

        if (!isolate_cluster_) {
            selected_cluster_ = cluster_count_ - 1;
        } else {
            selected_cluster_ =
                (selected_cluster_ + cluster_count_ - 1)
                % cluster_count_;
        }

        isolate_cluster_ = true;
    }

    // Keys 0-9: Select cluster directly
    else if (key.size() == 1 &&
             key[0] >= '0' &&
             key[0] <= '9') {

        std::size_t index =
            static_cast<std::size_t>(key[0] - '0');

        if (index >= cluster_count_) {
            return;
        }

        selected_cluster_ = index;
        isolate_cluster_ = true;
    }

    else {
        return;
    }

    // Apply updated visibility
    updateVisibility();

    std::cout << "Showing Cluster "
              << selected_cluster_
              << " / "
              << cluster_count_ - 1
              << "\n";
}


// Process one iteration of the VTK event loop without taking control
// of the application's playback loop. Call on the main GUI thread.
void Visualizer::spinOnce(int time_ms) {
    if (time_ms < 1) {
        throw std::invalid_argument("spinOnce time_ms must be positive");
    }
    if (!wasStopped()) {
        viewer_->spinOnce(time_ms);
    }
}

bool Visualizer::wasStopped() const {
    return viewer_->wasStopped();
}


// Preserve the original blocking single-frame behavior.
void Visualizer::run() {
    while (!wasStopped()) {
        spinOnce(30);
    }
}

} // namespace lidar
