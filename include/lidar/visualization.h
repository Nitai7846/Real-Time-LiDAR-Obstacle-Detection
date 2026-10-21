
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/keyboard_event.h>

#include <cstddef>
#include <functional>
#include <vector>

#include "lidar/euclidean_clustering.h"
#include "lidar/bounding_box.h"

namespace lidar {

class Visualizer {
public:
    using Point = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<Point>;
    using CloudConstPtr = Cloud::ConstPtr;

    using Clusters = EuclideanClustering::Clusters;
    using Boxes = BoundingBox::Boxes;

    // Playback controls forwarded from the existing VTK keyboard callback.
    // The main application owns playback state, not the viewer.
    enum class PlaybackCommand {
        TogglePause,
        StepForward,
        StepBackward,
        Quit
    };
    using PlaybackHandler = std::function<void(PlaybackCommand)>;

    Visualizer();

    // Optional: single-frame mode keeps all original controls unchanged.
    void setPlaybackHandler(PlaybackHandler handler);

    // Display a single point cloud
    void showCloud(const CloudConstPtr& cloud);

    // Display ground and non-ground points
    void showGroundAndNonGround(
        const CloudConstPtr& ground,
        const CloudConstPtr& nonground
    );

    // Display ground, clusters and their bounding boxes
    void showGroundAndClusters(
        const CloudConstPtr& ground,
        const CloudConstPtr& nonground,
        const Clusters& clusters,
        const Boxes& boxes = {}
    );

    // Process GUI events once; call repeatedly from the playback loop.
    // This does NOT wait for the window to close.
    void spinOnce(int time_ms = 1);

    // True when the visualization window has been closed.
    bool wasStopped() const;

    // Keep the original blocking viewer for single-frame mode.
    void run();

private:
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    PlaybackHandler playback_handler_;


    // Visualization state
    bool ground_visible_ = true;
    bool isolate_cluster_ = false;

    std::size_t cluster_count_ = 0;
    std::size_t selected_cluster_ = 0;

    // Track which actors exist in the CURRENT frame. This prevents
    // attempts to change the visibility of non-existent actors.
    bool ground_actor_present_ = false;
    bool excluded_actor_present_ = false;
    std::vector<bool> cluster_actor_present_;

    // Tracks which clusters have a drawn bounding box.
    std::vector<bool> box_present_;

    // Reset the camera only on the first non-empty scene.
    bool camera_initialized_ = false;
    bool controls_printed_ = false;

    // Remove all frame-specific actors, leaving the viewer and camera alive.
    void clearFrameActors();
    void clearBoundingBoxes();
    void resetCameraOnce();

    // Keyboard handling
    void handleKeyboard(
        const pcl::visualization::KeyboardEvent& event
    );

    // Update cloud and bounding-box visibility
    void updateVisibility();
};

} // namespace lidar
