cmake_minimum_required(VERSION 3.20)

project(Lidar_Detection LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED COMPONENTS
    common
    io
    visualization
    filters
    kdtree
    search
    segmentation
)

# Create our core LiDAR library
add_library(lidar_core
    src/frame_reader.cpp
    src/visualization.cpp
    src/preprocessing.cpp
    src/ransac_plane.cpp
    src/ground_separator.cpp
    src/kdtree3d.cpp
    src/euclidean_clustering.cpp
    src/bounding_box.cpp
    src/frame_sequence.cpp
    src/pipeline.cpp
    src/pcl_comparison.cpp
    src/performance_report.cpp
)

# Tell CMake where our headers are located
target_include_directories(lidar_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

# Link PCL to our core library
target_link_libraries(lidar_core PUBLIC
    ${PCL_LIBRARIES}
)

target_compile_options(lidar_core PRIVATE
    -Wall -Wextra -Wpedantic
)

# Create the main executable
add_executable(lidar_demo apps/main.cpp)

target_link_libraries(lidar_demo PRIVATE lidar_core)

enable_testing()

# Frame Reader Test
add_executable(test_frame_reader
    tests/test_frame_reader.cpp
)

target_link_libraries(test_frame_reader PRIVATE lidar_core)

add_test(
    NAME FrameReaderTest
    COMMAND test_frame_reader
)

# Preprocessing Test
add_executable(test_preprocessing
    tests/test_preprocessing.cpp
)

target_link_libraries(test_preprocessing PRIVATE lidar_core)

add_test(
    NAME PreprocessingTest
    COMMAND test_preprocessing
)

# RANSAC Ground Plane Test
add_executable(test_ransac_plane
    tests/test_ransac_plane.cpp
)

target_link_libraries(test_ransac_plane PRIVATE lidar_core)

add_test(
    NAME RansacPlaneTest
    COMMAND test_ransac_plane
)

# Ground Separator Test
add_executable(test_ground_separator
    tests/test_ground_separator.cpp
)

target_link_libraries(test_ground_separator PRIVATE lidar_core)

add_test(
    NAME GroundSeparatorTest
    COMMAND test_ground_separator
)

# Custom KD-Tree Test
add_executable(test_kdtree3d
    tests/test_kdtree3d.cpp
)

target_link_libraries(test_kdtree3d PRIVATE lidar_core)

add_test(
    NAME KdTree3DTest
    COMMAND test_kdtree3d
)

# Custom Euclidean Clustering Test
add_executable(test_euclidean_clustering
    tests/test_euclidean_clustering.cpp
)

target_link_libraries(test_euclidean_clustering
    PRIVATE lidar_core
)

add_test(
    NAME EuclideanClusteringTest
    COMMAND test_euclidean_clustering
)

# Bounding Box Tests
add_executable(test_bounding_box
    tests/test_bounding_box.cpp
)

target_link_libraries(test_bounding_box
    PRIVATE lidar_core
)

add_test(
    NAME BoundingBoxTests
    COMMAND test_bounding_box
)

# Frame Sequence Test (Milestone 10)
add_executable(test_frame_sequence
    tests/test_frame_sequence.cpp
)

target_link_libraries(test_frame_sequence
    PRIVATE lidar_core
)

add_test(
    NAME FrameSequenceTest
    COMMAND test_frame_sequence
)

# Pipeline Orchestration Test (Milestone 10)
add_executable(test_pipeline
    tests/test_pipeline.cpp
)

target_link_libraries(test_pipeline PRIVATE lidar_core)

add_test(
    NAME PipelineTest
    COMMAND test_pipeline
)

# Per-stage timing instrumentation test (Milestone 11)
add_executable(test_pipeline_timing
    tests/test_pipeline_timing.cpp
)

target_link_libraries(test_pipeline_timing PRIVATE lidar_core)

add_test(
    NAME PipelineTimingTest
    COMMAND test_pipeline_timing
)

# Milestone 11: all new test files registered together.
add_executable(test_pcl_comparison tests/test_pcl_comparison.cpp)
target_link_libraries(test_pcl_comparison PRIVATE lidar_core)
add_test(NAME PclComparisonTest COMMAND test_pcl_comparison)

add_executable(test_metrics_logger tests/test_metrics_logger.cpp)
target_link_libraries(test_metrics_logger PRIVATE lidar_core)
add_test(NAME MetricsLoggerTest COMMAND test_metrics_logger)

add_executable(test_performance_report tests/test_performance_report.cpp)
target_link_libraries(test_performance_report PRIVATE lidar_core)
add_test(NAME PerformanceReportTest COMMAND test_performance_report)
