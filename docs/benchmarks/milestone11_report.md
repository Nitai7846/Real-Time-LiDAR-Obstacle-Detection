# LiDAR Pipeline Performance — Milestone 11

Input: `/Volumes/One Touch/data_object_velodyne/training/velodyne`  
Frames: 200  
Record compiler optimization flags and OS/PCL versions alongside these measurements.

Invocation: `"./build/lidar_demo" "/Volumes/One Touch/data_object_velodyne/training/velodyne" "--headless" "--max-frames" "200" "--compare-pcl" "--csv" "milestone11_benchmark.csv" "--report" "milestone11_report.md"`

## Configuration

- Preprocessing voxel size (m): 0.2
- ROI x/y/z (m): [-15, 35] / [-10, 10] / [-3, 4]
- RANSAC max iterations: 100; distance threshold (m): 0.2; random seed: 42; max ground tilt (deg): 25
- Expected ground Z (m): unset; height tolerance (m): 1
- Cluster tolerance (m): 0.5; min/max points: 20/10000

## Per-frame latency statistics

All values below are milliseconds except overlap/angle rows. Skipped stages are recorded as zero. p95 uses linear interpolation at (N-1)*0.95.

| Measurement | n | Mean | Median | p95 | Max |
|---|---:|---:|---:|---:|---:|
| Disk load (ms) | 200 | 32.520 | 32.575 | 37.903 | 39.497 |
| Preprocessing (ms) | 200 | 3.986 | 3.925 | 4.785 | 6.852 |
| RANSAC (pipeline ms) | 200 | 2.145 | 2.130 | 2.797 | 3.888 |
| Ground separation (ms) | 200 | 0.078 | 0.075 | 0.123 | 0.163 |
| Clustering (pipeline ms) | 200 | 12.453 | 11.869 | 28.292 | 59.598 |
| Bounding boxes (ms) | 200 | 0.013 | 0.013 | 0.027 | 0.052 |
| Pipeline total (ms) | 200 | 18.675 | 17.624 | 34.963 | 66.792 |
| Load + pipeline (ms) | 200 | 51.196 | 49.579 | 67.395 | 95.112 |

Estimated load+pipeline-only throughput: **19.533 FPS**.

100 ms deadline using p95 load+pipeline: **meets** (not proof of sustained real-time operation).

Observed wall throughput (includes optional comparisons, CSV, console): **13.420 FPS**.

## Custom algorithm vs PCL references

Same filtered input for both plane fits, same CUSTOM-RANSAC non-ground input for both clusterers, identical numeric iteration/distance/cluster-size thresholds. PCL uses SACMODEL_PERPENDICULAR_PLANE constrained to the Z axis; random sample sequences and implementation details differ. The custom expected-ground-height option (if enabled) has no exact PCL counterpart. Times are separate benchmark reruns, NOT additional stages of the normal pipeline.

| Measurement | n | Mean | Median | p95 | Max |
|---|---:|---:|---:|---:|---:|
| Custom RANSAC benchmark (ms) | 200 | 2.119 | 2.089 | 2.825 | 3.700 |
| PCL RANSAC benchmark (ms) | 200 | 0.374 | 0.230 | 0.859 | 6.012 |
| Custom clustering benchmark (ms) | 200 | 12.489 | 11.517 | 28.366 | 60.132 |
| PCL clustering benchmark (ms) | 200 | 8.154 | 7.529 | 17.371 | 25.508 |
| Ground inlier Jaccard (0–1) | 200 | 0.868 | 0.909 | 0.990 | 0.998 |
| Plane normal difference (deg) | 200 | 0.757 | 0.553 | 2.575 | 7.000 |
| Cluster mean-best Jaccard (0–1) | 200 | 1.000 | 1.000 | 1.000 | 1.000 |

Frames with matching cluster counts: 200/200. Matching ground inlier counts: 0/200.

## Interpretation and limitations

- 'Load + pipeline' excludes rendering and logging; do **not** equate target `--fps 10` with measured throughput.
- Observed wall throughput is measured for the executed headless command; when `--compare-pcl` is enabled, extra reference work reduces it.
- Ground inlier Jaccard is intersection/union of inlier indices; undefined for two empty inlier sets.
- Cluster overlap averages each custom cluster's best PCL Jaccard match; it is order-invariant, but not a one-to-one matching metric.
- The KITTI object-detection training folder is not a temporally ordered drive sequence.
- One run is not a reliable hardware benchmark: repeat warmed-up Release-build runs, control load and background work, and report variance.
