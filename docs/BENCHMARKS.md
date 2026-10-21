# Milestone 11 — measured evaluation

**Data provenance:** the supplied `docs/benchmarks/milestone11_benchmark.csv` (200 rows; frames `000000.bin` to `000199.bin`) and `docs/benchmarks/milestone11_report.md`. All figures below come from that one completed run reported on the author's M2 Mac, not a rerun performed for Milestone 12.

## Reproduction context

Command recorded in original report:

```bash
./build/lidar_demo "/Volumes/One Touch/data_object_velodyne/training/velodyne" \
  --headless --max-frames 200 --compare-pcl \
  --csv milestone11_benchmark.csv --report milestone11_report.md
```

Configuration: voxel **0.2 m**; ROI `[-15,35] × [-10,10] × [-3,4]` m; custom RANSAC **100** iterations, **0.2 m** threshold, seed **42**, max tilt **25°**; cluster tolerance **0.5 m**, **20–10,000** points. Compiler flags/build type, exact macOS/compiler/CMake versions and run-to-run variance were *not recorded* alongside the supplied data. The reproducibility guide tells you how to capture these for new runs.

## Per-stage times (ms)

| Stage | Mean | Median | p95 | Maximum |
|---|---:|---:|---:|---:|
| Disk load | 32.520 | 32.575 | 37.903 | 39.497 |
| Preprocessing | 3.986 | 3.925 | 4.785 | 6.852 |
| Custom pipeline RANSAC | 2.145 | 2.130 | 2.797 | 3.888 |
| Ground separation | 0.078 | 0.075 | 0.123 | 0.163 |
| Custom pipeline clustering | 12.453 | 11.869 | 28.292 | 59.598 |
| Axis-aligned boxes | 0.013 | 0.013 | 0.027 | 0.052 |
| **Perception pipeline total** | **18.675** | **17.624** | **34.963** | **66.792** |
| **Disk load + pipeline** | **51.196** | **49.579** | **67.395** | **95.112** |

The report calculates p95 by linear interpolation at index `(N - 1) × 0.95`. `Disk load + pipeline` excludes visualization and extra reporting overhead. The mean-based *estimated* disk+pipeline throughput is **19.533 FPS**; measured wall throughput for the full **headless comparison invocation** is **13.420 FPS**. These are different measurements and cannot be exchanged. p95 disk+pipeline falls below a **100-ms 10-FPS budget**, but that does not verify sustained 10-FPS live-sensor operation or interactive rendering.

## Reference algorithm comparisons

| Algorithm | Custom median | PCL median | Ratio PCL faster |
|---|---:|---:|---:|
| Ground RANSAC | 2.089 ms | 0.230 ms | ~9.1× |
| Euclidean clustering | 11.517 ms | 7.529 ms | ~1.53× |

The reference timings are separately rerun algorithm timings, not stages added to normal pipeline latency. Plane fits share the filtered point input and similar thresholds, **not** identical model/sampling internals. PCL uses `SACMODEL_PERPENDICULAR_PLANE`, constrained along Z. Both clustering methods consume the **same custom-RANSAC-derived non-ground points** to make the clusterer comparison meaningful.

- Cluster counts agree in **200/200** frames.
- Per-custom-cluster mean of best PCL Jaccard overlaps: **1.000 median and mean**; this is **not** a one-to-one cluster assignment or semantic ground truth score.
- Plane ground inlier Jaccard: **0.909 median**, **0.868 mean**.
- Plane normal angular difference: **0.553° median**, **0.757° mean**.
- Exact ground inlier counts agree in **0/200** frames, reflecting differing fitted plane hypotheses without independently establishing correctness.

**Bottleneck:** disk load averages 32.520 ms, while custom clustering averages 12.453 ms (~66.7% of the 18.675-ms mean perception pipeline). The custom code is not claimed to be faster than PCL.

## Interpretation and caveats

This experiment measures **geometric grouping** and reference overlap, not obstacle recall/precision/mAP or semantic object detection. KITTI detection sample indices do not establish temporal adjacency. Replay of arbitrary numbered examples is not multi-frame tracking. Results reflect the particular hardware and input set, and only one run. Repeat with a warmed-up Release build, consistent machine load, environment details and varied repetitions before making cross-system performance claims. No live Velodyne sensor or ROS input is implemented.
