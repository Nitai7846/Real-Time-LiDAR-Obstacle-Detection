# Milestone 11 — Profiling and PCL references (C++17)

This bundle is an overlay of **complete replacement files** for the working repository. All prior modules not included here remain untouched. It provides:

- Per-stage `std::chrono::steady_clock` timings (preprocessing, RANSAC, ground split, clustering, boxes, total pipeline).
- Load-time and total load+pipeline timings (excluding visualization and logging).
- Optional CSV logging of every processed frame (`--csv`). Existing output files are not overwritten.
- **Opt-in, headless-only** side-by-side comparisons against `pcl::SACSegmentation` and `pcl::EuclideanClusterExtraction` (`--compare-pcl`).
- Aggregated mean, median, p95 and max, throughput estimates, detection disagreement statistics, and an optional Markdown report (`--report`).
- Three new complete test suites, preserving ten previous CTest entries (expected **13 total**).

## Included files

| Path | Role |
|---|---|
| `include/lidar/pipeline.h`, `src/pipeline.cpp` | Final stage timing implementation from the previous tested version |
| `include/lidar/pcl_comparison.h`, `src/pcl_comparison.cpp` | Optional reference benchmarks and overlap metrics |
| `include/lidar/metrics_logger.h` | CSV writer and scalar-only per-frame measurements |
| `include/lidar/performance_report.h`, `src/performance_report.cpp` | Percentiles, throughput, console and Markdown summaries |
| `apps/main.cpp` | **Complete** application retaining existing headless/GUI/single-frame behavior |
| `tests/test_pipeline_timing.cpp` | Previous nine timing tests (unchanged) |
| `tests/test_pcl_comparison.cpp`, `tests/test_metrics_logger.cpp`, `tests/test_performance_report.cpp` | All new tests together |
| `CMakeLists.txt` | Includes `segmentation`, `search`, two new sources and three new CTest targets |

**Not included and not modified:** point reader, sequence discovery and AppleDouble filtering, preprocessing, custom RANSAC, ground splitter, KD-Tree, custom clustering, bounding boxes, or PCL visualization.

## Safe application instructions

1. Back up the repository first: `cp -R Lidar_Detection Lidar_Detection_backup_m10` from its parent folder, or use your usual git checkpoint.
2. Copy the *contents* of this bundle onto corresponding project-relative paths, do not delete the other existing files.
3. Use the existing `build` directory. `cmake --build build -j 4` normally auto-regenerates CMake when needed. This milestone adds the two existing PCL components `segmentation` and `search`, so a normal Ninja reconfigure is expected **once**.
4. If Ninja loops on CMake timestamp regeneration (your external-drive copying issue), stop with Ctrl+C, run `touch CMakeLists.txt` from the project root, then retry the build. Do not remove the build directory or reinstall PCL.
5. Run `ctest --test-dir build --output-on-failure` (expected 13 suites, with the new tests).

## Commands

Normal, unchanged GUI replay:

```bash
./build/lidar_demo "/Volumes/One Touch/data_object_velodyne/training/velodyne" --fps 10 --max-frames 100
```

Headless pipeline-only metrics:

```bash
./build/lidar_demo "/Volumes/One Touch/data_object_velodyne/training/velodyne" \
  --headless --max-frames 100 --csv run_metrics.csv --report run_report.md
```

Headless complete benchmark and references:

```bash
./build/lidar_demo "/Volumes/One Touch/data_object_velodyne/training/velodyne" \
  --headless --max-frames 200 --compare-pcl \
  --csv custom_vs_pcl.csv --report custom_vs_pcl.md
```

**Use fresh filenames** on repeated runs. Both CSV and Markdown output paths refuse to overwrite existing files. For a credible comparison, build Release (`-DCMAKE_BUILD_TYPE=Release`) and repeat the experiment after warmup (this is a future one-time configure only if your current build isn't already Release).

## Fairness and limitations

- Both RANSAC implementations use the same **filtered points** and numeric max iterations, distance threshold and normal-tilt tolerance. PCL's `SACMODEL_PERPENDICULAR_PLANE` is Z-axis constrained. Random sampling, implicit constraint details and optional `expected_ground_z` filtering need not be identical; report inlier Jaccard and plane angle, not equality assumptions.
- Both clustering implementations use the same non-ground cloud **derived from the custom ground plane**, same tolerance and min/max cluster sizes. This avoids confounding clustering differences with upstream segmentation differences. Both construct their tree internally when `extract` is run.
- Custom/PCL benchmark timings are **separate re-runs**, not the pipeline stage times. Benchmark costs are excluded from `load_plus_pipeline_ms` but included in measured headless wall throughput.
- Cluster overlap is the mean of each custom cluster's highest Jaccard IoU with any PCL cluster (order invariant, not a one-to-one assignment).
- `--fps 10` is a requested visualization cadence, **not proof of 10 FPS processing**. A 10 Hz sensor stream provides 100 ms per frame; use p95 pipeline time and load+pipeline time when assessing sustained readiness.
- KITTI object-detection frame indices do not prove temporal adjacency.
