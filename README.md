# LiDAR Obstacle Detection — C++17 / PCL

**Custom geometry-based 3D obstacle segmentation on KITTI Velodyne scans**, implemented in C++17 with the Point Cloud Library (PCL), and demonstrated on Apple Silicon (M2). Includes deterministic 3D RANSAC, a custom 3D KD-tree, iterative Euclidean clustering, 3D axis-aligned bounding boxes, interactive replay, and per-stage performance analysis.

> **What this project does:** identifies *spatial point-cloud clusters*; it does **not** assign semantic classes (car/pedestrian), track persistent object identities, operate on a live sensor, or prove sustained real-time performance. The KITTI object-detection training frames used here are not necessarily a time-continuous driving sequence.

## Demo — LiDAR Obstacle Detection Playback

PASTE_GITHUB_VIDEO_URL_HERE

*Sequential KITTI Velodyne frame playback with ground segmentation, Euclidean obstacle clustering, and 3D bounding-box visualization. Dataset frames are not necessarily temporally consecutive.*

**[Architecture](docs/ARCHITECTURE.md)** · **[Measured Benchmarks](docs/BENCHMARKS.md)** · **[Reproduction Guide](docs/REPRODUCIBILITY.md)**

## Pipeline

```text
Sorted KITTI .bin file paths
    ↓
FrameReader (XYZ + reflectance, pcl::PointXYZI)
    ↓
Finite-point filtering
    ↓
VoxelGrid Downsampling (0.2 m)
    ↓
3D CropBox ROI
    ↓
Custom Deterministic 3D RANSAC
    ↓
Ground / Non-Ground Separation
    ↓
Custom 3D KD-Tree Radius Search
    ↓
Iterative BFS Euclidean Clustering
    ↓
Axis-Aligned 3D Bounding Boxes
    ↓
PCLVisualizer / Headless Statistics
    ↓
Optional CSV Logging + PCL Comparisons
```

**Implementation:** Reusable `lidar_core` library; `apps/main.cpp` provides the `lidar_demo` CLI. Thirteen CTest executables cover loading, geometry, clustering, frame sequencing, pipeline timing, and benchmarks.

For module-level details, see [Architecture](docs/ARCHITECTURE.md).

## Results — 200 KITTI Object-Detection Training Frames

Measured in the supplied Milestone 11 Mac benchmark (one run):

| Metric | Result |
|---|---:|
| Median processing pipeline (excluding disk load/rendering) | **17.624 ms** |
| p95 processing pipeline | **34.963 ms** |
| Median disk load + pipeline | **49.579 ms** |
| p95 disk load + pipeline | **67.395 ms** |
| Observed headless wall throughput, **with optional PCL comparisons** | **13.420 FPS** |
| Custom/PCL cluster-count agreement | **200/200 frames** |
| Mean-best cluster Jaccard, median | **1.000** |

The normal processing pipeline and optional reference benchmarks are timed separately.

The measured headless wall throughput includes extra comparison work, CSV logging, and console output. It should **not** be quoted as GUI FPS or live-LiDAR FPS.

Agreement with PCL is not ground-truth semantic detection accuracy.

See [Benchmark Methodology, Full Metrics and Caveats](docs/BENCHMARKS.md).

## Requirements

- macOS on Apple Silicon (tested by project author on M2), C++17 with Apple Clang
- Homebrew PCL **1.15.1** (reported environment), CMake **3.20+**, Ninja
- KITTI **object detection** Velodyne binary frames downloaded separately
- Optional VS Code; GUI display for `PCLVisualizer`

Dependencies are **not** bundled. Install only if missing.

See [Reproducibility](docs/REPRODUCIBILITY.md) for environment recording and a fresh-machine procedure.

## Build and Test

```bash
# From repository root; requires PCL installed and discoverable

cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$(brew --prefix pcl);$(brew --prefix)"

cmake --build build -j 4

ctest --test-dir build --output-on-failure

./build/lidar_demo --help
```

The prior run on the author's Mac reported **13/13 passing**.

Tests have **not** been rebuilt in this documentation package's environment.

See [Validation Checklist](VALIDATION_CHECKLIST.md).

## Run

The input may be a single valid KITTI `.bin` file or a folder of such files.

Quote paths containing spaces.

### Interactive Multi-Frame Playback

```bash
./build/lidar_demo "/path/to/kitti/training/velodyne" \
  --fps 10 --max-frames 100
```

The `--fps` argument controls the target GUI playback cadence, **not measured throughput**.

### Frame-by-Frame Manual Playback

```bash
./build/lidar_demo "/path/to/kitti/training/velodyne" \
  --step --max-frames 100
```

Start paused and use the keyboard controls to step through frames.

### Headless Benchmark with PCL Comparisons

```bash
./build/lidar_demo "/path/to/kitti/training/velodyne" \
  --headless --max-frames 200 --compare-pcl \
  --csv run_$(date +%Y%m%d_%H%M%S).csv \
  --report run_$(date +%Y%m%d_%H%M%S).md
```

Use new output filenames to avoid overwrite rejection. For a single shared timestamp across both output files, see [REPRODUCIBILITY.md](docs/REPRODUCIBILITY.md).

`--compare-pcl` requires `--headless`.

The program's `--help` is the authoritative options list.

### CLI Options

| CLI Flag | Function |
|---|---|
| `--headless` | Process without PCL viewer |
| `--fps N` | Target GUI replay cadence; default 10 |
| `--step` | Start GUI paused |
| `--max-frames N` | Limit frame count |
| `--csv PATH` | Write a new metrics CSV |
| `--compare-pcl` | Headless-only reference benchmark |
| `--report PATH` | Write a new Markdown report |
| `--help` | Display usage |

### Interactive Viewer Controls

| Key | Action |
|---|---|
| `Space` | Pause / Resume |
| `F` | Next frame when paused |
| `B` | Previous frame when paused |
| `Q` | Quit visualization |
| `G` | Toggle ground visualization |
| `N` / `P` | Next / Previous cluster |
| `A` | Show all clusters |
| `0–9` | Select cluster |

Some keys act only during sequence replay or when clusters exist.

## Data and Default Parameters

### KITTI Point-Cloud Format

KITTI frames use repeating **4 × 32-bit floats**:

```text
(x, y, z, reflectance)
```

The data is loaded into `pcl::PointXYZI`.

### Preprocessing

| Parameter | Value |
|---|---|
| Voxel leaf size | 0.2 m |
| ROI X | [-15, 35] m |
| ROI Y | [-10, 10] m |
| ROI Z | [-3, 4] m |

### Ground Segmentation — Custom RANSAC

| Parameter | Value |
|---|---|
| Maximum iterations | 100 |
| Plane distance tolerance | 0.2 m |
| Random seed | 42 |
| Maximum ground-plane tilt | 25° |

### Custom KD-Tree Euclidean Clustering

| Parameter | Value |
|---|---|
| Cluster distance tolerance | 0.5 m |
| Minimum cluster size | 20 points |
| Maximum cluster size | 10,000 points |

Consecutive filenames in the **KITTI object-detection training** directory should **not** be interpreted as a recorded temporal driving trajectory.

See [Dataset Instructions](docs/DATASET.md) and [Known Limitations](docs/LIMITATIONS.md).

## Repository Layout

```text
lidar-obstacle-detection/
│
├── CMakeLists.txt
├── README.md
├── .gitignore
├── VALIDATION_CHECKLIST.md
│
├── apps/
│   └── main.cpp
│
├── include/lidar/
│   └── C++ public headers
│
├── src/
│   └── C++ implementations
│
├── tests/
│   └── 13 CTest test sources
│
├── config/
│   └── Reserved configuration directory
│
└── docs/
    ├── ARCHITECTURE.md
    ├── BENCHMARKS.md
    ├── DATASET.md
    ├── REPRODUCIBILITY.md
    ├── LIMITATIONS.md
    │
    ├── benchmarks/
    │   ├── Supplied benchmark CSV
    │   └── Generated Markdown report
    │
    └── assets/
        ├── clustered-point-cloud.png
        └── sequence-playback.mp4
```

## Notes and Scope

Processing is geometric, not learned.

PCL reference algorithms are used only for opt-in comparison; core RANSAC, KD-tree neighbor queries, Euclidean clustering, and AABBs are custom C++ implementations.

The archived experiment is provided for transparency, not to claim superiority: PCL is faster in both measured reference comparisons.

No project license has been selected here. **Do not assume permission to redistribute code or KITTI assets.** Add a license only after deciding how to release your work.
