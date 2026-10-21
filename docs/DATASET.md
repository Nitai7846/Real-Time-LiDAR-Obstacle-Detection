# KITTI data: acquisition and usage

This project used the **KITTI Object Detection** training Velodyne directory, not a KITTI raw synchronized-drive sequence. The original input path on the author's Mac was `/Volumes/One Touch/data_object_velodyne/training/velodyne/`.

1. Visit the [official KITTI datasets website](https://www.cvlibs.net/datasets/kitti/) and find the **Object Detection Evaluation** dataset's **Velodyne point clouds** training download.
2. Read and comply with the dataset's current license, citation, and redistribution terms. Data are **not included** in this repository; do not upload downloaded scans unless rights permit.
3. Extract locally and note the directory containing `000000.bin`, `000001.bin`, and so on. Pass that directory to `lidar_demo` as a quoted argument.
4. For a small smoke test, supply a genuine single `.bin` frame instead of a full directory.

**Frame format:** an array of little-endian 32-bit floating-point quadruples `(x, y, z, intensity/reflectance)`, 16 bytes per point; loaded into PCL `PointXYZI`. The reader checks for file-size consistency. Coordinate convention for KITTI Velodyne is typically x forward, y left, z up; verify for alternative datasets.

**Important:** numbered examples in the KITTI **object detection** training split are not guaranteed to be consecutive captures of a drive. Alphabetical sorting permits a demonstration of changing scenes, but does not mean the camera/sensor moved smoothly between these frames. The demo is described as **dataset-frame replay**, not a tracking sequence.

**macOS metadata:** downloaded/extracted volumes can contain `._000000.bin` AppleDouble metadata. `FrameSequence` intentionally ignores names beginning with `._` when discovering `.bin` files. Keep this behavior intact.

**Privacy and repository hygiene:** retain downloads locally; `.gitignore` excludes `*.bin`, `*.pcd`, and `data/` content except the placeholder. Benchmark metrics in `docs/benchmarks/` are small, textual, and included as recorded observations rather than raw KITTI point clouds.
