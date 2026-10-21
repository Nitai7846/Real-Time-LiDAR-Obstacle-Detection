# Reproduction — native Apple Silicon / PCL

## Known environment

The author's successful Milestone 11 run used an Apple Silicon M2 Mac, C++17, Apple Clang, VS Code, Homebrew PCL **1.15.1**, CMake/Ninja and KITTI object-detection Velodyne files. Exact macOS/CMake/Ninja/compiler/build-type/optimization **versions or flags used during the archived benchmark are not completely evidenced** in the CSV/report. Record these for any new published run.

## Fresh development machine

```bash
uname -m                  # arm64 expected on Apple Silicon
xcode-select -p           # Command Line Tools must be available
brew --version
brew --prefix pcl          # if unavailable, install: brew install pcl
cmake --version            # if unavailable, install: brew install cmake ninja
ninja --version
clang++ --version
```

From the project root:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$(brew --prefix pcl);$(brew --prefix)"
cmake --build build -j 4
ctest --test-dir build --output-on-failure
./build/lidar_demo --help
```

If working with the already-configured original project on the author's Mac, **do not remove `build/` needlessly**; instead use `cmake --build build -j 4`, then CTest. The README/documentation package makes no CMake/source modifications. If Ninja repeatedly reconfigures due to an anomalous future modification time on `CMakeLists.txt`, stop the build and correct the timestamp (`touch CMakeLists.txt`), then retry. No dependency reinstall is required to handle that problem.

## Verify input and interactive playback

```bash
# On the original author's Mac (replace on a different machine):
DATA="/Volumes/One Touch/data_object_velodyne/training/velodyne"
./build/lidar_demo "$DATA/000000.bin" --headless
./build/lidar_demo "$DATA" --fps 10 --max-frames 100
./build/lidar_demo "$DATA" --step --max-frames 20
```

The viewer supports pause/step/back and must not accumulate old scene shapes. KITTI object training samples are **not** continuous driving frames. `--fps` specifies target GUI pacing only.

## Re-run benchmark with new output names

```bash
DATA="/Volumes/One Touch/data_object_velodyne/training/velodyne"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
mkdir -p local_results
./build/lidar_demo "$DATA" \
  --headless --max-frames 200 --compare-pcl \
  --csv "local_results/${RUN_ID}_metrics.csv" \
  --report "local_results/${RUN_ID}_summary.md"
```

**Output paths must be new**; this executable intentionally avoids silently overwriting existing CSV/report files. `local_results/` is ignored by Git. A test without comparison can omit `--compare-pcl` but will not produce direct PCL reference metrics; report which mode was used.

Record exact environment alongside new measurements:

```bash
{
  date -u
  sw_vers
  uname -m
  clang++ --version
  cmake --version
  ninja --version
  brew list --versions pcl
  grep -E 'CMAKE_BUILD_TYPE:|CMAKE_CXX_FLAGS_RELEASE:' build/CMakeCache.txt
  git rev-parse HEAD 2>/dev/null || true
} > "local_results/${RUN_ID}_environment.txt"
```

Report cold/warm state, machine background load, and multiple run distributions if comparing with another system. The project author previously reported **13 passing CTest suites**, not a verification performed on this system. The archived `docs/benchmarks/` results remain unchanged and provide historical reference numbers.
