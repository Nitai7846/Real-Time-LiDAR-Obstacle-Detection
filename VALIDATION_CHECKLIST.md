# Milestone 12 — one local validation procedure

**Scope:** documentation/media only. No `apps/`, `src/`, `include/`, `tests/`, or `CMakeLists.txt` source files were intentionally modified. The historical benchmark fixtures now live under `docs/benchmarks/` to keep the repository tidy; references in README/docs point there. This ZIP is a **full project snapshot** excluding `build/`, macOS metadata and the KITTI dataset.

## Single verification pass on the Mac

- [ ] Extract the final ZIP into a **new folder** and verify `README.md`, `.gitignore`, `docs/` and project source files exist. Keep the preexisting working repository intact until verification passes.
- [ ] Confirm screenshots and demo appear at `docs/assets/clustered-point-cloud.png` and `docs/assets/sequence-playback.mp4`. Open the MP4 locally; ensure it is an actual moving replay (and not necessarily a continuous driving sequence).
- [ ] Inspect `docs/benchmarks/milestone11_benchmark.csv` and `.md`: 200 rows, accurate results; ensure no raw KITTI `.bin` were packaged.
- [ ] From the extracted project root on Mac, run:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$(brew --prefix pcl);$(brew --prefix)"
cmake --build build -j 4
ctest --test-dir build --output-on-failure
./build/lidar_demo --help
```

- [ ] Confirm **13/13** suites pass locally. This documentation package has **not** executed native PCL/VTK build/tests in the container.
- [ ] Run `./build/lidar_demo "/Volumes/One Touch/data_object_velodyne/training/velodyne" --fps 10 --max-frames 20` and verify the viewer/scene switches frames; `Space`, `F`, `B`, `Q` function.
- [ ] Run a small **headless** test with a genuine `.bin` file. If desired rerun the 200-frame comparison with **new** CSV/report names following `docs/REPRODUCIBILITY.md`.
- [ ] Check `git status`/`git diff --stat`; confirm `build/`, data files, macOS metadata, old binaries, recordings outside `docs/assets/`, and generated metrics outside `docs/benchmarks/` are not staged. `git check-ignore -v build/lidar_demo` should identify the ignore rule after a local build.
- [ ] Review the README rendering and Mermaid diagram on GitHub. Verify image/video links work and GitHub accepts the MP4 size.
- [ ] Choose a software license explicitly **before** claiming the repo is open-source. Review KITTI usage terms; do not commit downloaded datasets.

**Acceptance criterion:** documented project builds and passes 13 tests on the owner's Mac; actual playback and headless modes remain operational; benchmark results and their limitations are presented faithfully; GitHub shows a screenshot and linked video. These checks must be performed locally before announcing a fully reproducible release.
