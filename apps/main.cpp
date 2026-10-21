#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "lidar/frame_reader.h"
#include "lidar/frame_sequence.h"
#include "lidar/metrics_logger.h"
#include "lidar/pcl_comparison.h"
#include "lidar/performance_report.h"
#include "lidar/pipeline.h"
#include "lidar/visualization.h"

namespace {

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using Frames = lidar::FrameSequence::Frames;

struct Options {
    fs::path input;
    bool headless = false;
    bool step = false;
    double fps = 10.0;
    std::optional<std::size_t> max_frames;
    std::optional<fs::path> csv_path;
    std::optional<fs::path> report_path;
    bool compare_pcl = false;
};

void printUsage() {
    std::cout
        << "Usage: lidar_demo <file.bin|directory> [options]\n"
        << "Options:\n"
        << "  --headless        Process frames without opening PCLVisualizer\n"
        << "  --fps N           Target playback rate, default 10 (GUI only)\n"
        << "  --step            Start GUI playback paused (first frame displayed)\n"
        << "  --max-frames N    Process at most N sorted input frames\n"
        << "  --csv PATH        Write per-frame pipeline metrics to a new CSV file\n"
        << "  --compare-pcl     Compare custom algorithms vs PCL (headless only)\n"
        << "  --report PATH     Write Markdown statistics to a new report file\n"
        << "  --help            Show this help\n\n"
        << "GUI playback: Space pause/resume, F next frame, B previous frame,\n"
        << "              Q quit; G/N/P/A/0-9 retain cluster controls.\n";
}

Options parseOptions(int argc, char** argv) {
    if (argc < 2) {
        throw std::invalid_argument("Missing KITTI file or directory path");
    }

    Options options;
    options.input = argv[1];

    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "--headless") {
            options.headless = true;
        } else if (arg == "--step") {
            options.step = true;
        } else if (arg == "--fps") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--fps requires a numeric value");
            }
            const std::string value = argv[++i];
            std::size_t used = 0;
            try {
                options.fps = std::stod(value, &used);
            } catch (const std::exception&) {
                throw std::invalid_argument("Invalid --fps value: " + value);
            }
            if (used != value.size() || !std::isfinite(options.fps) ||
                options.fps <= 0.0 || options.fps > 1000.0) {
                throw std::invalid_argument("--fps must be greater than 0 and at most 1000");
            }
        } else if (arg == "--csv") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--csv requires an output file path");
            }
            const std::string value = argv[++i];
            if (value.empty() || value.rfind("--", 0) == 0) {
                throw std::invalid_argument("--csv requires a valid output file path");
            }
            options.csv_path = fs::path(value);
        } else if (arg == "--compare-pcl") {
            options.compare_pcl = true;
        } else if (arg == "--report") {
            if (i + 1 >= argc || std::string(argv[i + 1]).rfind("--", 0) == 0) {
                throw std::invalid_argument("--report requires an output path");
            }
            options.report_path = fs::path(argv[++i]);
        } else if (arg == "--max-frames") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--max-frames requires an integer");
            }
            const std::string value = argv[++i];
            if (value.empty() || value.front() == '-') {
                throw std::invalid_argument("--max-frames must be a positive integer");
            }
            std::size_t used = 0;
            unsigned long long count = 0;
            try {
                count = std::stoull(value, &used);
            } catch (const std::exception&) {
                throw std::invalid_argument("Invalid --max-frames value: " + value);
            }
            if (used != value.size() || count == 0 ||
                count > static_cast<unsigned long long>(
                    std::numeric_limits<std::size_t>::max())) {
                throw std::invalid_argument("--max-frames must be a positive integer");
            }
            options.max_frames = static_cast<std::size_t>(count);
        } else {
            throw std::invalid_argument("Unknown option: " + arg);
        }
    }

    if (options.headless && options.step) {
        throw std::invalid_argument("--step requires visualization (cannot combine with --headless)");
    }
    if (options.compare_pcl && !options.headless) {
        throw std::invalid_argument("--compare-pcl requires --headless to preserve GUI playback speed");
    }
    if (options.report_path && options.report_path->empty()) {
        throw std::invalid_argument("--report path cannot be empty");
    }
    if (options.report_path && fs::exists(*options.report_path)) {
        throw std::runtime_error("Refusing to overwrite existing report: " + options.report_path->string());
    }
    if (options.csv_path && options.report_path &&
        fs::absolute(*options.csv_path).lexically_normal() ==
        fs::absolute(*options.report_path).lexically_normal()) {
        throw std::invalid_argument("CSV and report output paths must be different");
    }
    return options;
}

struct ProcessedFrame {
    std::size_t raw_points = 0;
    double load_ms = 0.0;
    lidar::PipelineResult result;
    std::optional<lidar::PclComparison> comparison;
};

// Loads and processes exactly one KITTI frame. No frame history is cached.
ProcessedFrame processFrame(const fs::path& path, const lidar::Pipeline& pipeline,
                            const lidar::PipelineConfig& config, bool compare_pcl) {
    try {
        const auto load_start = Clock::now();
        auto cloud = lidar::FrameReader::loadKittiBin(path.string());
        const double load_ms = std::chrono::duration<double, std::milli>(
            Clock::now() - load_start).count();
        if (!cloud) {
            throw std::runtime_error("FrameReader returned a null cloud");
        }

        ProcessedFrame processed;
        processed.raw_points = cloud->size();
        processed.load_ms = load_ms;
        processed.result = pipeline.process(cloud);
        if (compare_pcl) {
            processed.comparison = lidar::PclComparator::compare(processed.result, config);
        }
        return processed;
    } catch (const std::exception& e) {
        throw std::runtime_error("Unable to process '" + path.string() + "': " + e.what());
    }
}

void printFrameSummary(const ProcessedFrame& frame,
                       const fs::path& path,
                       std::size_t index,
                       std::size_t count,
                       bool detailed) {
    const auto& result = frame.result;
    const auto ground_points = result.separated.ground->size();
    const auto nonground_points = result.separated.nonground->size();
    const auto valid_boxes = std::count_if(result.boxes.begin(), result.boxes.end(),
                                          [](const auto& box) { return box.has_value(); });

    std::cout << "[" << (index + 1) << "/" << count << "] " << path.filename().string()
              << " | raw=" << frame.raw_points
              << " filtered=" << result.filtered->size()
              << " ground=" << ground_points
              << " nonground=" << nonground_points
              << " clusters=" << result.clusters.size()
              << " boxes=" << valid_boxes;
    if (!result.ransac.found_plane) {
        std::cout << " | no ground plane";
    }
    std::cout << '\n';

    if (!detailed) {
        return;
    }

    if (result.ransac.found_plane) {
        const auto& p = result.ransac.plane;
        std::cout << "Ground plane: " << p.a << "x + " << p.b << "y + "
                  << p.c << "z + " << p.d << " = 0\n";
    }
    for (std::size_t i = 0; i < result.clusters.size(); ++i) {
        std::cout << "Cluster " << i << ": " << result.clusters[i].size() << " points";
        if (i < result.boxes.size() && result.boxes[i].has_value()) {
            const auto& box = *result.boxes[i];
            std::cout << " | Min (" << box.min_x << ", " << box.min_y << ", "
                      << box.min_z << ") Max (" << box.max_x << ", "
                      << box.max_y << ", " << box.max_z << ")"
                      << " Center (" << box.center_x << ", " << box.center_y << ", "
                      << box.center_z << ") Dimensions (" << box.length << ", "
                      << box.width << ", " << box.height << ")";
        }
        std::cout << '\n';
    }
}

void showFrame(lidar::Visualizer& visualizer, const ProcessedFrame& frame) {
    const auto& result = frame.result;
    if (result.ransac.found_plane) {
        visualizer.showGroundAndClusters(result.separated.ground,
                                         result.separated.nonground,
                                         result.clusters, result.boxes);
    } else {
        // Retain the original single-frame fallback for no detected plane.
        visualizer.showCloud(result.filtered);
    }
}

void recordMetrics(const ProcessedFrame& processed, const fs::path& path,
                   std::size_t index, lidar::MetricsLogger* logger,
                   lidar::PerformanceReport& report) {
    auto row = lidar::makeFrameMetrics(index, path, processed.raw_points,
                                      processed.load_ms, processed.result,
                                      processed.comparison);
    if (logger) logger->write(row);
    report.add(row);
}

void runHeadless(const Frames& frames,
                 const lidar::Pipeline& pipeline,
                 const lidar::PipelineConfig& config,
                 bool compare_pcl,
                 lidar::MetricsLogger* logger,
                 lidar::PerformanceReport& report) {
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const auto processed = processFrame(frames[i], pipeline, config, compare_pcl);
        printFrameSummary(processed, frames[i], i, frames.size(), frames.size() == 1);
        recordMetrics(processed, frames[i], i, logger, report);
    }
    std::cout << "Completed " << frames.size() << " frame(s) headlessly.\n";
}

void runVisualPlayback(const Frames& frames,
                       const Options& options,
                       const lidar::Pipeline& pipeline,
                       bool single_file_input,
                       lidar::MetricsLogger* logger,
                       lidar::PerformanceReport& report,
                       const lidar::PipelineConfig& config) {
    lidar::Visualizer visualizer;

    // Preserve the original blocking single-file viewer and its key bindings.
    if (single_file_input && !options.step) {
        const auto processed = processFrame(frames.front(), pipeline, config, false);
        printFrameSummary(processed, frames.front(), 0, 1, true);
        recordMetrics(processed, frames.front(), 0, logger, report);
        showFrame(visualizer, processed);
        visualizer.run();
        return;
    }

    // All playback state is local to this function, on the GUI/main thread.
    std::size_t current_index = 0;
    bool paused = options.step;
    bool quit = false;
    std::optional<std::size_t> requested_index;
    const auto interval = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / options.fps));
    auto next_deadline = Clock::now() + interval;

    visualizer.setPlaybackHandler([&](lidar::Visualizer::PlaybackCommand cmd) {
        using Command = lidar::Visualizer::PlaybackCommand;
        switch (cmd) {
            case Command::TogglePause:
                if (paused) {
                    if (current_index + 1 >= frames.size()) {
                        std::cout << "At the final frame. Press B to step backward.\n";
                        return;
                    }
                    paused = false;
                    next_deadline = Clock::now() + interval;
                } else {
                    paused = true;
                }
                std::cout << (paused ? "Paused\n" : "Playing\n");
                break;
            case Command::StepForward:
                if (!paused) {
                    std::cout << "Pause playback (Space) before stepping.\n";
                } else if (current_index + 1 < frames.size()) {
                    requested_index = current_index + 1;
                } else {
                    std::cout << "Already at last frame.\n";
                }
                break;
            case Command::StepBackward:
                if (!paused) {
                    std::cout << "Pause playback (Space) before stepping.\n";
                } else if (current_index > 0) {
                    requested_index = current_index - 1;
                } else {
                    std::cout << "Already at first frame.\n";
                }
                break;
            case Command::Quit:
                quit = true;
                break;
        }
    });

    std::cout << "\n--- Playback Controls ---\n"
              << "Space: Pause/resume\n"
              << "F: Next frame (when paused)\n"
              << "B: Previous frame (when paused)\n"
              << "Q: Quit playback\n";

    const auto first = processFrame(frames.front(), pipeline, config, false);
    printFrameSummary(first, frames.front(), 0, frames.size(), false);
    recordMetrics(first, frames.front(), 0, logger, report);
    showFrame(visualizer, first);
    next_deadline = Clock::now() + interval;
    if (paused) {
        std::cout << "Paused at first frame (--step). Press F to advance.\n";
    }

    while (!visualizer.wasStopped() && !quit) {
        visualizer.spinOnce(1);
        if (quit || visualizer.wasStopped()) {
            break;
        }

        std::optional<std::size_t> index_to_show;
        if (requested_index) {
            index_to_show = *requested_index;
            requested_index.reset();
        } else if (!paused && current_index + 1 < frames.size() &&
                   Clock::now() >= next_deadline) {
            index_to_show = current_index + 1;
        }

        if (!index_to_show) {
            continue;
        }

        // The previous ProcessedFrame is destroyed before the next iteration;
        // PCLVisualizer clears previous actors when this scene is installed.
        const auto processed = processFrame(frames[*index_to_show], pipeline, config, false);
        showFrame(visualizer, processed);
        current_index = *index_to_show;
        printFrameSummary(processed, frames[current_index],
                          current_index, frames.size(), false);
        recordMetrics(processed, frames[current_index], current_index, logger, report);

        // Respect the requested cadence when processing is faster than the
        // deadline; do not claim the hardware achieved the target FPS.
        next_deadline = std::max(next_deadline + interval, Clock::now());
        if (!paused && current_index + 1 >= frames.size()) {
            paused = true;
            std::cout << "Reached final frame. Press B to review or Q to exit.\n";
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        printUsage();
        return 0;
    }

    try {
        const Options options = parseOptions(argc, argv);
        const bool single_file_input = fs::is_regular_file(options.input);
        auto frames = lidar::FrameSequence::discover(options.input);
        if (options.max_frames && frames.size() > *options.max_frames) {
            frames.resize(*options.max_frames);
        }
        if (frames.empty()) {
            std::cout << "No .bin files found in " << options.input.string() << ".\n";
            return 0;
        }

        const lidar::PipelineConfig config; // Existing default configurations.
        lidar::Pipeline pipeline(config);
        std::optional<lidar::MetricsLogger> logger;
        if (options.csv_path) {
            logger.emplace(*options.csv_path);  // Fail before processing if output is invalid.
            std::cout << "CSV metrics: " << options.csv_path->string() << '\n';
        }
        lidar::MetricsLogger* logger_ptr = logger ? &*logger : nullptr;

        std::cout << "LiDAR Obstacle Detection: " << frames.size()
                  << " frame(s) discovered.\n";

        std::string invocation;
        for (int i = 0; i < argc; ++i) {
            if (i) invocation += ' ';
            invocation += '"';
            for (char c : std::string(argv[i])) {
                if (c == '"' || c == '\\') invocation += '\\';
                invocation += c;
            }
            invocation += '"';
        }
        lidar::PerformanceReport report;
        const auto run_start = Clock::now();
        if (options.headless) {
            runHeadless(frames, pipeline, config, options.compare_pcl, logger_ptr, report);
        } else {
            runVisualPlayback(frames, options, pipeline, single_file_input,
                              logger_ptr, report, config);
        }
        const double observed_ms = options.headless
            ? std::chrono::duration<double, std::milli>(Clock::now() - run_start).count()
            : 0.0; // GUI wall time includes intentional pauses and is not throughput.
        if (options.headless || options.report_path) report.print(std::cout, observed_ms);
        if (options.report_path) {
            std::ofstream file;
            file.exceptions(std::ios::badbit | std::ios::failbit);
            file.open(*options.report_path, std::ios::out | std::ios::trunc);
            report.writeMarkdown(file, options.input.string(), observed_ms,
                                 config, invocation);
            file.flush();
            std::cout << "Saved Markdown report: " << options.report_path->string() << '\n';
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n\n";
        printUsage();
        return 1;
    }
}
