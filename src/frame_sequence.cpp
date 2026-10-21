#include "lidar/frame_sequence.h"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace lidar {

namespace {

// On macOS external drives, AppleDouble sidecar files may have names such as
// "._000000.bin". They contain filesystem metadata, not LiDAR point data.
bool isKittiBinFile(const std::filesystem::path& path) {
    const std::string filename = path.filename().string();
    return path.extension() == ".bin" && filename.rfind("._", 0) != 0;
}

}  // namespace

FrameSequence::Frames FrameSequence::discover(const Path& input) {
    namespace fs = std::filesystem;

    if (!fs::exists(input)) {
        throw std::invalid_argument(
            "FrameSequence: input path does not exist: " + input.string()
        );
    }

    // Preserve the existing single-frame workflow, but never accept AppleDouble metadata.
    if (fs::is_regular_file(input)) {
        if (!isKittiBinFile(input)) {
            throw std::invalid_argument(
                "FrameSequence: expected a KITTI .bin file (not ._ metadata): " +
                input.string()
            );
        }
        return {input};
    }

    if (!fs::is_directory(input)) {
        throw std::invalid_argument(
            "FrameSequence: input must be a regular file or directory: " +
            input.string()
        );
    }

    Frames frames;

    // Non-recursive: each frame must be directly inside the given directory.
    for (const auto& entry : fs::directory_iterator(input)) {
        if (entry.is_regular_file() && isKittiBinFile(entry.path())) {
            frames.push_back(entry.path());
        }
    }

    // KITTI's zero-padded names sort chronologically by filename.
    std::sort(frames.begin(), frames.end(),
              [](const Path& lhs, const Path& rhs) {
                  return lhs.filename().string() < rhs.filename().string();
              });

    return frames;
}

}  // namespace lidar
