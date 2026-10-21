
#pragma once

#include <filesystem>
#include <vector>

namespace lidar {

class FrameSequence {
public:
    using Path = std::filesystem::path;
    using Frames = std::vector<Path>;

    // Accepts a single KITTI .bin file or a directory of .bin files.
    // Returns files in lexicographic filename order.
    // An empty directory returns an empty vector.
    // Invalid paths or unsupported file types cause an exception.
    static Frames discover(const Path& input);
};

}  // namespace lidar
