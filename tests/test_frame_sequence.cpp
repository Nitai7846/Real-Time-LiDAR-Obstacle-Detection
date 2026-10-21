#include "lidar/frame_sequence.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using lidar::FrameSequence;

namespace {

class TemporaryDirectory {
public:
    TemporaryDirectory() {
        const auto stamp = std::chrono::steady_clock::now()
                               .time_since_epoch().count();
        for (int attempt = 0; attempt < 100; ++attempt) {
            path_ = fs::temp_directory_path() /
                    ("lidar_frames_test_" + std::to_string(stamp) + "_" +
                     std::to_string(attempt));
            if (fs::create_directory(path_)) {
                return;
            }
        }
        throw std::runtime_error("Failed to create a temporary test directory");
    }

    ~TemporaryDirectory() {
        std::error_code error;
        fs::remove_all(path_, error);
    }

    TemporaryDirectory(const TemporaryDirectory&) = delete;
    TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;

    const fs::path& path() const { return path_; }

    void createFile(const fs::path& relative_path) const {
        const auto full_path = path_ / relative_path;
        fs::create_directories(full_path.parent_path());
        std::ofstream output(full_path, std::ios::binary);
        if (!output) {
            throw std::runtime_error("Failed to create test file: " +
                                     full_path.string());
        }
        // Empty placeholders are sufficient: discovery never reads file data.
    }

private:
    fs::path path_;
};

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNames(const FrameSequence::Frames& frames,
                  const std::vector<std::string>& expected) {
    require(frames.size() == expected.size(), "Unexpected number of frames");
    for (std::size_t i = 0; i < expected.size(); ++i) {
        require(frames[i].filename().string() == expected[i],
                "Unexpected frame filename at index " + std::to_string(i));
    }
}

template <typename Exception, typename Function>
void requireThrows(Function action, const std::string& message) {
    try {
        action();
    } catch (const Exception&) {
        return;
    }
    throw std::runtime_error(message);
}

void testSingleBinFile() {
    TemporaryDirectory temp;
    temp.createFile("000000.bin");
    const auto file = temp.path() / "000000.bin";
    const auto frames = FrameSequence::discover(file);
    require(frames.size() == 1 && frames.front() == file,
            "Single .bin input must return exactly its own path");
}

void testSortedFilenameOrder() {
    TemporaryDirectory temp;
    temp.createFile("000010.bin");
    temp.createFile("000002.bin");
    temp.createFile("000000.bin");
    temp.createFile("000001.bin");

    requireNames(FrameSequence::discover(temp.path()),
                 {"000000.bin", "000001.bin", "000002.bin", "000010.bin"});
}

void testIgnoreOtherExtensions() {
    TemporaryDirectory temp;
    temp.createFile("000001.bin");
    temp.createFile("000002.pcd");
    temp.createFile("notes.txt");
    temp.createFile("000003.BIN");
    temp.createFile("000004.bin.bak");

    requireNames(FrameSequence::discover(temp.path()), {"000001.bin"});
}

void testEmptyDirectory() {
    TemporaryDirectory temp;
    require(FrameSequence::discover(temp.path()).empty(),
            "Empty directory should yield no frames");
}

void testDirectoryWithNoBinFiles() {
    TemporaryDirectory temp;
    temp.createFile("readme.txt");
    temp.createFile("000000.pcd");
    require(FrameSequence::discover(temp.path()).empty(),
            "Directory without .bin files should yield no frames");
}

void testNonRecursiveDiscovery() {
    TemporaryDirectory temp;
    temp.createFile("000001.bin");
    temp.createFile("nested/000002.bin");
    requireNames(FrameSequence::discover(temp.path()), {"000001.bin"});
}

void testMissingInputThrows() {
    TemporaryDirectory temp;
    requireThrows<std::invalid_argument>(
        [&] { FrameSequence::discover(temp.path() / "missing.bin"); },
        "Nonexistent path should throw std::invalid_argument");
}

void testUnsupportedSingleFileThrows() {
    TemporaryDirectory temp;
    temp.createFile("frame.pcd");
    requireThrows<std::invalid_argument>(
        [&] { FrameSequence::discover(temp.path() / "frame.pcd"); },
        "Single non-.bin file should throw std::invalid_argument");
}

void testDirectoryNamedBinIsStillDirectory() {
    TemporaryDirectory temp;
    temp.createFile("frames.bin/000000.bin");
    requireNames(FrameSequence::discover(temp.path() / "frames.bin"),
                 {"000000.bin"});
}

void testPathsWithSpaces() {
    TemporaryDirectory temp;
    temp.createFile("velodyne scans/000001.bin");
    temp.createFile("velodyne scans/000000.bin");
    const auto frames = FrameSequence::discover(temp.path() / "velodyne scans");
    requireNames(frames, {"000000.bin", "000001.bin"});
    require(frames.front().parent_path() == temp.path() / "velodyne scans",
            "Returned path must preserve original parent directory");
}

void testSkipAppleDoubleMetadata() {
    TemporaryDirectory temp;
    temp.createFile("000000.bin");
    temp.createFile("._000000.bin");
    temp.createFile("000001.bin");
    temp.createFile("._000001.bin");
    temp.createFile("._orphan.bin");

    // AppleDouble files must not appear in the discovered sequence or
    // consume slots when --max-frames is applied by the playback program.
    requireNames(FrameSequence::discover(temp.path()),
                 {"000000.bin", "000001.bin"});
}

void testOnlyAppleDoubleFiles() {
    TemporaryDirectory temp;
    temp.createFile("._000000.bin");
    temp.createFile("._000001.bin");
    require(FrameSequence::discover(temp.path()).empty(),
            "A directory of AppleDouble files must yield no LiDAR frames");
}

void testAppleDoubleSingleFileRejected() {
    TemporaryDirectory temp;
    temp.createFile("._000000.bin");
    requireThrows<std::invalid_argument>(
        [&] { FrameSequence::discover(temp.path() / "._000000.bin"); },
        "A single AppleDouble file must not be accepted as a KITTI scan");
}

void testRepeatedDiscoveryIsDeterministic() {
    TemporaryDirectory temp;
    temp.createFile("000003.bin");
    temp.createFile("000001.bin");
    require(FrameSequence::discover(temp.path()) ==
                FrameSequence::discover(temp.path()),
            "Repeated discovery should return the same paths in the same order");
}

}  // namespace

int main() {
    struct TestCase {
        const char* name;
        void (*run)();
    };

    const std::vector<TestCase> tests = {
        {"single .bin file", testSingleBinFile},
        {"sorted filename order", testSortedFilenameOrder},
        {"ignore other extensions", testIgnoreOtherExtensions},
        {"empty directory", testEmptyDirectory},
        {"directory without .bin files", testDirectoryWithNoBinFiles},
        {"non-recursive discovery", testNonRecursiveDiscovery},
        {"missing input error", testMissingInputThrows},
        {"unsupported single file error", testUnsupportedSingleFileThrows},
        {"directory ending in .bin", testDirectoryNamedBinIsStillDirectory},
        {"paths containing spaces", testPathsWithSpaces},
        {"skip AppleDouble metadata", testSkipAppleDoubleMetadata},
        {"AppleDouble-only directory", testOnlyAppleDoubleFiles},
        {"reject single AppleDouble file", testAppleDoubleSingleFileRejected},
        {"deterministic discovery", testRepeatedDiscoveryIsDeterministic},
    };

    std::size_t passed = 0;
    for (const auto& test : tests) {
        try {
            test.run();
            ++passed;
            std::cout << "[PASS] " << test.name << '\n';
        } catch (const std::exception& e) {
            std::cerr << "[FAIL] " << test.name << ": " << e.what() << '\n';
        }
    }

    std::cout << "\n" << passed << " / " << tests.size()
              << " FrameSequence tests passed.\n";
    return passed == tests.size() ? 0 : 1;
}
