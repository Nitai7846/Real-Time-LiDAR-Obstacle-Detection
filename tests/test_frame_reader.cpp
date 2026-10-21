
#include "lidar/frame_reader.h"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace fs = std::filesystem;

int main() {
    const fs::path test_dir =
        fs::temp_directory_path() / "lidar_reader_tests";

    fs::create_directories(test_dir);

    // Test 1: Valid KITTI binary file
    {
        fs::path path = test_dir / "valid.bin";
        std::ofstream out(path, std::ios::binary);

        float point[4] = {1.0f, 2.0f, 3.0f, 0.5f};
        out.write(reinterpret_cast<const char*>(point),
                  sizeof(point));
        out.close();

        auto cloud = lidar::FrameReader::loadKittiBin(path.string());

        if (cloud->size() != 1 ||
            cloud->points[0].x != 1.0f ||
            cloud->points[0].intensity != 0.5f) {
            throw std::runtime_error("Valid file test failed");
        }
    }

    // Test 2: Empty file
    {
        fs::path path = test_dir / "empty.bin";
        std::ofstream out(path, std::ios::binary);
        out.close();

        auto cloud = lidar::FrameReader::loadKittiBin(path.string());

        if (!cloud->empty()) {
            throw std::runtime_error("Empty file test failed");
        }
    }

    // Test 3: Invalid file size
    {
        fs::path path = test_dir / "invalid.bin";
        std::ofstream out(path, std::ios::binary);
        out.write("abc", 3);
        out.close();

        bool rejected = false;

        try {
            lidar::FrameReader::loadKittiBin(path.string());
        } catch (const std::runtime_error&) {
            rejected = true;
        }

        if (!rejected) {
            throw std::runtime_error("Invalid size test failed");
        }
    }

    // Test 4: Missing file
    {
        fs::path path = test_dir / "missing.bin";
        fs::remove(path);

        bool rejected = false;

        try {
            lidar::FrameReader::loadKittiBin(path.string());
        } catch (const std::runtime_error&) {
            rejected = true;
        }

        if (!rejected) {
            throw std::runtime_error("Missing file test failed");
        }
    }

    fs::remove_all(test_dir);

    std::cout << "All FrameReader tests passed!\n";
    return 0;
}
