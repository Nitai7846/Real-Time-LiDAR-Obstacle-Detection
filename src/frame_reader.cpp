#include "lidar/frame_reader.h"

#include <array>
#include <fstream>
#include <stdexcept>

namespace lidar {

FrameReader::CloudPtr FrameReader::loadKittiBin(
    const std::string& file_path
) {
    // Open the binary file and move to its end
    std::ifstream file(file_path, std::ios::binary | std::ios::ate);

    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + file_path);
    }

    // Get the file size
    std::streampos file_size = file.tellg();

    if (file_size < 0 || file_size % 16 != 0) {
        throw std::runtime_error("Invalid KITTI file size");
    }

    // Return to the beginning of the file
    file.seekg(0, std::ios::beg);

    // Create a point cloud
    CloudPtr cloud(new Cloud);

    // Reserve space for all points
    cloud->reserve(static_cast<std::size_t>(file_size / 16));

    // Each point contains X, Y, Z, intensity
    std::array<float, 4> data;

    while (file.read(
        reinterpret_cast<char*>(data.data()),
        sizeof(data)
    )) {
        pcl::PointXYZI point;

        point.x = data[0];
        point.y = data[1];
        point.z = data[2];
        point.intensity = data[3];

        cloud->push_back(point);
    }

    if (!file.eof() || file.gcount() != 0) {
        throw std::runtime_error("Error reading KITTI file");
    }

    return cloud;
}

} // namespace lidar