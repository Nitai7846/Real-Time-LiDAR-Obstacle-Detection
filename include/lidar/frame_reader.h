#pragma once
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar {

class FrameReader {
public:
    using Cloud = pcl::PointCloud<pcl::PointXYZI>;
    using CloudPtr = Cloud::Ptr;

    static CloudPtr loadKittiBin(const std::string& file_path);
};

}