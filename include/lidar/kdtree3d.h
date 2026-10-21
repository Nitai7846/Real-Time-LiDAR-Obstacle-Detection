
#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <pcl/point_types.h>

namespace lidar {

class KdTree3D {

public:

    using Point = pcl::PointXYZI;

    KdTree3D() = default;

    void insert(const Point& point, std::size_t index);

    std::vector<std::size_t> radiusSearch(
        const Point& target,
        float radius
    ) const;

    std::size_t size() const noexcept;

private:

    struct Node {

        Point point;
        std::size_t index;

        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;

        Node(const Point& point, std::size_t index);
    };

    std::unique_ptr<Node> root_;

    std::size_t size_ = 0;

    static void insertRecursive(
        std::unique_ptr<Node>& node,
        const Point& point,
        std::size_t index,
        std::size_t depth
    );

    static void radiusSearchRecursive(
        const Node* node,
        const Point& target,
        double radius_squared,
        std::size_t depth,
        std::vector<std::size_t>& neighbors
    );
};

} // namespace lidar
