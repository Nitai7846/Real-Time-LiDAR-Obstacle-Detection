
#include "lidar/kdtree3d.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lidar {

// -----------------------------------------
// 1. Node Constructor
// -----------------------------------------

KdTree3D::Node::Node(const Point& point, std::size_t index)
    : point(point), index(index) {}


// -----------------------------------------
// 2. Public Insert Function
// -----------------------------------------

void KdTree3D::insert(const Point& point, std::size_t index) {

    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {

        throw std::invalid_argument(
            "Point must have finite XYZ coordinates"
        );
    }

    insertRecursive(root_, point, index, 0);

    size_++;
}


// -----------------------------------------
// 3. Recursive Insertion
// -----------------------------------------

void KdTree3D::insertRecursive(
    std::unique_ptr<Node>& node,
    const Point& point,
    std::size_t index,
    std::size_t depth
) {

    // If the current position is empty,
    // create a new node here.
    if (!node) {
        node = std::make_unique<Node>(point, index);
        return;
    }

    // Select splitting axis: X, Y, Z
    std::size_t axis = depth % 3;

    float point_value;
    float node_value;

    if (axis == 0) {
        point_value = point.x;
        node_value = node->point.x;
    }
    else if (axis == 1) {
        point_value = point.y;
        node_value = node->point.y;
    }
    else {
        point_value = point.z;
        node_value = node->point.z;
    }

    // Recursively insert into left or right subtree.
    if (point_value < node_value) {

        insertRecursive(
            node->left,
            point,
            index,
            depth + 1
        );
    }
    else {

        insertRecursive(
            node->right,
            point,
            index,
            depth + 1
        );
    }
}


// -----------------------------------------
// 4. Public Radius Search
// -----------------------------------------

std::vector<std::size_t> KdTree3D::radiusSearch(
    const Point& target,
    float radius
) const {

    if (!std::isfinite(target.x) ||
        !std::isfinite(target.y) ||
        !std::isfinite(target.z)) {

        throw std::invalid_argument(
            "Target must have finite XYZ coordinates"
        );
    }

    if (!std::isfinite(radius) || radius < 0.0f) {

        throw std::invalid_argument(
            "Radius must be finite and non-negative"
        );
    }

    std::vector<std::size_t> neighbors;

    double r = static_cast<double>(radius);
    double radius_squared = r * r;

    radiusSearchRecursive(
        root_.get(),
        target,
        radius_squared,
        0,
        neighbors
    );

    // Ensure indices are unique and deterministic.
    std::sort(neighbors.begin(), neighbors.end());

    neighbors.erase(
        std::unique(neighbors.begin(), neighbors.end()),
        neighbors.end()
    );

    return neighbors;
}


// -----------------------------------------
// 5. Recursive Radius Search
// -----------------------------------------

void KdTree3D::radiusSearchRecursive(
    const Node* node,
    const Point& target,
    double radius_squared,
    std::size_t depth,
    std::vector<std::size_t>& neighbors
) {

    // Base case: no node exists.
    if (node == nullptr) {
        return;
    }

    // Calculate squared Euclidean distance.
    double dx = static_cast<double>(node->point.x) - target.x;
    double dy = static_cast<double>(node->point.y) - target.y;
    double dz = static_cast<double>(node->point.z) - target.z;

    double distance_squared =
        dx * dx +
        dy * dy +
        dz * dz;

    // If within radius, store the point index.
    if (distance_squared <= radius_squared) {
        neighbors.push_back(node->index);
    }

    // Determine splitting axis.
    std::size_t axis = depth % 3;

    double target_value;
    double node_value;

    if (axis == 0) {
        target_value = target.x;
        node_value = node->point.x;
    }
    else if (axis == 1) {
        target_value = target.y;
        node_value = node->point.y;
    }
    else {
        target_value = target.z;
        node_value = node->point.z;
    }

    // Signed distance along the splitting axis.
    double difference = target_value - node_value;

    if (difference < 0.0) {

        // Search left subtree first.
        radiusSearchRecursive(
            node->left.get(),
            target,
            radius_squared,
            depth + 1,
            neighbors
        );

        // Search right subtree if the radius
        // reaches across the splitting plane.
        if (difference * difference <= radius_squared) {

            radiusSearchRecursive(
                node->right.get(),
                target,
                radius_squared,
                depth + 1,
                neighbors
            );
        }
    }
    else {

        // Search right subtree first.
        radiusSearchRecursive(
            node->right.get(),
            target,
            radius_squared,
            depth + 1,
            neighbors
        );

        // Search left subtree if the radius
        // reaches across the splitting plane.
        if (difference * difference <= radius_squared) {

            radiusSearchRecursive(
                node->left.get(),
                target,
                radius_squared,
                depth + 1,
                neighbors
            );
        }
    }
}


// -----------------------------------------
// 6. Return Number of Stored Points
// -----------------------------------------

std::size_t KdTree3D::size() const noexcept {
    return size_;
}

} // namespace lidar
