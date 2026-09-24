#pragma once

#include <optional>
#include <random>
#include <vector>
#include "point2d.hpp"
#include "search_space.hpp"

// Tree node. Parents are stored as indices into the node list (-1 = root).
struct Node {
    Point2D point;
    int parent;
};

struct PlanResult {
    bool success = false;
    int iterations = 0;          // -1 if start/goal is in collision
    double timeSeconds = 0.0;
    std::vector<Point2D> path;   // start -> goal, empty on failure

    double pathLength() const {
        double len = 0.0;
        for (size_t i = 1; i < path.size(); ++i) len += path[i - 1].distanceTo(path[i]);
        return len;
    }
};

class RRT {
private:
    const SearchSpace& searchSpace;
    double goalSampleRate;
    double stepSize;
    int maxIterations;
    std::mt19937 rng;

    std::vector<Node> tree;
    Point2D goal;

    Point2D getRandomPoint();
    int nearestNode(const Point2D& point) const;
    std::optional<Point2D> steer(const Point2D& from, const Point2D& to) const;
    std::vector<Point2D> retrace(int goalIndex) const;

public:
    RRT(const SearchSpace& searchSpace, double goalSampleRate = 0.1, double stepSize = 1.0,
        int maxIterations = 10000, unsigned int seed = std::random_device{}());

    PlanResult generatePath(const Pose2D& start, const Pose2D& goal);

    // The tree from the most recent call to generatePath (for visualization).
    const std::vector<Node>& getTree() const { return tree; }
};
