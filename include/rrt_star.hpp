#pragma once

#include <optional>
#include <random>
#include <vector>
#include "motion_planner.hpp"
#include "point2d.hpp"
#include "search_space.hpp"

// RRT*: like RRT, but each new node picks the lowest-cost parent among its
// neighbors, and neighbors are rewired through the new node when that shortens
// their path. Keeps optimizing for `optimizationIterations` after the goal is reached.
class RRTStar : public MotionPlanner {
private:
    struct Node {
        Point2D point;
        int parent;                 // -1 = root
        double cost;                // path length from the start
        std::vector<int> children;  // needed to push cost changes down after a rewire
    };

    const SearchSpace& searchSpace;
    double goalSampleRate;
    double stepSize;
    double searchRadius;
    int optimizationIterations;
    int maxIterations;
    std::mt19937 rng;

    std::vector<Node> tree;
    Point2D goal;
    int goalIndex = -1;

    Point2D getRandomPoint();
    int nearestNode(const Point2D& point) const;
    std::optional<Point2D> steer(const Point2D& from, const Point2D& to) const;
    std::vector<int> nearbyNodes(const Point2D& point) const;
    int addNode(int iteration, const Point2D& point, int parent);
    void rewire(int iteration, int node, int newParent);
    void propagateCost(int node, double delta);
    std::vector<Point2D> retrace(int index) const;

public:
    RRTStar(const SearchSpace& searchSpace, double goalSampleRate = 0.1, double stepSize = 1.0,
            double searchRadius = 1.0, int optimizationIterations = 500,
            int maxIterations = 10000, unsigned int seed = std::random_device{}());

    PlanResult generatePath(const Pose2D& start, const Pose2D& goal) override;
    std::string name() const override { return "RRT*"; }
};
