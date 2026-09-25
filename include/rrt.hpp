#pragma once

#include <optional>
#include <random>
#include <vector>
#include "motion_planner.hpp"
#include "point2d.hpp"
#include "search_space.hpp"

class RRT : public MotionPlanner {
private:
    // Tree node. Parents are stored as indices into the node list (-1 = root).
    struct Node {
        Point2D point;
        int parent;
    };

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

    PlanResult generatePath(const Pose2D& start, const Pose2D& goal) override;
    std::string name() const override { return "RRT"; }
};
