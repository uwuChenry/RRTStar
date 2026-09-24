#include "rrt.hpp"

#include <algorithm>
#include <chrono>
#include <limits>

RRT::RRT(const SearchSpace& searchSpace, double goalSampleRate, double stepSize,
         int maxIterations, unsigned int seed)
    : searchSpace(searchSpace),
      goalSampleRate(goalSampleRate),
      stepSize(stepSize),
      maxIterations(maxIterations),
      rng(seed) {}

PlanResult RRT::generatePath(const Pose2D& startPose, const Pose2D& goalPose) {
    auto tStart = std::chrono::steady_clock::now();
    auto elapsed = [&]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - tStart).count();
    };

    PlanResult result;
    goal = goalPose.point;
    tree.clear();
    tree.push_back({startPose.point, -1, 0});

    if (searchSpace.checkPointCollision(startPose.point) || searchSpace.checkPointCollision(goal)) {
        result.iterations = -1;
        return result;
    }

    for (int i = 0; i < maxIterations; ++i) {
        Point2D sample = getRandomPoint();
        int nearest = nearestNode(sample);
        std::optional<Point2D> newPoint = steer(tree[nearest].point, sample);

        if (!newPoint || searchSpace.checkLineCollision(Line(tree[nearest].point, *newPoint))) {
            continue;
        }

        tree.push_back({*newPoint, nearest, i + 1});
        int newIndex = static_cast<int>(tree.size()) - 1;

        // Close enough to the goal with a clear line of sight: connect and finish.
        if (newPoint->distanceTo(goal) <= stepSize &&
            !searchSpace.checkLineCollision(Line(*newPoint, goal))) {
            tree.push_back({goal, newIndex, i + 1});
            result.success = true;
            result.iterations = i + 1;
            result.path = retrace(static_cast<int>(tree.size()) - 1);
            result.timeSeconds = elapsed();
            return result;
        }
    }

    result.iterations = maxIterations;
    result.timeSeconds = elapsed();
    return result;
}

Point2D RRT::getRandomPoint() {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    if (dist(rng) < goalSampleRate) return goal;
    return searchSpace.randomPoint(rng);
}

int RRT::nearestNode(const Point2D& point) const {
    int best = 0;
    double bestDist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < tree.size(); ++i) {
        double d = tree[i].point.distanceSquaredTo(point);
        if (d < bestDist) {
            bestDist = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

// Move from `from` toward `to`, at most stepSize.
std::optional<Point2D> RRT::steer(const Point2D& from, const Point2D& to) const {
    if (from == to) return std::nullopt;
    if (from.distanceTo(to) < stepSize) return to;
    return from + (to - from).normalized() * stepSize;
}

std::vector<Point2D> RRT::retrace(int goalIndex) const {
    std::vector<Point2D> path;
    for (int i = goalIndex; i != -1; i = tree[i].parent) {
        path.push_back(tree[i].point);
    }
    std::reverse(path.begin(), path.end());
    return path;
}
