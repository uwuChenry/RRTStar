#include "rrt_star.hpp"

#include <algorithm>
#include <chrono>
#include <limits>

namespace {
// Rewire only on a real improvement, so floating-point noise doesn't cause churn.
constexpr double kCostEpsilon = 1e-9;
}

RRTStar::RRTStar(const SearchSpace& searchSpace, double goalSampleRate, double stepSize,
                 double searchRadius, int optimizationIterations, int maxIterations,
                 unsigned int seed)
    : searchSpace(searchSpace),
      goalSampleRate(goalSampleRate),
      stepSize(stepSize),
      searchRadius(searchRadius),
      optimizationIterations(optimizationIterations),
      maxIterations(maxIterations),
      rng(seed) {}

PlanResult RRTStar::generatePath(const Pose2D& startPose, const Pose2D& goalPose) {
    auto tStart = std::chrono::steady_clock::now();
    auto elapsed = [&]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - tStart).count();
    };

    PlanResult result;
    goal = goalPose.point;
    goalIndex = -1;
    tree.clear();
    resetHistory();
    addNode(0, startPose.point, -1);

    if (searchSpace.checkPointCollision(startPose.point) || searchSpace.checkPointCollision(goal)) {
        result.iterations = -1;
        return result;
    }

    int iteration = 0;
    int optimizationCount = 0;
    while (iteration < maxIterations && optimizationCount < optimizationIterations) {
        ++iteration;
        if (goalIndex != -1) ++optimizationCount;

        Point2D sample = getRandomPoint();
        int nearest = nearestNode(sample);
        std::optional<Point2D> newPoint = steer(tree[nearest].point, sample);

        if (!newPoint || searchSpace.checkLineCollision(Line(tree[nearest].point, *newPoint))) {
            continue;
        }

        // Choose parent: the neighbor giving the cheapest collision-free path to newPoint.
        std::vector<int> neighbors = nearbyNodes(*newPoint);
        int bestParent = nearest;
        double bestCost = tree[nearest].cost + tree[nearest].point.distanceTo(*newPoint);
        for (int n : neighbors) {
            if (n == nearest || n == goalIndex) continue;
            double c = tree[n].cost + tree[n].point.distanceTo(*newPoint);
            if (c < bestCost - kCostEpsilon &&
                !searchSpace.checkLineCollision(Line(tree[n].point, *newPoint))) {
                bestParent = n;
                bestCost = c;
            }
        }

        int newIndex = addNode(iteration, *newPoint, bestParent);

        // Rewire: route neighbors through the new node if that makes them cheaper.
        for (int n : neighbors) {
            if (n == bestParent || tree[n].parent == -1) continue;
            double c = bestCost + tree[n].point.distanceTo(*newPoint);
            if (c < tree[n].cost - kCostEpsilon &&
                !searchSpace.checkLineCollision(Line(*newPoint, tree[n].point))) {
                rewire(iteration, n, newIndex);
            }
        }

        // Connect to the goal, or improve the existing goal connection.
        double toGoal = newPoint->distanceTo(goal);
        if (toGoal <= stepSize) {
            double goalCost = bestCost + toGoal;
            bool better = goalIndex == -1 || goalCost < tree[goalIndex].cost - kCostEpsilon;
            if (better && !searchSpace.checkLineCollision(Line(*newPoint, goal))) {
                if (goalIndex == -1) {
                    goalIndex = addNode(iteration, goal, newIndex);
                    history.goalIndex = goalIndex;
                    result.firstSolutionIteration = iteration;
                    result.firstSolutionCost = tree[goalIndex].cost;
                } else {
                    rewire(iteration, goalIndex, newIndex);
                }
            }
        }
    }

    history.totalIterations = iteration;
    result.iterations = iteration;
    if (goalIndex != -1) {
        result.success = true;
        result.path = retrace(goalIndex);
    }
    result.timeSeconds = elapsed();
    return result;
}

Point2D RRTStar::getRandomPoint() {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    if (dist(rng) < goalSampleRate) return goal;
    return searchSpace.randomPoint(rng);
}

int RRTStar::nearestNode(const Point2D& point) const {
    int best = 0;
    double bestDist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < tree.size(); ++i) {
        if (static_cast<int>(i) == goalIndex) continue;  // never grow the tree out of the goal
        double d = tree[i].point.distanceSquaredTo(point);
        if (d < bestDist) {
            bestDist = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

// Move from `from` toward `to`, at most stepSize.
std::optional<Point2D> RRTStar::steer(const Point2D& from, const Point2D& to) const {
    if (from == to) return std::nullopt;
    if (from.distanceTo(to) < stepSize) return to;
    return from + (to - from).normalized() * stepSize;
}

std::vector<int> RRTStar::nearbyNodes(const Point2D& point) const {
    std::vector<int> result;
    double r2 = searchRadius * searchRadius;
    for (size_t i = 0; i < tree.size(); ++i) {
        if (tree[i].point.distanceSquaredTo(point) <= r2) result.push_back(static_cast<int>(i));
    }
    return result;
}

int RRTStar::addNode(int iteration, const Point2D& point, int parent) {
    double cost = parent == -1 ? 0.0 : tree[parent].cost + tree[parent].point.distanceTo(point);
    tree.push_back({point, parent, cost, {}});
    int index = static_cast<int>(tree.size()) - 1;
    if (parent != -1) tree[parent].children.push_back(index);
    recordNewNode(iteration, point, parent);
    return index;
}

void RRTStar::rewire(int iteration, int node, int newParent) {
    auto& oldChildren = tree[tree[node].parent].children;
    oldChildren.erase(std::find(oldChildren.begin(), oldChildren.end(), node));

    tree[node].parent = newParent;
    tree[newParent].children.push_back(node);

    double newCost = tree[newParent].cost + tree[newParent].point.distanceTo(tree[node].point);
    propagateCost(node, newCost - tree[node].cost);
    recordRewire(iteration, node, newParent);
}

// Shift the cost of `node` and its whole subtree by `delta`.
void RRTStar::propagateCost(int node, double delta) {
    std::vector<int> stack = {node};
    while (!stack.empty()) {
        int n = stack.back();
        stack.pop_back();
        tree[n].cost += delta;
        for (int c : tree[n].children) stack.push_back(c);
    }
}

std::vector<Point2D> RRTStar::retrace(int index) const {
    std::vector<Point2D> path;
    for (int i = index; i != -1; i = tree[i].parent) {
        path.push_back(tree[i].point);
    }
    std::reverse(path.begin(), path.end());
    return path;
}
