#pragma once

#include <string>
#include <vector>
#include "point2d.hpp"

// One change to the tree: `node` gets `parent` during `iteration`.
// The first event for a node adds it; later events for it are rewires (RRT*).
struct TreeEvent {
    int iteration;
    int node;
    int parent;  // -1 = root
};

// Everything needed to replay how the tree grew (used by the animation/SVG writers).
struct TreeHistory {
    std::vector<Point2D> points;    // position of every node, by index
    std::vector<TreeEvent> events;  // in the order they happened
    int goalIndex = -1;             // node index of the goal, -1 if never reached
    int totalIterations = 0;

    std::vector<int> finalParents() const {
        std::vector<int> parents(points.size(), -1);
        for (const auto& e : events) parents[e.node] = e.parent;
        return parents;
    }
};

struct PlanResult {
    bool success = false;
    int iterations = 0;               // -1 if start/goal is in collision
    double timeSeconds = 0.0;
    std::vector<Point2D> path;        // start -> goal, empty on failure
    int firstSolutionIteration = -1;  // when the goal was first reached
    double firstSolutionCost = 0.0;   // path length at that moment

    double pathLength() const {
        double len = 0.0;
        for (size_t i = 1; i < path.size(); ++i) len += path[i - 1].distanceTo(path[i]);
        return len;
    }
};

class MotionPlanner {
public:
    virtual ~MotionPlanner() = default;

    virtual PlanResult generatePath(const Pose2D& start, const Pose2D& goal) = 0;
    virtual std::string name() const = 0;

    // History of the most recent call to generatePath.
    const TreeHistory& getHistory() const { return history; }

protected:
    TreeHistory history;

    void resetHistory() { history = TreeHistory(); }

    int recordNewNode(int iteration, const Point2D& point, int parent) {
        history.points.push_back(point);
        int index = static_cast<int>(history.points.size()) - 1;
        history.events.push_back({iteration, index, parent});
        return index;
    }

    void recordRewire(int iteration, int node, int newParent) {
        history.events.push_back({iteration, node, newParent});
    }
};
