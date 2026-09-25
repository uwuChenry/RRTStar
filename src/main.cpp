#include <iostream>
#include <memory>
#include <string>
#include "circle.hpp"
#include "html_animation.hpp"
#include "rectangle.hpp"
#include "rrt.hpp"
#include "rrt_star.hpp"
#include "search_space.hpp"
#include "svg_writer.hpp"

// Runs a planner, prints the result, and writes <filePrefix>.svg and <filePrefix>_animation.html.
bool runPlanner(MotionPlanner& planner, const SearchSpace& space, const Pose2D& start,
                const Pose2D& goal, const std::string& filePrefix) {
    PlanResult result = planner.generatePath(start, goal);
    std::cout << "=== " << planner.name() << " ===" << std::endl;

    if (result.iterations == -1) {
        std::cout << "Start or goal is inside an obstacle." << std::endl;
        return false;
    }
    if (!result.success) {
        std::cout << "No path found after " << result.iterations << " iterations." << std::endl;
    } else {
        std::cout << "First path at iteration " << result.firstSolutionIteration
                  << ", length " << result.firstSolutionCost << std::endl;
        std::cout << "Final path after " << result.iterations << " iterations ("
                  << result.timeSeconds * 1000.0 << " ms), length " << result.pathLength()
                  << ", " << result.path.size() << " waypoints" << std::endl;
    }

    writeSvg(filePrefix + ".svg", space, planner.getHistory(), result.path, start, goal);
    writeAnimation(filePrefix + "_animation.html", space, planner.getHistory(), start, goal,
                   planner.name());
    std::cout << "Wrote " << filePrefix << ".svg and " << filePrefix << "_animation.html"
              << std::endl << std::endl;
    return result.success;
}

int main() {
    // 12 x 12 field with obstacles (same setup as the Python main.py, plus a rectangle).
    std::vector<std::shared_ptr<Shape>> obstacles = {
        std::make_shared<Circle>(Point2D(6.0, 6.0), 3.9),
        std::make_shared<Rectangle>(Point2D(1.0, 9.0), Point2D(4.0, 10.0)),
    };
    SearchSpace space(12.0, 12.0, obstacles, 0.0);

    Pose2D start(2.0, 6.0, 0.0);
    Pose2D goal(10.0, 6.0, 0.0);

    // goalSampleRate, stepSize, maxIterations
    RRT rrt(space, 0.1, 0.5, 2000);
    // goalSampleRate, stepSize, searchRadius, optimizationIterations, maxIterations
    RRTStar rrtStar(space, 0.1, 0.5, 2.0, 1000, 5000);

    bool ok = runPlanner(rrt, space, start, goal, "rrt");
    ok = runPlanner(rrtStar, space, start, goal, "rrt_star") && ok;
    return ok ? 0 : 1;
}
