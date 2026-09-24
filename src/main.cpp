#include <iostream>
#include <memory>
#include "circle.hpp"
#include "html_animation.hpp"
#include "rectangle.hpp"
#include "rrt.hpp"
#include "search_space.hpp"
#include "svg_writer.hpp"

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
    PlanResult result = rrt.generatePath(start, goal);

    if (result.iterations == -1) {
        std::cout << "Start or goal is inside an obstacle." << std::endl;
        return 1;
    }
    if (!result.success) {
        std::cout << "No path found after " << result.iterations << " iterations." << std::endl;
    } else {
        std::cout << "Path found in " << result.iterations << " iterations ("
                  << result.timeSeconds * 1000.0 << " ms), length "
                  << result.pathLength() << ", " << result.path.size() << " waypoints:" << std::endl;
        for (const auto& p : result.path) {
            std::cout << "  [" << p.getX() << ", " << p.getY() << "]" << std::endl;
        }
    }

    if (writeSvg("rrt.svg", space, rrt.getTree(), result.path, start, goal)) {
        std::cout << "Wrote rrt.svg" << std::endl;
    }
    if (writeAnimation("rrt_animation.html", space, rrt.getTree(), result.path, start, goal,
                       result.iterations)) {
        std::cout << "Wrote rrt_animation.html (open in a browser)" << std::endl;
    }
    return result.success ? 0 : 1;
}
