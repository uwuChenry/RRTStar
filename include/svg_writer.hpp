#pragma once

#include <fstream>
#include <string>
#include <vector>
#include "point2d.hpp"
#include "rrt.hpp"
#include "search_space.hpp"

// Writes the field, obstacles, RRT tree and final path to an SVG file
// (open it in any browser). Replaces the matplotlib drawing in the Python version.
inline bool writeSvg(const std::string& filename, const SearchSpace& space,
                     const std::vector<Node>& tree, const std::vector<Point2D>& path,
                     const Pose2D& start, const Pose2D& goal, double scale = 50.0) {
    std::ofstream out(filename);
    if (!out) return false;

    double h = space.getHeight();
    auto sx = [&](const Point2D& p) { return p.getX() * scale; };
    auto sy = [&](const Point2D& p) { return (h - p.getY()) * scale; };  // flip y so +y is up

    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << space.getWidth() * scale
        << "\" height=\"" << h * scale << "\">\n";
    out << "<rect width=\"100%\" height=\"100%\" fill=\"white\" stroke=\"black\"/>\n";

    for (const auto& obstacle : space.getObstacles()) {
        out << obstacle->toSvg(scale, h) << "\n";
    }

    for (const auto& node : tree) {
        if (node.parent < 0) continue;
        const Point2D& p = tree[node.parent].point;
        out << "<line x1=\"" << sx(node.point) << "\" y1=\"" << sy(node.point)
            << "\" x2=\"" << sx(p) << "\" y2=\"" << sy(p)
            << "\" stroke=\"red\" stroke-width=\"1\"/>\n";
    }

    if (!path.empty()) {
        out << "<polyline fill=\"none\" stroke=\"green\" stroke-width=\"4\" points=\"";
        for (const auto& p : path) out << sx(p) << "," << sy(p) << " ";
        out << "\"/>\n";
    }

    out << "<circle cx=\"" << sx(start.point) << "\" cy=\"" << sy(start.point)
        << "\" r=\"8\" fill=\"blue\"/>\n";
    out << "<circle cx=\"" << sx(goal.point) << "\" cy=\"" << sy(goal.point)
        << "\" r=\"8\" fill=\"gold\" stroke=\"black\"/>\n";
    out << "</svg>\n";
    return true;
}
