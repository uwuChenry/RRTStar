#pragma once

#include <memory>
#include <random>
#include <vector>
#include "line.hpp"
#include "shape.hpp"

// The 2D world: a width x height field with obstacles.
// `robotRadius` inflates every obstacle and shrinks the field boundary.
class SearchSpace {
private:
    double width;
    double height;
    double robotRadius;
    std::vector<std::shared_ptr<Shape>> obstacles;
    std::vector<Line> edges;

public:
    SearchSpace(double width, double height,
                std::vector<std::shared_ptr<Shape>> obstacles, double robotRadius = 0.0)
        : width(width), height(height), robotRadius(robotRadius), obstacles(std::move(obstacles)) {
        Point2D p1(robotRadius, robotRadius);
        Point2D p2(robotRadius, height - robotRadius);
        Point2D p3(width - robotRadius, height - robotRadius);
        Point2D p4(width - robotRadius, robotRadius);
        edges = {Line(p1, p2), Line(p2, p3), Line(p3, p4), Line(p4, p1)};
    }

    double getWidth() const { return width; }
    double getHeight() const { return height; }
    const std::vector<std::shared_ptr<Shape>>& getObstacles() const { return obstacles; }

    Point2D randomPoint(std::mt19937& rng) const {
        std::uniform_real_distribution<double> xDist(0.0, width);
        std::uniform_real_distribution<double> yDist(0.0, height);
        return Point2D(xDist(rng), yDist(rng));
    }

    bool isInBounds(const Point2D& point) const {
        return point.getX() >= robotRadius && point.getX() <= width - robotRadius &&
               point.getY() >= robotRadius && point.getY() <= height - robotRadius;
    }

    // True if the point is inside an obstacle or outside the field.
    bool checkPointCollision(const Point2D& point) const {
        if (!isInBounds(point)) return true;
        for (const auto& obstacle : obstacles) {
            if (obstacle->pointIntersect(point, robotRadius)) return true;
        }
        return false;
    }

    // True if the segment hits an obstacle or crosses the field boundary.
    bool checkLineCollision(const Line& line) const {
        for (const auto& obstacle : obstacles) {
            if (obstacle->lineIntersect(line, robotRadius)) return true;
        }
        for (const auto& edge : edges) {
            if (edge.intersects(line)) return true;
        }
        return false;
    }
};
