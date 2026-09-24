#pragma once

#include <string>
#include "line.hpp"
#include "point2d.hpp"

// Base class for obstacles. `inflate` grows the obstacle by that distance
// (used to account for the robot's radius).
class Shape {
public:
    virtual ~Shape() = default;

    virtual bool lineIntersect(const Line& line, double inflate = 0.0) const = 0;
    virtual bool pointIntersect(const Point2D& point, double inflate = 0.0) const = 0;

    // SVG element for this shape. `toSvg` maps world coordinates to SVG coordinates.
    virtual std::string toSvg(double scale, double worldHeight) const = 0;
};
