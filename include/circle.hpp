#pragma once

#include <sstream>
#include "shape.hpp"

class Circle : public Shape {
private:
    Point2D center;
    double radius;

public:
    Circle(const Point2D& center, double radius) : center(center), radius(radius) {}

    bool lineIntersect(const Line& line, double inflate = 0.0) const override {
        Point2D closest = line.closestPointTo(center);
        return center.distanceTo(closest) <= radius + inflate;
    }

    bool pointIntersect(const Point2D& point, double inflate = 0.0) const override {
        return point.distanceTo(center) <= radius + inflate;
    }

    std::string toSvg(double scale, double worldHeight) const override {
        std::ostringstream ss;
        ss << "<circle cx=\"" << center.getX() * scale
           << "\" cy=\"" << (worldHeight - center.getY()) * scale
           << "\" r=\"" << radius * scale << "\" fill=\"black\"/>";
        return ss.str();
    }
};
