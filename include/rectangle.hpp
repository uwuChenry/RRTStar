#pragma once

#include <algorithm>
#include <sstream>
#include "shape.hpp"

// Axis-aligned rectangle defined by two opposite corners.
class Rectangle : public Shape {
private:
    double minX, minY, maxX, maxY;

public:
    Rectangle(const Point2D& corner1, const Point2D& corner2)
        : minX(std::min(corner1.getX(), corner2.getX())),
          minY(std::min(corner1.getY(), corner2.getY())),
          maxX(std::max(corner1.getX(), corner2.getX())),
          maxY(std::max(corner1.getY(), corner2.getY())) {}

    double getWidth() const { return maxX - minX; }
    double getHeight() const { return maxY - minY; }

    bool pointIntersect(const Point2D& point, double inflate = 0.0) const override {
        return point.getX() >= minX - inflate && point.getX() <= maxX + inflate &&
               point.getY() >= minY - inflate && point.getY() <= maxY + inflate;
    }

    bool lineIntersect(const Line& line, double inflate = 0.0) const override {
        // A segment fully inside the rectangle crosses no edge, so check endpoints too.
        if (pointIntersect(line.getStart(), inflate) || pointIntersect(line.getEnd(), inflate)) {
            return true;
        }

        Point2D p1(minX - inflate, maxY + inflate);
        Point2D p2(maxX + inflate, maxY + inflate);
        Point2D p3(maxX + inflate, minY - inflate);
        Point2D p4(minX - inflate, minY - inflate);

        return line.intersects(Line(p1, p2)) || line.intersects(Line(p2, p3)) ||
               line.intersects(Line(p3, p4)) || line.intersects(Line(p4, p1));
    }

    std::string toSvg(double scale, double worldHeight) const override {
        std::ostringstream ss;
        ss << "<rect x=\"" << minX * scale
           << "\" y=\"" << (worldHeight - maxY) * scale
           << "\" width=\"" << getWidth() * scale
           << "\" height=\"" << getHeight() * scale << "\" fill=\"black\"/>";
        return ss.str();
    }
};
