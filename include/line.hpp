#pragma once

#include <algorithm>
#include "point2d.hpp"

// Line segment from start to end.
class Line {
private:
    Point2D start;
    Point2D end;

public:
    Line(const Point2D& start, const Point2D& end) : start(start), end(end) {}

    const Point2D& getStart() const { return start; }
    const Point2D& getEnd() const { return end; }

    double length() const { return start.distanceTo(end); }

    // t in [0, 1]: 0 -> start, 1 -> end
    Point2D interpolate(double t) const {
        t = std::clamp(t, 0.0, 1.0);
        return start + (end - start) * t;
    }

    // Closest point on the segment to pt (projection clamped to the segment).
    Point2D closestPointTo(const Point2D& pt) const {
        Point2D d = end - start;
        double len2 = d.dot(d);
        if (len2 == 0.0) return start;
        double t = (pt - start).dot(d) / len2;
        return interpolate(t);
    }

    // True if the two segments cross or touch.
    bool intersects(const Line& other) const {
        double x1 = start.getX(), y1 = start.getY();
        double x2 = end.getX(), y2 = end.getY();
        double x3 = other.start.getX(), y3 = other.start.getY();
        double x4 = other.end.getX(), y4 = other.end.getY();

        double denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
        if (denom == 0.0) return false;  // parallel

        double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
        if (ua < 0.0 || ua > 1.0) return false;

        double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;
        if (ub < 0.0 || ub > 1.0) return false;

        return true;
    }
};
