#pragma once

#include <cmath>

class Point2D {
private:
    double x;
    double y;

public:
    Point2D(double x, double y) : x(x), y(y) {};

    double getX() const { return x; }
    double getY() const { return y; }

    double distanceTo(const Point2D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return sqrt(dx * dx + dy * dy);
    }
};