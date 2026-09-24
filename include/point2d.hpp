#pragma once

#include <cmath>

class Point2D {
private:
    double x;
    double y;

public:
    Point2D() : x(0.0), y(0.0) {}
    Point2D(double x, double y) : x(x), y(y) {};

    double getX() const { return x; }
    double getY() const { return y; }

    Point2D operator+(const Point2D& rhs) const { return Point2D(x + rhs.x, y + rhs.y); }
    Point2D operator-(const Point2D& rhs) const { return Point2D(x - rhs.x, y - rhs.y); }
    Point2D operator*(double s) const { return Point2D(x * s, y * s); }
    Point2D operator/(double s) const { return Point2D(x / s, y / s); }
    Point2D operator-() const { return Point2D(-x, -y); }

    bool operator==(const Point2D& rhs) const { return x == rhs.x && y == rhs.y; }
    bool operator!=(const Point2D& rhs) const { return !(*this == rhs); }

    double dot(const Point2D& rhs) const { return x * rhs.x + y * rhs.y; }
    double cross(const Point2D& rhs) const { return x * rhs.y - y * rhs.x; }

    double magnitude() const { return std::sqrt(x * x + y * y); }

    Point2D normalized() const {
        double m = magnitude();
        if (m == 0.0) return Point2D(0.0, 0.0);
        return Point2D(x / m, y / m);
    }

    double distanceTo(const Point2D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return sqrt(dx * dx + dy * dy);
    }

    double distanceSquaredTo(const Point2D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return dx * dx + dy * dy;
    }
};

// A position plus heading (radians). Plain RRT only plans over (x, y);
// the heading is carried through so start/goal can be specified as poses.
struct Pose2D {
    Point2D point;
    double theta;

    Pose2D(double x, double y, double theta = 0.0) : point(x, y), theta(theta) {}
    Pose2D(const Point2D& p, double theta = 0.0) : point(p), theta(theta) {}
};
