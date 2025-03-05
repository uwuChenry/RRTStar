
#include <iostream>
#include "../include/point2d.hpp"

int main(){
    Point2D p1(3.0, 4.0);
    Point2D p2(0.0, 0.0);
    std::cout << "Distance: " << p1.distanceTo(p2) << std::endl;
    return 0;
}