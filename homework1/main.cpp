#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
#include "./transformation/transform.h"

int main()
{

    std::cout << "Example of transformation:" << std::endl;
    Eigen::Vector3f raw_point(2, 1, 1);
    std::cout << "raw point:\n" << raw_point << std::endl;
    std::cout << translate2d(Eigen::Vector2f(1, 2)) * (angle2radian(45)) * raw_point << std::endl;

    return 0;
}