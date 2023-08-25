#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
using namespace Eigen;

Matrix3f translate2d(const Vector2f& s);

Matrix3f rotate2d(float s, const Vector2f& p = Eigen::Vector2f(0, 0));

float angle2radian(float a);

float radian2angle(float r);
