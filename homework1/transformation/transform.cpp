#include "transform.h"
#define Pi acos(-1)

Matrix3f translate2d(const Vector2f& s) {
    Matrix3f res = Matrix3f::Identity();
    res(0, 2) = s[0];
    res(1, 2) = s[1];
    return res;
}

float angle2radian(float a)
{
    return a / 180 * Pi;
}

float radian2angle(float r) {
    return r / Pi * 180;
}

// s is given by radian
Matrix3f rotate2d(float s, const Vector2f& p)
{
    Matrix3f res = Matrix3f::Identity();
    res(0, 0) = std::cos(s);
    res(0, 1) = -std::sin(s);
    res(1, 0) = std::sin(s);
    res(1, 1) = std::cos(s);

    return translate2d(p) * res * translate2d(-p);
}
