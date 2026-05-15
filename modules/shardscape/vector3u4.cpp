#include "vector3u4.h"
#include <cmath>

Vector3u4::Vector3u4(int32_t xi, int32_t yi, int32_t zi) {
    set(xi, yi, zi);
}

void Vector3u4::set(int xi, int yi, int zi) {
    packed = 0;
    invalid = ((unsigned)xi > MAX_VALUE || (unsigned)yi > MAX_VALUE || (unsigned)zi > MAX_VALUE);
    x |= xi & MAX_VALUE;
    y |= yi & MAX_VALUE;
    z |= zi & MAX_VALUE;
}

Vector3u4 Vector3u4::try_add(int axis, int value) {
    ERR_FAIL_COND_V_EDMSG(axis < 0 || axis > 2, *this, "Vector3u4 axis out of range");
    if (is_invalid()) return *this;   // stay invalid
    int c[3] = {(int)x, (int)y, (int)z};
    c[axis] += value;
    set(c[0], c[1], c[2]);
    return *this;
}

Vector3u4 Vector3u4::try_add(int dx, int dy, int dz) {
    if (is_invalid()) return *this;   // stay invalid
    set(x + dx, y + dy, z + dz);
    return *this;
}

int Vector3u4::operator[](int axis) const {
    ERR_FAIL_COND_V_EDMSG(axis < 0 || axis > 2, 0, "Vector3u4 axis out of range");
    return  packed >> (AXIS_SHIFT * axis) & MAX_VALUE;
}

Vector3u4::operator Vector3() const {
    return Vector3((real_t)operator[](0), (real_t)operator[](1), (real_t)operator[](2));
}

Vector3u4::operator Vector3i() const {
    return Vector3i(operator[](0), operator[](1), operator[](2));
}
