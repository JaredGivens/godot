#include "vector3u4.h"
#include <cmath>

Vector3u4::Vector3u4(int32_t xi, int32_t yi, int32_t zi) {
    set(xi, yi, zi);
}

void Vector3u4::set(int32_t xi, int32_t yi, int32_t zi) {
    packed = 0;
    // Check if any value exceeds 4-bit unsigned (0-15) or is negative
    if (xi < 0 || xi > MAX_VALUE || yi < 0 || yi > MAX_VALUE || zi < 0 || zi > MAX_VALUE) {
        invalid = 1;
        store_with_overflow(0, xi);
        store_with_overflow(1, yi);
        store_with_overflow(2, zi);
    } else {
        x = xi;
        y = yi;
        z = zi;
        invalid = 0;
    }
}

Vector3u4 Vector3u4::try_add(int axis, int value) {
    ERR_FAIL_COND_V_EDMSG(axis < 0 || axis > 2, *this, "Vector3u4 axis out of range");
    int comp[3];

    for (int i = 0; i < 3; i++) {
        comp[i] = (is_invalid()) ? reconstruct_value(i) : (*this)[i];
    }

    comp[axis] += value;
    set(comp[0], comp[1], comp[2]);
    return *this;
}

Vector3u4 Vector3u4::try_add(int dx, int dy, int dz) {
    int n[3];
    if (is_invalid()) {
        n[0] = reconstruct_value(0);
        n[1] = reconstruct_value(1);
        n[2] = reconstruct_value(2);
    } else {
        n[0] = x; n[1] = y; n[2] = z;
    }

    set(n[0] + dx, n[1] + dy, n[2] + dz);
    return *this;
}

int Vector3u4::operator[](int axis) const {
    ERR_FAIL_COND_V_EDMSG(axis < 0 || axis > 2, 0, "Vector3u4 axis out of range");
    return is_invalid() ? reconstruct_value(axis) : (packed >> (AXIS_SHIFT * axis)) & MAX_VALUE;
}

Vector3u4::operator Vector3() const {
    return Vector3((real_t)operator[](0), (real_t)operator[](1), (real_t)operator[](2));
}

Vector3u4::operator Vector3i() const {
    return Vector3i(operator[](0), operator[](1), operator[](2));
}

void Vector3u4::store_with_overflow(int axis, int value) {
    int avalue = std::abs(value);
    uint32_t shift = AXIS_SHIFT * axis;

    // Mask and store lower 4 bits
    packed = (packed & ~(MAX_VALUE << shift)) | (avalue & MAX_VALUE) << shift;
    // Store 5th bit (overflow)
    packed = (packed & ~(1 << (BIT_SHIFT + axis))) | ((avalue >> 4) & 1) << (BIT_SHIFT + axis);
    // Store sign bit
    packed = (packed & ~(1 << (SIGN_SHIFT + axis))) | (uint32_t)(value < 0) << (SIGN_SHIFT + axis);
}

int Vector3u4::reconstruct_value(int axis) const {
    uint32_t shift = AXIS_SHIFT * axis;
    int value = ((packed >> shift) & MAX_VALUE) | (((packed >> (BIT_SHIFT + axis)) & 1) << 4);
    if ((packed >> (SIGN_SHIFT + axis)) & 1) {
        value = -value;
    }
    return value;
}
