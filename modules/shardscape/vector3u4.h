#pragma once
#include "core/error/error_macros.h"
#include "core/math/vector3.h"
#include "core/math/vector3i.h"
#include <cstdint>

struct Vector3u4 {
    static constexpr int BIT_SHIFT = 13;
    static constexpr int SIGN_SHIFT = 16;
    static constexpr int AXIS_SHIFT = 4;
    static constexpr int MAX_VALUE = (1 << AXIS_SHIFT) - 1;

    union {
        uint32_t packed;
        struct {
            uint32_t x        : 4;  // bits 0–3
            uint32_t y        : 4;  // bits 4–7
            uint32_t z        : 4;  // bits 8–11
            uint32_t invalid  : 1;  // bit 12
            uint32_t _unused  : 19; // bits 19-31
        };
    };

    Vector3u4() : packed(0) {}
    explicit Vector3u4(uint32_t p) : packed(p) {}
    Vector3u4(int32_t xi, int32_t yi, int32_t zi);

    void set(int32_t xi, int32_t yi, int32_t zi);
    bool is_invalid() const { return invalid != 0; }

    Vector3u4 try_add(int axis, int value);
    Vector3u4 try_add(int dx, int dy, int dz);

    int operator[](int axis) const;
    operator Vector3() const;
    operator Vector3i() const;
};
