#pragma once
#include "core/error/error_macros.h"
#include <cstdint>
#include <stdexcept>
#include <cmath>

struct Vector3u5 {
    static constexpr int BIT_SHIFT = 16;
    static constexpr int SIGN_SHIFT = 19;
    static constexpr int AXIS_SHIFT = 5;
    static constexpr int MAX_VALUE = (1 << AXIS_SHIFT) - 1; // 31

    union {
        uint32_t packed;
        struct {
            uint32_t x     : 5;  // bits 0–4
            uint32_t y     : 5;  // bits 5–9
            uint32_t z     : 5;  // bits 10–14
            uint32_t invalid : 1; // bit 15
            uint32_t x_bit5 : 1;  // bit 16
            uint32_t y_bit5 : 1;  // bit 19
            uint32_t z_bit5 : 1;  // bit 22
            uint32_t x_sign : 1;  // bit 17
            uint32_t y_sign : 1;  // bit 20
            uint32_t z_sign : 1;  // bit 23
            uint32_t _unused : 7; // bits 25–31
        };
    };

    // Constructors
    Vector3u5() : packed(0) {}
    explicit Vector3u5(uint32_t p) : packed(p) {}
    Vector3u5(int32_t xi, int32_t yi, int32_t zi) {
        set(xi, yi, zi);
    }
    void set(int32_t xi, int32_t yi, int32_t zi) {
        packed = 0;
        if (xi < 0 || xi > MAX_VALUE ||
            yi < 0 || yi > MAX_VALUE ||
            zi < 0 || zi > MAX_VALUE) {
            store_with_overflow(0, xi);
            store_with_overflow(1, yi);
            store_with_overflow(2, zi);
            invalid = 1;
        } else {
            x = xi & MAX_VALUE;
            y = yi & MAX_VALUE;
            z = zi & MAX_VALUE;
        }
    }

    // Validity
    bool is_invalid() const { return invalid != 0; }

    // Add operations
    Vector3u5 try_add(int axis, int value) {
        ERR_FAIL_COND_V_EDMSG(axis < 0 || axis > 2, Vector3u5(), "Vector3u5 axis out of range");
        int comp[3];

        if (is_invalid()) {
            comp[0] = reconstruct_value(0);
            comp[1] = reconstruct_value(1);
            comp[2] = reconstruct_value(2);
        } else {
            comp[0] = x;
            comp[1] = y;
            comp[2] = z;
        }

        comp[axis] += value;

        set(comp[0], comp[1], comp[2]);
        return *this;
    }

    Vector3u5 try_add(int dx, int dy, int dz) {
        int nx, ny, nz;

        if (is_invalid()) {
            nx = reconstruct_value(0);
            ny = reconstruct_value(1);
            nz = reconstruct_value(2);
        } else {
            nx = x;
            ny = y;
            nz = z;
        }

        nx += dx; ny += dy; nz += dz;
        set(nx, ny, nz);
        return *this;
    }

    int operator[](int axis) const {
        ERR_FAIL_COND_V_EDMSG(axis < 0 || axis > 2, 0, "Vector3u5 axis out of range");
        
        if (is_invalid()) {
            return reconstruct_value(axis);
        }
        
        return (packed >> (AXIS_SHIFT * axis)) & MAX_VALUE;
    }

private:
    void store_with_overflow(int axis, int value) {
        // lower 5 bits stored in x/y/z field
        int avalue = abs(value);
        auto shift = AXIS_SHIFT * axis;
        packed = (packed & ~(MAX_VALUE << shift)) | (avalue & MAX_VALUE) << shift;
        packed = (packed & ~(1 << (BIT_SHIFT + axis))) | ((avalue >> 5) & 1) << (BIT_SHIFT + axis);
        packed = (packed & ~(1 << (SIGN_SHIFT + axis))) | (value < 0) << (SIGN_SHIFT + axis);
    }

    int reconstruct_value(int axis) const {
        auto shift = AXIS_SHIFT * axis;
        int value = ((packed >> shift) & MAX_VALUE) | ((packed >> (BIT_SHIFT + axis) & 1) << 5);
        if (packed >> (SIGN_SHIFT + axis) & 1) {
            value = -value; 
        }
        return value;
    }
};

