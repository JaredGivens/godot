#pragma once

#include "core/object/class_db.h"
#include "core/object/ref_counted.h"
#include "core/variant/variant.h"
#include "core/math/vector3.h"
#include "core/math/vector3i.h"
#include "vector3u4.h"
#include <cstdint>

namespace SSK {

#pragma pack(push, 1)
struct Block {
    uint16_t index;
    uint16_t blockId;

    uint32_t x : 5;
    uint32_t y : 5;
    uint32_t z : 5;
    uint32_t padding0 : 17;

    uint32_t padding1;
    uint32_t padding2;
};
#pragma pack(pop)

class QuadChunk16 : public RefCounted {
    GDCLASS(QuadChunk16, RefCounted);

public:
    static constexpr int32_t SIZE_1D = 16;
    static constexpr int32_t SIZE_2D = SIZE_1D * SIZE_1D;
    static constexpr int32_t SIZE_3D = SIZE_2D * SIZE_1D;


    enum QuadDir {
        QUAD_DIR_EMPTY,
        QUAD_DIR_POS,
        QUAD_DIR_NEG,
    };

    int32_t get_count() const;
    PackedInt64Array get_addr() const;
    void set_buffer(int32_t *p_buffer);
    void set_block(int32_t blocki, Vector3i pos, int32_t blockId);
    int get_tri(int32_t blocki, int32_t trii, float *outFloats) const;
    PackedInt32Array get_quad(int32_t blocki, int32_t trii) const;

private:
    int32_t count_ = 0;
    int32_t *buffer_ = nullptr;

    inline Block *blocks() { return reinterpret_cast<Block *>(buffer_); }
    inline Block const *blocks() const { return reinterpret_cast<Block const *>(buffer_); }

protected:
    static void _bind_methods();
};

} // namespace SSK

VARIANT_ENUM_CAST(SSK::QuadChunk16::QuadDir);
