#pragma once

#include "core/templates/local_vector.h"
#include <core/object/ref_counted.h>
#include <core/variant/variant.h>
#include <core/math/vector3.h>
#include <core/math/vector3i.h>
#include <cstdint>

namespace SSK {

#pragma pack(push, 1)
struct Block {
    uint32_t blockId : 13;
    uint32_t active : 1;
    uint32_t dir : 3;
    uint32_t x : 5;
    uint32_t y : 5;
    uint32_t z : 5;
};
#pragma pack(pop)

class QuadChunk16 : public RefCounted {
    GDCLASS(QuadChunk16, RefCounted);

public:
    static constexpr int32_t SIZE_1D = 16;
    static constexpr int32_t SIZE_2D = SIZE_1D * SIZE_1D;
    static constexpr int32_t SIZE_3D = SIZE_2D * SIZE_1D;
    static constexpr int32_t SUB_GRID_SCALE = 24;

    enum QuadDir {
        QUAD_DIR_EMPTY,
        QUAD_DIR_POS,
        QUAD_DIR_NEG,
    };

    QuadChunk16();
    int32_t get_count() const;
    PackedInt64Array get_addr() const;
    bool set_block(int32_t blocki, Vector3i pos, int32_t blockId, int32_t automata);
    int get_tri(int32_t blocki, int32_t trii, float *outFloats) const;
    PackedVector3Array get_tri_verts(int32_t blocki, int32_t trii) const;
    PackedInt32Array get_quad(int32_t blocki, int32_t trii) const;
    PackedByteArray get_compressed() const;
    bool from_compressed(PackedByteArray data);
    int32_t count_ = 0;
    LocalVector<uint8_t> buffer_;

private:

    inline Block *blocks() { return reinterpret_cast<Block *>(buffer_.ptr()); }
    inline Block const *blocks() const { return reinterpret_cast<Block const *>(buffer_.ptr()); }

protected:
    static void _bind_methods();
};

} // namespace SSK

VARIANT_ENUM_CAST(SSK::QuadChunk16::QuadDir);
