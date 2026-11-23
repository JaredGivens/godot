#pragma once

#include "core/templates/local_vector.h"
#include <core/object/ref_counted.h>
#include <core/os/rw_lock.h>
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
    static constexpr int32_t kSize1D = 16;
    static constexpr int32_t kSize2D = kSize1D * kSize1D;
    static constexpr int32_t kSize3D = kSize2D * kSize1D;
    static constexpr int32_t kSubGridScale = 16;

    // Mirrors of the ChunkData cell layout / Glob constants used by the ported DualContour.
    static constexpr int32_t kCellMaxDist = 4;   // Glob.CellMaxDist
    static constexpr int32_t kBlockDist = kCellMaxDist / 2;
    static constexpr int32_t kChunkDimLen = 33;  // ChunkData.DimLen
    static constexpr int32_t kCidMask = 0x1FFF;
    static constexpr int32_t kMetaShift = 13;
    static constexpr int32_t kMetaMask = 0xF;
    static constexpr int32_t kDistMask = 0x7;
    static constexpr int32_t kDistXShift = 17;
    static constexpr int32_t kDistBits = 3;

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

    // Ported DualContour: full/partial remesh over a pinned ChunkData (cells + palette bids).
    // Addresses are raw int32_t*/uint16_t* passed as int64 (see get_addr precedent); nothing copied.
    void remesh_all(int64_t cells_addr, int64_t palette_addr, Vector3i quad_chunk_diff, bool is_empty);
    void remesh_block(int64_t cells_addr, int64_t palette_addr, Vector3i quad_chunk_diff, Vector3i cell);

    enum DrawRank {
        DRAW_EMPTY,
        DRAW_TRANSLUCENT,
        DRAW_CUTOUT,
        DRAW_OPAQUE,
    };

    uint32_t draw_cells(bool translucent, uint16_t *out) const;

    static constexpr uint32_t kRankLow = 0x15555555u;
    static constexpr int32_t kPortalBid = 2;

    static inline uint32_t occ(uint32_t row) { return (row | (row >> 1)) & 0x55555555u; }

    inline bool occupied(int32_t blocki) const { return ((ranks_[blocki >> 4] >> ((blocki & 15) * 2)) & 3) != 0; }

    inline uint32_t face_row(int32_t axis, int32_t y, int32_t z) const {
        uint32_t hi = occ(ranks_[(y + 1) | ((z + 1) << 4)]);
        if (axis == 0) {
            return (hi ^ (hi >> 2)) & kRankLow;
        }
        uint32_t lo = occ(ranks_[axis == 1 ? (y | ((z + 1) << 4)) : ((y + 1) | (z << 4))]);
        return ((lo ^ hi) >> 2) & kRankLow;
    }

    int32_t occupied_bounds(Vector3i &lo, Vector3i &hi) const;

    int32_t count_ = 0;
    int32_t gas_count_ = 0;
    LocalVector<uint8_t> buffer_;
    uint32_t ranks_[kSize2D] = {};
    uint16_t portal_rows_[kSize2D] = {};
    RWLock lock_;

private:
    struct Cell {
        Vector3i vert;
        int32_t bid;
        int32_t automata;
    };

    Cell mesh_cell(const int32_t *cells, const uint16_t *palette, Vector3i cd, bool is_empty) const;

    inline Block *blocks() { return reinterpret_cast<Block *>(buffer_.ptr()); }
    inline Block const *blocks() const { return reinterpret_cast<Block const *>(buffer_.ptr()); }

    Vector3i mesh_vert(const int32_t *cells, const uint16_t *palette, Vector3i cell) const;
    static Vector3 solve_intersection(const Vector3 *mids, const Vector3 *norms, int32_t edge_amt);

protected:
    static void _bind_methods();
};

} // namespace SSK

VARIANT_ENUM_CAST(SSK::QuadChunk16::QuadDir);
