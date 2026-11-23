#include "QuadChunk16.h"
#include "Global.h"
#include "core/variant/variant.h"
#include "vector3u4.h"
#include <core/math/basis.h>
#include <cstring>
#include <thirdparty/zstd/zstd.h>

#define GB Global::get_singleton()

namespace SSK {

static bool is_gas(int32_t props) {
    int32_t opacity = props & 0xFF;
    return opacity != 0 && (props & Global::PROPERTY_CUTOUT) == 0 && (props & Global::PROPERTY_GAS) != 0
            && (opacity == 255 || (props & Global::PROPERTY_SHARP) != 0);
}

static uint32_t draw_rank(int32_t props) {
    if (props & Global::PROPERTY_CUTOUT) return QuadChunk16::DRAW_CUTOUT;
    int32_t opacity = props & 0xFF;
    if (opacity == 0) return QuadChunk16::DRAW_EMPTY;
    return opacity < 255 && !is_gas(props) ? QuadChunk16::DRAW_TRANSLUCENT : QuadChunk16::DRAW_OPAQUE;
}

static uint32_t occupancy_rank(int32_t bid, int32_t props) {
    return bid == QuadChunk16::kPortalBid ? QuadChunk16::DRAW_EMPTY : draw_rank(props);
}

static bool is_face(int32_t bid0, int32_t props0, int32_t bid1, int32_t props1) {
    uint32_t rank0 = draw_rank(props0);
    uint32_t rank1 = draw_rank(props1);
    if (rank0 != rank1) return true;
    if (rank0 == QuadChunk16::DRAW_TRANSLUCENT) return bid0 != bid1;
    if (rank0 == QuadChunk16::DRAW_OPAQUE) return is_gas(props0) != is_gas(props1);
    return false;
}

static bool is_solid(int32_t props) {
    return draw_rank(props) >= QuadChunk16::DRAW_CUTOUT && !is_gas(props);
}

uint32_t QuadChunk16::draw_cells(bool translucent, uint16_t *out) const {
    const Block *b = blocks();
    uint32_t count = 0;
    for (int32_t z = 0; z < kSize1D - 1; z++) {
        for (int32_t y = 0; y < kSize1D - 1; y++) {
            int32_t r0 = (y + 1) | ((z + 1) << 4);
            uint32_t hi = ranks_[r0] >> 2;
            uint32_t los[3] = { ranks_[r0], ranks_[y | ((z + 1) << 4)] >> 2, ranks_[(y + 1) | (z << 4)] >> 2 };
            uint32_t bits = 0;
            for (int32_t q = 0; q < 3; q++) {
                uint32_t lo = los[q];
                uint32_t diff = lo ^ hi;
                uint32_t any = lo | hi;
                uint32_t pairs;
                if (translucent) {
                    uint32_t below = ~(any >> 1);
                    bits |= diff & below;
                    pairs = lo & hi & below & kRankLow;
                } else {
                    bits |= (diff | (diff >> 1)) & (any >> 1);
                    pairs = gas_count_ == 0 ? 0u : lo & hi & (lo >> 1) & (hi >> 1) & kRankLow;
                }
                for (; pairs != 0; pairs &= pairs - 1) {
                    int32_t x = __builtin_ctz(pairs) >> 1;
                    int32_t hii = (x + 1) | ((y + 1) << 4) | ((z + 1) << 8);
                    int32_t loi = hii - (1 << (4 * q));
                    bool face = translucent
                            ? b[hii].blockId != b[loi].blockId
                            : is_gas(GB->get_block_props(b[hii].blockId)) != is_gas(GB->get_block_props(b[loi].blockId));
                    if (face) {
                        bits |= 1u << (x * 2);
                    }
                }
            }
            if (!translucent) {
                uint32_t p = portal_rows_[r0];
                uint32_t portal = (p | (p >> 1) | (portal_rows_[y | ((z + 1) << 4)] >> 1)
                        | (portal_rows_[(y + 1) | (z << 4)] >> 1)) & 0x7FFFu;
                for (; portal != 0; portal &= portal - 1) {
                    bits |= 1u << (__builtin_ctz(portal) * 2);
                }
            }
            for (uint32_t m = bits & kRankLow; m != 0; m &= m - 1) {
                out[count++] = (__builtin_ctz(m) >> 1) | (y << 4) | (z << 8);
            }
        }
    }
    return count;
}

QuadChunk16::QuadChunk16() {
	buffer_.resize_initialized(sizeof(Block) * kSize3D);
}

void QuadChunk16::_bind_methods() {
    ClassDB::bind_integer_constant("QuadChunk16", StringName(), "SIZE_1D", kSize1D);
    ClassDB::bind_integer_constant("QuadChunk16", StringName(), "SIZE_2D", kSize2D);
    ClassDB::bind_integer_constant("QuadChunk16", StringName(), "SIZE_3D", kSize3D);
    ClassDB::bind_integer_constant("QuadChunk16", StringName(), "SUB_GRID_SCALE", kSubGridScale);

    BIND_ENUM_CONSTANT(QUAD_DIR_EMPTY);
    BIND_ENUM_CONSTANT(QUAD_DIR_POS);
    BIND_ENUM_CONSTANT(QUAD_DIR_NEG);

    ClassDB::bind_method(D_METHOD("get_count"), &QuadChunk16::get_count);
    ClassDB::bind_method(D_METHOD("get_addr"), &QuadChunk16::get_addr);
    ClassDB::bind_method(D_METHOD("set_block", "blocki", "position", "blockId", "automata"), &QuadChunk16::set_block);
    ClassDB::bind_method(D_METHOD("get_tri_verts", "blocki", "trii"), &QuadChunk16::get_tri_verts);
    ClassDB::bind_method(D_METHOD("get_quad", "blocki", "trii"), &QuadChunk16::get_quad);
    ClassDB::bind_method(D_METHOD("get_compressed"), &QuadChunk16::get_compressed);
    ClassDB::bind_method(D_METHOD("from_compressed", "data"), &QuadChunk16::from_compressed);
    ClassDB::bind_method(D_METHOD("remesh_all", "cells_addr", "palette_addr", "quad_chunk_diff", "is_empty"), &QuadChunk16::remesh_all);
    ClassDB::bind_method(D_METHOD("remesh_block", "cells_addr", "palette_addr", "quad_chunk_diff", "cell"), &QuadChunk16::remesh_block);
}

int32_t QuadChunk16::get_count() const { return count_; }

PackedInt64Array QuadChunk16::get_addr() const {
    PackedInt64Array arr;
    arr.push_back(static_cast<int64_t>(reinterpret_cast<intptr_t>(this)));
    return arr;
}

bool QuadChunk16::set_block(int32_t blocki, Vector3i pos, int32_t blockId, int32_t automata) {
    if (blocki < 0 || blocki >= kSize3D) return false;
    bool active = automata & 1;
    int32_t dir = automata >> 1;
    Block *b = blocks() + blocki;
    bool changed = (b->x != (pos.x & 31)) || (b->y != (pos.y & 31)) || (b->z != (pos.z & 31))
            || (b->blockId != (blockId & 0x1FFF)) || (b->active != (uint32_t)active) || (b->dir != (dir & 7));
    b->x = pos.x & 31;
    b->y = pos.y & 31;
    b->z = pos.z & 31;
    count_ += (blockId != 0) - (b->blockId != 0);
    if (b->blockId != (uint32_t)(blockId & 0x1FFF)) {
        int32_t old_props = GB->get_block_props(b->blockId);
        int32_t new_props = GB->get_block_props(blockId & 0x1FFF);
        gas_count_ += (int32_t)is_gas(new_props) - (int32_t)is_gas(old_props);
        int32_t shift = (blocki & 15) * 2;
        uint32_t &row = ranks_[blocki >> 4];
        row = (row & ~(3u << shift)) | (occupancy_rank(blockId & 0x1FFF, new_props) << shift);
        uint16_t portal_bit = (uint16_t)(1u << (blocki & 15));
        uint16_t &portal_row = portal_rows_[blocki >> 4];
        portal_row = (blockId & 0x1FFF) == kPortalBid ? (portal_row | portal_bit) : (portal_row & ~portal_bit);
    }
    b->blockId = blockId;
    b->active = active;
    b->dir = dir;
    return changed;
}

int QuadChunk16::get_tri(int32_t blocki, int32_t trii, float *outFloats) const {
    int32_t quadi = trii >> 1;
    int32_t qt = trii & 1;

    Vector3u4 v3u40(blocki);
	if (v3u40.x == 15 || v3u40.y == 15 || v3u40.z == 15) {
		return 1;
	}
    Vector3u4 v3u41 = v3u40;
    v3u41.try_add(~quadi & (1 << (qt ^ 0)), 1);

    Vector3u4 v3u43 = v3u41;
    v3u43.try_add(~quadi & (1 << (qt ^ 1)), 1);

    Vector3u4 v3u47 = v3u43;
    v3u47.try_add(quadi, 1);

    Block const *b0 = blocks() + v3u40.packed;
    Block const *b1 = blocks() + v3u41.packed;
    Block const *b3 = blocks() + v3u43.packed;

    int32_t dir = (int32_t)occupied(v3u43.packed) | (int32_t)occupied(v3u47.packed) << 1;

    if (dir != QUAD_DIR_NEG && dir != QUAD_DIR_POS) return 1;

    if ((qt == 1) ^ (quadi == 0 ^ dir == QUAD_DIR_NEG)) {
        SWAP(b1, b3);
        SWAP(v3u41, v3u43);
    }

    Vector3 vt0 = Vector3(v3u40) + Vector3(b0->x, b0->y, b0->z) / (float)kSubGridScale;
    Vector3 vt1 = Vector3(v3u41) + Vector3(b1->x, b1->y, b1->z) / (float)kSubGridScale;
    Vector3 vt3 = Vector3(v3u43) + Vector3(b3->x, b3->y, b3->z) / (float)kSubGridScale;

    if ((vt3 - vt0).cross((vt1 - vt0)).is_zero_approx()) return 1;

    outFloats[0] = vt0.x; outFloats[1] = vt0.y; outFloats[2] = vt0.z;
    outFloats[3] = vt1.x; outFloats[4] = vt1.y; outFloats[5] = vt1.z;
    outFloats[6] = vt3.x; outFloats[7] = vt3.y; outFloats[8] = vt3.z;
    return 0;
}

int32_t QuadChunk16::occupied_bounds(Vector3i &lo, Vector3i &hi) const {
    uint32_t xs = 0;
    int32_t count = 0;
    lo = Vector3i(kSize1D, kSize1D, kSize1D);
    hi = Vector3i(-1, -1, -1);
    for (int32_t row = 0; row < kSize2D; row++) {
        uint32_t o = occ(ranks_[row]);
        if (o == 0) {
            continue;
        }
        xs |= o;
        count += __builtin_popcount(o);
        int32_t y = row & 15;
        int32_t z = row >> 4;
        lo.y = MIN(lo.y, y);
        lo.z = MIN(lo.z, z);
        hi.y = MAX(hi.y, y);
        hi.z = MAX(hi.z, z);
    }
    if (count == 0) {
        lo = Vector3i();
        hi = Vector3i(kSize1D - 1, kSize1D - 1, kSize1D - 1);
        return 0;
    }
    lo.x = __builtin_ctz(xs) >> 1;
    hi.x = (31 - __builtin_clz(xs)) >> 1;
    return count;
}

PackedVector3Array QuadChunk16::get_tri_verts(int32_t blocki, int32_t trii) const {
    PackedVector3Array res;
    float f[9];
    RWLockRead guard(lock_);
    if (get_tri(blocki, trii, f) != 0) return res;
    res.resize(3);
    Vector3 *w = res.ptrw();
    w[0] = Vector3(f[0], f[1], f[2]);
    w[1] = Vector3(f[3], f[4], f[5]);
    w[2] = Vector3(f[6], f[7], f[8]);
    return res;
}

PackedInt32Array QuadChunk16::get_quad(int32_t blocki, int32_t trii) const {
    int32_t quadi = trii >> 1;
    int32_t qt = trii & 1;

    Vector3u4 v3u40(blocki);
    Vector3u4 v3u41 = v3u40;
    v3u41.try_add(~quadi & (1 << (qt ^ 0)), 1);

    Vector3u4 v3u42 = v3u40;
    v3u42.try_add(~quadi & (1 << (qt ^ 1)), 1);

    Vector3u4 v3u43 = v3u41;
    v3u43.try_add(~quadi & (1 << (qt ^ 1)), 1);

    Vector3u4 v3u47 = v3u43;
    v3u47.try_add(quadi, 1);

    RWLockRead guard(lock_);
    Block const *b0 = blocks() + v3u40.packed;
    Block const *b1 = blocks() + v3u41.packed;
    Block const *b2 = blocks() + v3u42.packed;
    Block const *b3 = blocks() + v3u43.packed;

    int32_t dir = (int32_t)occupied(v3u43.packed) | (int32_t)occupied(v3u47.packed) << 1;

    Vector3i vt0 = Vector3i(v3u40) * kSubGridScale + Vector3i(b0->x, b0->y, b0->z);
    Vector3i vt1 = Vector3i(v3u41) * kSubGridScale + Vector3i(b1->x, b1->y, b1->z);
    Vector3i vt2 = Vector3i(v3u42) * kSubGridScale + Vector3i(b2->x, b2->y, b2->z);
    Vector3i vt3 = Vector3i(v3u43) * kSubGridScale + Vector3i(b3->x, b3->y, b3->z);

    PackedInt32Array res;
    res.resize(14);
    int32_t *w = res.ptrw();
    w[0]  = vt0.x; w[1]  = vt0.y; w[2]  = vt0.z;
    w[3]  = vt1.x; w[4]  = vt1.y; w[5]  = vt1.z;
    w[6]  = vt2.x; w[7]  = vt2.y; w[8]  = vt2.z;
    w[9]  = vt3.x; w[10] = vt3.y; w[11] = vt3.z;
    w[12] = dir;   w[13] = dir == QUAD_DIR_POS ? v3u43.packed : v3u47.packed;

    return res;
}

PackedByteArray QuadChunk16::get_compressed() const {
    PackedByteArray out;

    RWLockRead guard(lock_);
    const void *src_ptr = buffer_.ptr();
    size_t src_size = buffer_.size();
    if (count_ == 0 || src_ptr == nullptr) {
        return out;
    }

    // Single-call compression at level 5
    size_t const max_csize = ZSTD_compressBound(src_size);
    LocalVector<uint8_t> tmp;
    tmp.resize(max_csize);

    size_t const csize = ZSTD_compress(tmp.ptr(), max_csize, src_ptr, src_size, 5);
    if (ZSTD_isError(csize)) {
        // compression failed
        return out;
    }

    out.resize((int)csize);
    return (PackedByteArray)out;
}

bool QuadChunk16::from_compressed(PackedByteArray in) {
    const void *cbuf = in.ptr();
    size_t csize = (size_t)in.size();
    if (csize == 0 || cbuf == nullptr) {
        return false;
    }

    unsigned long long dSize = ZSTD_getFrameContentSize(cbuf, csize);
    if (dSize == ZSTD_CONTENTSIZE_ERROR || dSize == ZSTD_CONTENTSIZE_UNKNOWN) {
        // Cannot determine decompressed size
        return false;
    }

    LocalVector<uint8_t> decoded;
    decoded.resize_initialized((int)dSize);

    size_t const r = ZSTD_decompress(decoded.ptr(), (size_t)dSize, cbuf, csize);
    if (ZSTD_isError(r) || r != (size_t)dSize || dSize != sizeof(Block) * kSize3D) {
        return false;
    }

    const Block *b = reinterpret_cast<const Block *>(decoded.ptr());
    uint32_t ranks[kSize2D];
    uint16_t portal_rows[kSize2D];
    int32_t gas_count = 0;
    for (int32_t row = 0; row < kSize2D; row++) {
        uint32_t packed = 0;
        uint16_t portal = 0;
        for (int32_t x = 0; x < kSize1D; x++) {
            int32_t bid = b[(row << 4) | x].blockId;
            int32_t props = GB->get_block_props(bid);
            packed |= occupancy_rank(bid, props) << (x * 2);
            portal |= (uint16_t)((bid == kPortalBid) << x);
            gas_count += is_gas(props);
        }
        ranks[row] = packed;
        portal_rows[row] = portal;
    }

    RWLockWrite guard(lock_);
    buffer_ = std::move(decoded);
    memcpy(ranks_, ranks, sizeof(ranks_));
    memcpy(portal_rows_, portal_rows, sizeof(portal_rows_));
    gas_count_ = gas_count;
    return true;
}

// Ported from DualContour.SolveIntersection: least-squares plane intersection via the
// normal equations, regularized to stay solvable for 2 / near-parallel planes.
Vector3 QuadChunk16::solve_intersection(const Vector3 *mids, const Vector3 *norms, int32_t edge_amt) {
    Vector3 mass_center = Vector3(1, 1, 1) * (float)kBlockDist;
    Vector3 ata_row0, ata_row1, ata_row2, atb;
    for (int32_t i = 0; i < edge_amt; ++i) {
        Vector3 n = norms[i].normalized();
        float b = n.dot(mids[i] - mass_center);
        ata_row0 += n * n.x;
        ata_row1 += n * n.y;
        ata_row2 += n * n.z;
        atb += n * b;
    }
    float alpha = 1e-3f;
    ata_row0.x += alpha;
    ata_row1.y += alpha;
    ata_row2.z += alpha;
    Basis ata = Basis(ata_row0, ata_row1, ata_row2).transposed();
    if (Math::abs(ata.determinant()) < 1e-6f) {
        return mass_center;
    }
    return ata.inverse().xform(atb) + mass_center;
}

// Ported from DualContour.MeshVert. `cell` is the ChunkData cell coord; `cells`/`palette`
// are the pinned ChunkData.Cells (packed int) and PaletteBids arrays.
Vector3i QuadChunk16::mesh_vert(const int32_t *cells, const uint16_t *palette, Vector3i cell) const {
    constexpr int32_t EDGE_AMT = 12;
    constexpr int32_t D2 = kChunkDimLen * kChunkDimLen;
    Vector3 mids[EDGE_AMT] = {};
    Vector3 norms[EDGE_AMT] = {};
    int32_t priority = -1;
    int32_t intersections = 0;

    for (int32_t i = 0; i < EDGE_AMT; ++i) {
        int32_t axis = i >> 2;
        int32_t c0[3] = { 0, 0, 0 };
        c0[~axis & 1] += (i >> 0) & 1;
        c0[~axis & 2] += (i >> 1) & 1;
        int32_t c1[3] = { c0[0], c0[1], c0[2] };
        c1[axis] += 1;

        int32_t idx0 = (c0[0] + cell.x) + (c0[1] + cell.y) * kChunkDimLen + (c0[2] + cell.z) * D2;
        int32_t idx1 = (c1[0] + cell.x) + (c1[1] + cell.y) * kChunkDimLen + (c1[2] + cell.z) * D2;
        int32_t raw0 = cells[idx0];
        int32_t raw1 = cells[idx1];
        int32_t bid0 = palette[raw0 & kCidMask];
        int32_t bid1 = palette[raw1 & kCidMask];
        int32_t prop0 = GB->get_block_props(bid0);
        int32_t prop1 = GB->get_block_props(bid1);
        int32_t d0 = (raw0 >> (kDistXShift + kDistBits * axis)) & kDistMask;

        if (is_face(bid0, prop0, bid1, prop1)) {
            int32_t edge_solid = is_solid(prop0) || is_solid(prop1);
            int32_t edge_sharp = ((prop0 | prop1) & Global::PROPERTY_SHARP) != 0;
            int32_t edge_priority = edge_solid * 2 + edge_sharp;
            if (edge_priority > priority) {
                for (int32_t j = 0; j < EDGE_AMT; ++j) {
                    mids[j] = Vector3();
                    norms[j] = Vector3();
                }
                intersections = 0;
                priority = edge_priority;
            }
            if (edge_priority == priority) {
                ++intersections;
                mids[i] = Vector3(c0[0], c0[1], c0[2]) * (float)kCellMaxDist;
                mids[i][axis] += d0;
                norms[i][axis] = 1.0f;
            }
        }
    }

    Vector3 vert;
    if (intersections != 0) {
        if ((priority & 1) != 0) {
            vert = solve_intersection(mids, norms, EDGE_AMT);
        } else {
            Vector3 sum;
            for (int32_t j = 0; j < EDGE_AMT; ++j) {
                sum += mids[j];
            }
            vert = sum / (float)intersections;
        }
    }

    Vector3 scaled = vert * ((float)kSubGridScale / (float)kCellMaxDist);
    Vector3i vert_i((int32_t)Math::round(scaled.x), (int32_t)Math::round(scaled.y), (int32_t)Math::round(scaled.z));
    vert_i = vert_i.clamp(Vector3i(0, 0, 0), Vector3i(kSubGridScale, kSubGridScale, kSubGridScale));
    return intersections == 0 ? Vector3i(31, 31, 31) : vert_i;
}

QuadChunk16::Cell QuadChunk16::mesh_cell(const int32_t *cells, const uint16_t *palette, Vector3i cd, bool is_empty) const {
    int32_t raw = cells[cd.x + cd.y * kChunkDimLen + cd.z * kChunkDimLen * kChunkDimLen];
    return Cell{
        is_empty ? Vector3i(1, 1, 1) * (kBlockDist * kSubGridScale / kCellMaxDist) : mesh_vert(cells, palette, cd),
        palette[raw & kCidMask],
        (raw >> kMetaShift) & kMetaMask,
    };
}

void QuadChunk16::remesh_all(int64_t cells_addr, int64_t palette_addr, Vector3i quad_chunk_diff, bool is_empty) {
    const int32_t *cells = reinterpret_cast<const int32_t *>(cells_addr);
    const uint16_t *palette = reinterpret_cast<const uint16_t *>(palette_addr);
    const Vector3i skirt(1, 1, 1);
    LocalVector<Cell> meshed;
    meshed.resize(kSize3D);
    for (int32_t blocki = 0; blocki < kSize3D; ++blocki) {
        Vector3u4 c(blocki);
        meshed[blocki] = mesh_cell(cells, palette, Vector3i(c) + skirt + quad_chunk_diff, is_empty);
    }
    RWLockWrite guard(lock_);
    for (int32_t blocki = 0; blocki < kSize3D; ++blocki) {
        set_block(blocki, meshed[blocki].vert, meshed[blocki].bid, meshed[blocki].automata);
    }
}

void QuadChunk16::remesh_block(int64_t cells_addr, int64_t palette_addr, Vector3i quad_chunk_diff, Vector3i cell) {
    const int32_t *cells = reinterpret_cast<const int32_t *>(cells_addr);
    const uint16_t *palette = reinterpret_cast<const uint16_t *>(palette_addr);
    const Vector3i skirt(1, 1, 1);
    constexpr int32_t SPACING = 15;
    int32_t indices[27];
    Cell meshed[27];
    int32_t n = 0;
    for (int32_t i = 0; i < 3; ++i) {
        for (int32_t j = 0; j < 3; ++j) {
            for (int32_t k = 0; k < 3; ++k) {
                Vector3i c0 = cell + Vector3i(i, j, k);
                if (c0.x < 0 || c0.y < 0 || c0.z < 0 || c0.x > SPACING || c0.y > SPACING || c0.z > SPACING) {
                    continue;
                }
                indices[n] = Vector3u4(c0.x, c0.y, c0.z).packed;
                meshed[n++] = mesh_cell(cells, palette, c0 + skirt + quad_chunk_diff, false);
            }
        }
    }
    RWLockWrite guard(lock_);
    for (int32_t m = 0; m < n; ++m) {
        set_block(indices[m], meshed[m].vert, meshed[m].bid, meshed[m].automata);
    }
}

} // namespace SSK
