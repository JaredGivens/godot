#include "QuadChunk16.h"
#include "Global.h"
#include "core/variant/variant.h"
#include "vector3u4.h"
#include <cstring>
#include <thirdparty/zstd/zstd.h>

#define GB Global::get_singleton()

namespace SSK {

QuadChunk16::QuadChunk16() {
	buffer_.resize_initialized(sizeof(Block) * SIZE_3D);
}

void QuadChunk16::_bind_methods() {
    ClassDB::bind_integer_constant("QuadChunk16", StringName(), "SIZE_1D", SIZE_1D);
    ClassDB::bind_integer_constant("QuadChunk16", StringName(), "SIZE_2D", SIZE_2D);
    ClassDB::bind_integer_constant("QuadChunk16", StringName(), "SIZE_3D", SIZE_3D);
    ClassDB::bind_integer_constant("QuadChunk16", StringName(), "SUB_GRID_SCALE", SUB_GRID_SCALE);

    BIND_ENUM_CONSTANT(QUAD_DIR_EMPTY);
    BIND_ENUM_CONSTANT(QUAD_DIR_POS);
    BIND_ENUM_CONSTANT(QUAD_DIR_NEG);

    ClassDB::bind_method(D_METHOD("get_count"), &QuadChunk16::get_count);
    ClassDB::bind_method(D_METHOD("get_addr"), &QuadChunk16::get_addr);
    ClassDB::bind_method(D_METHOD("set_block", "blocki", "position", "blockId"), &QuadChunk16::set_block);
    ClassDB::bind_method(D_METHOD("get_quad", "blocki", "trii"), &QuadChunk16::get_quad);
    ClassDB::bind_method(D_METHOD("get_compressed"), &QuadChunk16::get_compressed);
    ClassDB::bind_method(D_METHOD("from_compressed", "data"), &QuadChunk16::from_compressed);
}

int32_t QuadChunk16::get_count() const { return count_; }

PackedInt64Array QuadChunk16::get_addr() const {
    PackedInt64Array arr;
    arr.push_back(static_cast<int64_t>(reinterpret_cast<intptr_t>(this)));
    return arr;
}

bool QuadChunk16::set_block(int32_t blocki, Vector3i pos, int32_t blockId) {
    if (blocki < 0 || blocki >= SIZE_3D) return false;
    Block *b = blocks() + blocki;
    bool changed = (b->x != (pos.x & 31)) || (b->y != (pos.y & 31)) || (b->z != (pos.z & 31)) || (b->blockId != blockId);
    b->x = pos.x & 31;
    b->y = pos.y & 31;
    b->z = pos.z & 31;
    count_ += (blockId != 0) - (b->blockId != 0);
    b->blockId = blockId;
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
    Block const *b7 = blocks() + v3u47.packed;

    int32_t bid3 = b3->blockId;
    int32_t bid7 = b7->blockId;
	int32_t prop3 = GB->get_block_props(bid3);
	int32_t prop7 = GB->get_block_props(bid7);
    int32_t dir = ((prop3 & Global::PROPERTY_GAS) == 0) | ((prop7 & Global::PROPERTY_GAS) == 0) << 1;

    //int32_t use_trs = -(dir == 3 && bid3 != bid7);
    //bool trs3 = (prop3 & Global::PROPERTY_TRANSLUCENT) != 0;
    //bool trs7 = (prop7 & Global::PROPERTY_TRANSLUCENT) != 0;
    //dir = (use_trs & ((trs3 && !trs7) << 0 | (trs7 && !trs3) << 1)) | (~use_trs & dir);

    if (dir != QUAD_DIR_NEG && dir != QUAD_DIR_POS) return 1;

    if ((qt == 1) ^ (quadi == 0 ^ dir == QUAD_DIR_NEG)) {
        SWAP(b1, b3);
        SWAP(v3u41, v3u43);
    }

    Vector3 vt0 = Vector3(v3u40) + Vector3(b0->x, b0->y, b0->z) / (float)SUB_GRID_SCALE;
    Vector3 vt1 = Vector3(v3u41) + Vector3(b1->x, b1->y, b1->z) / (float)SUB_GRID_SCALE;
    Vector3 vt3 = Vector3(v3u43) + Vector3(b3->x, b3->y, b3->z) / (float)SUB_GRID_SCALE;

    if ((vt3 - vt0).cross((vt1 - vt0)).is_zero_approx()) return 1;

    outFloats[0] = vt0.x; outFloats[1] = vt0.y; outFloats[2] = vt0.z;
    outFloats[3] = vt1.x; outFloats[4] = vt1.y; outFloats[5] = vt1.z;
    outFloats[6] = vt3.x; outFloats[7] = vt3.y; outFloats[8] = vt3.z;
    return 0;
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

    Block const *b0 = blocks() + v3u40.packed;
    Block const *b1 = blocks() + v3u41.packed;
    Block const *b2 = blocks() + v3u42.packed;
    Block const *b3 = blocks() + v3u43.packed;
    Block const *b7 = blocks() + v3u47.packed;

    int32_t bid3 = b3->blockId;
    int32_t bid7 = b7->blockId;
	int32_t prop3 = GB->get_block_props(bid3);
	int32_t prop7 = GB->get_block_props(bid7);
    int32_t dir = ((prop3 & Global::PROPERTY_GAS) == 0) | ((prop7 & Global::PROPERTY_GAS) == 0) << 1;

    //int32_t use_trs = -(dir == 3 && bid3 != bid7);
    //bool trs3 = (prop3 & Global::PROPERTY_TRANSPARENT) != 0;
    //bool trs7 = (prop7 & Global::PROPERTY_TRANSPARENT) != 0;
    //dir = (use_trs & ((trs3 && !trs7) << 0 | (trs7 && !trs3) << 1)) | (~use_trs & dir);

    Vector3i vt0 = Vector3i(v3u40) * SUB_GRID_SCALE + Vector3i(b0->x, b0->y, b0->z);
    Vector3i vt1 = Vector3i(v3u41) * SUB_GRID_SCALE + Vector3i(b1->x, b1->y, b1->z);
    Vector3i vt2 = Vector3i(v3u42) * SUB_GRID_SCALE + Vector3i(b2->x, b2->y, b2->z);
    Vector3i vt3 = Vector3i(v3u43) * SUB_GRID_SCALE + Vector3i(b3->x, b3->y, b3->z);

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

    // Resize local buffer and decompress in-place
    buffer_.resize_initialized((int)dSize);

    size_t const r = ZSTD_decompress(buffer_.ptr(), (size_t)dSize, cbuf, csize);
    if (ZSTD_isError(r) || r != (size_t)dSize) {
        // decompression failed
        return false;
    }

    return true;
}

} // namespace SSK
