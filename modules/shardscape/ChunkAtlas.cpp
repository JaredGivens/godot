#include "ChunkAtlas.h"
#include "modules/shardscape/QuadChunk16.h"
#include <core/os/memory.h>
#include <servers/rendering/rendering_device.h>

namespace SSK {

constexpr int32_t CHUNK_BLOCKS_1D = 64;
constexpr int32_t ATLAS_PIXELS_1D = ChunkAtlas::SIZE_1D * CHUNK_BLOCKS_1D;

// chunki is the C# Glob.Flat(v, SIZE_1D) = x + y*SIZE_1D + z*SIZE_1D^2, so each axis is one
// ATLAS_BITS-wide bitfield. That decomposition only holds while SIZE_1D is a power of 2.
static_assert((ChunkAtlas::SIZE_1D & (ChunkAtlas::SIZE_1D - 1)) == 0, "SIZE_1D must be a power of 2");
constexpr int32_t log2_of(int32_t v) {
    return v <= 1 ? 0 : 1 + log2_of(v >> 1);
}
constexpr int32_t ATLAS_BITS = log2_of(ChunkAtlas::SIZE_1D);
constexpr int32_t ATLAS_MASK = ChunkAtlas::SIZE_1D - 1;

ChunkAtlas::ChunkAtlas() {
	RenderingDevice::TextureFormat format;
    format.format = RenderingDevice::DataFormat::DATA_FORMAT_R32_SINT;
    format.width = ATLAS_PIXELS_1D;
    format.height = ATLAS_PIXELS_1D;
    format.mipmaps = 1;
    format.texture_type = RenderingDevice::TextureType::TEXTURE_TYPE_2D_ARRAY;
    format.array_layers = SIZE_1D;
    format.usage_bits = RenderingDevice::TextureUsageBits::TEXTURE_USAGE_STORAGE_BIT |
                        RenderingDevice::TextureUsageBits::TEXTURE_USAGE_CAN_UPDATE_BIT |
                        RenderingDevice::TextureUsageBits::TEXTURE_USAGE_SAMPLING_BIT;
	RenderingDevice::TextureView view;
    RenderingDevice *rd = RenderingDevice::get_singleton();
	texture_ = rd->texture_create(format, view);
	cells_ = rd->storage_buffer_create(SIZE_3D * PASSES * LIST_STRIDE * sizeof(uint16_t));
	counts_ = rd->storage_buffer_create(SIZE_3D * PASSES * sizeof(uint32_t));
	ranks_ = rd->storage_buffer_create(SIZE_3D * sizeof(QuadChunk16::ranks_));
	counts_cpu_.resize(SIZE_3D * PASSES);
	counts_cpu_.fill(0);
}

RID ChunkAtlas::get_texture() const {
    return texture_;
}

RID ChunkAtlas::get_cell_buffer() const {
    return cells_;
}

RID ChunkAtlas::get_count_buffer() const {
    return counts_;
}

RID ChunkAtlas::get_rank_buffer() const {
    return ranks_;
}

PackedInt32Array ChunkAtlas::get_counts() const {
    return counts_cpu_;
}

void ChunkAtlas::set_chunk(int chunki, Ref<QuadChunk16> qc) {
    int atlas_x = (chunki >> 0) & ATLAS_MASK;
    int atlas_y = (chunki >> ATLAS_BITS) & ATLAS_MASK;
    int atlas_z = (chunki >> (ATLAS_BITS * 2)) & ATLAS_MASK;
    int offset_x = atlas_x * CHUNK_BLOCKS_1D;
    int offset_z = atlas_z * CHUNK_BLOCKS_1D;
    RenderingDevice *rd = RenderingDevice::get_singleton();
    RWLockRead guard(qc->lock_);
    rd->texture_update_region(texture_, atlas_y, offset_x, offset_z, CHUNK_BLOCKS_1D, CHUNK_BLOCKS_1D, qc->buffer_.ptr());

    uint16_t lists[PASSES][LIST_STRIDE];
    uint32_t counts[PASSES];
    for (int32_t pass = 0; pass < PASSES; pass++) {
        counts[pass] = qc->draw_cells(pass == 1, lists[pass]);
        counts_cpu_.set(chunki * PASSES + pass, counts[pass]);
        if (counts[pass] == 0) continue;
        uint32_t padded = (counts[pass] + 1) & ~1u;
        if (padded != counts[pass]) lists[pass][counts[pass]] = 0;
        uint32_t offset = (chunki * PASSES + pass) * LIST_STRIDE * sizeof(uint16_t);
        rd->buffer_update(cells_, offset, padded * sizeof(uint16_t), lists[pass]);
    }
    rd->buffer_update(counts_, chunki * PASSES * sizeof(uint32_t), sizeof(counts), counts);
    rd->buffer_update(ranks_, chunki * sizeof(qc->ranks_), sizeof(qc->ranks_), qc->ranks_);
}

// Runs at ObjectDB cleanup, which can be after the RD already dropped the texture — free only what
// is still live, or shutdown reports "Attempted to free invalid ID" with no script frame to blame.
ChunkAtlas::~ChunkAtlas() {
    RenderingDevice *rd = RenderingDevice::get_singleton();
	if (rd != nullptr && texture_.is_valid() && rd->texture_is_valid(texture_)) {
		rd->free_rid(texture_);
		rd->free_rid(cells_);
		rd->free_rid(counts_);
		rd->free_rid(ranks_);
	}
	texture_ = RID();
	cells_ = RID();
	counts_ = RID();
	ranks_ = RID();
}

void ChunkAtlas::_bind_methods() {
	BIND_CONSTANT(SIZE_1D)
	BIND_CONSTANT(SIZE_2D)
	BIND_CONSTANT(SIZE_3D)
    ClassDB::bind_method(D_METHOD("get_texture"), &ChunkAtlas::get_texture);
    ClassDB::bind_method(D_METHOD("set_chunk", "chunki", "qc"), &ChunkAtlas::set_chunk);
    ClassDB::bind_method(D_METHOD("get_cell_buffer"), &ChunkAtlas::get_cell_buffer);
    ClassDB::bind_method(D_METHOD("get_count_buffer"), &ChunkAtlas::get_count_buffer);
    ClassDB::bind_method(D_METHOD("get_rank_buffer"), &ChunkAtlas::get_rank_buffer);
    ClassDB::bind_method(D_METHOD("get_counts"), &ChunkAtlas::get_counts);
}
}
