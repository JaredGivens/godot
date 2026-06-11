#include "ChunkAtlas.h"
#include "modules/shardscape/QuadChunk16.h"
#include <core/os/memory.h>
#include <servers/rendering/rendering_device.h>

namespace SSK {

constexpr int32_t CHUNK_BLOCKS_1D = 64;
constexpr int32_t ATLAS_PIXELS_1D = ChunkAtlas::SIZE_1D * CHUNK_BLOCKS_1D;

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
}

RID ChunkAtlas::get_texture() const {
    return texture_;
}

void ChunkAtlas::set_chunk(int chunki, Ref<QuadChunk16> qc) {
    int atlas_x = (chunki >> 0) & 15;
    int atlas_y = (chunki >> 4) & 15;
    int atlas_z = (chunki >> 8) & 15;
    int offset_x = atlas_x * 64;
    int offset_z = atlas_z * 64;
    RenderingDevice::get_singleton()->texture_update_region(texture_, atlas_y, offset_x, offset_z, 64, 64, qc->buffer_.ptr());
}

ChunkAtlas::~ChunkAtlas() {
    RenderingDevice *rd = RenderingDevice::get_singleton();
	rd->free_rid(texture_);
}

void ChunkAtlas::_bind_methods() {
	BIND_CONSTANT(SIZE_1D)
	BIND_CONSTANT(SIZE_2D)
	BIND_CONSTANT(SIZE_3D)
    ClassDB::bind_method(D_METHOD("get_texture"), &ChunkAtlas::get_texture);
    ClassDB::bind_method(D_METHOD("set_chunk", "chunki", "qc"), &ChunkAtlas::set_chunk);
}
}
