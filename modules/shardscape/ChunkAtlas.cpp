#include "ChunkAtlas.h"
#include "core/os/memory.h"
#include <servers/rendering/renderer_rd/storage_rd/mesh_storage.h>

namespace SSK {
constexpr int32_t CHUNK_SIZE = 8 * QuadChunk::SIZE_3D;

ChunkAtlas::ChunkAtlas() {
    // Allocate buffer for 4^3 quad chunks, each with 32^3 hexes
    // Each hex is 8 floats (from LocalVector<float>)
    int32_t buffer_size = CHUNK_SIZE * SIZE_3D;
    buffer_.resize_initialized(buffer_size);
	ssbo_ = RD::get_singleton()->storage_buffer_create(buffer_size * sizeof(float));

    // Initialize quad chunks with pointers into the buffer
    quad_chunks_.resize(SIZE_3D);
    for (int32_t i = 0; i < SIZE_3D; ++i) {
        quad_chunks_[i] = memnew(QuadChunk);
        float *chunk_buffer = buffer_.ptr() + (i * CHUNK_SIZE);
		quad_chunks_[i]->set_buffer(chunk_buffer);
    }
}

void ChunkAtlas::update_multimesh(RID multimesh, int chunki) const {
    int32_t o = (chunki * CHUNK_SIZE);
	RS::get_singleton()->multimesh_update_buffer(multimesh, buffer_.ptr() + o);
}

void ChunkAtlas::_bind_methods() {
	BIND_CONSTANT(SIZE_1D)
	BIND_CONSTANT(SIZE_2D)
	BIND_CONSTANT(SIZE_3D)
    ClassDB::bind_method(D_METHOD("get_quad_chunk", "chunki"), &ChunkAtlas::get_quad_chunk);
    ClassDB::bind_method(D_METHOD("get_ssbo"), &ChunkAtlas::get_ssbo);
    ClassDB::bind_method(D_METHOD("update_multimesh", "multimesh", "chunki"), &ChunkAtlas::update_multimesh);
}

} // namespace SSK
