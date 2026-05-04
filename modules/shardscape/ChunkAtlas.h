#pragma once
#include "QuadChunk16.h"
#include <core/object/ref_counted.h>
#include <servers/rendering/rendering_device.h>

namespace SSK {


class ChunkAtlas : public RefCounted {
    GDCLASS(ChunkAtlas, RefCounted);

protected:
    static void _bind_methods();

public:
    static constexpr int32_t SIZE_1D = 16;
    static constexpr int32_t SIZE_2D = SIZE_1D * SIZE_1D;
    static constexpr int32_t SIZE_3D = SIZE_2D * SIZE_1D;

    ChunkAtlas();
    ~ChunkAtlas();

    Ref<QuadChunk16> get_quad_chunk(int chunki) const;
    RID get_texture() const ;
	void update_chunk(int chunki);

private:
	RID texture_;
    LocalVector<uint8_t> layer_arrays_[SIZE_1D];
    LocalVector<Ref<QuadChunk16>> quad_chunks_;
};


} // namespace SSK
