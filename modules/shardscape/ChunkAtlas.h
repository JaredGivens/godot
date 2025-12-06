#pragma once
#include "QuadChunk.h"
#include <core/object/ref_counted.h>

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

    Ref<QuadChunk> get_quad_chunk(int chunki) const { return quad_chunks_[chunki]; }
    RID get_ssbo() const { return ssbo_; }
    void update_multimesh(RID multimesh, int chunki) const;

private:
	RID ssbo_;
    LocalVector<float> buffer_;
    LocalVector<Ref<QuadChunk>> quad_chunks_;
};

} // namespace SSK
