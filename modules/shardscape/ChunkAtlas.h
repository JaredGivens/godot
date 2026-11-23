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
    // Sub-chunks per atlas axis, power of 2. Mirror of the C# Env.SubChunk.MapDimLen
    // (= Glob.MapDimLen * 2) — Glob.CheckMapMirrors() errors at startup if they drift.
    static constexpr int32_t SIZE_1D = 8;
    static constexpr int32_t SIZE_2D = SIZE_1D * SIZE_1D;
    static constexpr int32_t SIZE_3D = SIZE_2D * SIZE_1D;
    static constexpr int32_t LIST_STRIDE = 15 * 15 * 15 + 1;
    static constexpr int32_t PASSES = 2;

    ChunkAtlas();
    ~ChunkAtlas();

    RID get_texture() const;
    RID get_cell_buffer() const;
    RID get_count_buffer() const;
    RID get_rank_buffer() const;
    PackedInt32Array get_counts() const;
	void set_chunk(int chunki, Ref<QuadChunk16> qc);

private:
	RID texture_;
	RID cells_;
	RID counts_;
	RID ranks_;
	PackedInt32Array counts_cpu_;
};


} // namespace SSK
