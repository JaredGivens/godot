#pragma once
#include "core/variant/variant.h"
#include <core/math/vector3.h>
#include <core/object/ref_counted.h>
namespace SSK {
struct Hex {
	uint16_t index : 15;
	uint16_t paddingi: 1;
	// 0,0,0
	// 0,0,1
	// 0,1,0
	// 0,1,1
	// 1,0,0
	// 1,0,1
	// 1,1,0
    struct {
        uint16_t x : 5;
        uint16_t y : 5;
        uint16_t z : 5;
		uint16_t paddingv: 1;
    } pos[7];

    uint16_t textures[3];
    uint16_t states : 6;
	// z0 z1 z2 z3
	uint16_t paddings: 10;
    uint32_t padding0;
    uint32_t padding2;
} __attribute__((packed));

class QuadChunk : public RefCounted {
	GDCLASS(QuadChunk, RefCounted);

protected:
	static void _bind_methods();

public:
	static constexpr int32_t SIZE_1D = 16;
	static constexpr int32_t SIZE_2D = SIZE_1D * SIZE_1D;
	static constexpr int32_t SIZE_3D = SIZE_2D * SIZE_1D;
	enum QuadState {
		 QUAD_STATE_DISABLED,
		 QUAD_STATE_INSIDE,
		 QUAD_STATE_OUTSIDE,
	};
	void set_buffer(float * buffer);
	static constexpr int32_t QUAD_STATE_SHIFT = 2;
	static constexpr int32_t QUAD_STATE_MASK = 3;
	int32_t count_ = 0;
	float *buffer_;
	Hex *hexes() { return reinterpret_cast<Hex *>(buffer_); }
	Hex const *hexes() const { return reinterpret_cast<Hex const *>(buffer_); }
	int32_t get_count() const { return count_; };
	PackedInt64Array get_addr() const { return { static_cast<int64_t>(reinterpret_cast<intptr_t>(this)) }; }
	void set_vert(int32_t hexi, int32_t verti, Vector3i vert);
	void set_quad(int32_t hexi, int32_t quadi, QuadState state, int32_t tx);
	int get_tri(int32_t hexi, int32_t trii, float *verts) const;
	PackedInt32Array get_quad(int32_t hexi, int32_t trii) const;
};
} //namespace SSK

VARIANT_ENUM_CAST(SSK::QuadChunk::QuadState);
