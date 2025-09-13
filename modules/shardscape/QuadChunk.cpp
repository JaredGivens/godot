#include "QuadChunk.h"
#include "core/math/math_defs.h"
#include "core/math/math_funcs.h"
#include "core/object/class_db.h"
#include "core/templates/local_vector.h"
#include "core/variant/variant.h"
#include "vector3u5.h"
#include <limits>

namespace SSK {
void QuadChunk::_bind_methods() {
	BIND_CONSTANT(SIZE_1D)
	BIND_CONSTANT(SIZE_2D)
	BIND_CONSTANT(SIZE_3D)
	BIND_ENUM_CONSTANT(QUAD_STATE_DISABLED)
	BIND_ENUM_CONSTANT(QUAD_STATE_INSIDE)
	BIND_ENUM_CONSTANT(QUAD_STATE_OUTSIDE)
	ClassDB::bind_method(D_METHOD("get_count"), &QuadChunk::get_count);
	ClassDB::bind_method(D_METHOD("get_addr"), &QuadChunk::get_addr);
	ClassDB::bind_method(D_METHOD("get_buffer"), &QuadChunk::get_buffer);
	ClassDB::bind_method(D_METHOD("set_vert", "verti", "position"), &QuadChunk::set_vert);
	ClassDB::bind_method(D_METHOD("set_quad", "hexi", "state", "texture"), &QuadChunk::set_quad);
	ClassDB::bind_method(D_METHOD("get_block", "hexi", "trii"), &QuadChunk::get_block);
}

QuadChunk::QuadChunk() : buffer_()  {
	buffer_.resize_initialized(SIZE_3D * 8);
	auto hex = hexes();
	for (int32_t i = 0; i < SIZE_3D; ++i){
		hex[i].index = i;
	}
}

void QuadChunk::set_vert(int32_t hexi, int32_t verti, Vector3i pos) {
	auto hex = hexes() + hexi;
	hex->pos[verti].x = pos.x & 31;
	hex->pos[verti].y = pos.y & 31;
	hex->pos[verti].z = pos.z & 31;
}

void QuadChunk::set_quad(int32_t hexi, int32_t quadi, QuadState state, int16_t tx) {
	auto hex = hexes() + hexi;
	auto ss = quadi * QUAD_STATE_SHIFT;
	count_ += (state != QUAD_STATE_DISABLED) - (((hex->states >> ss) & QUAD_STATE_MASK) != QUAD_STATE_DISABLED);
	hex->states = (hex->states & ~(QUAD_STATE_MASK << ss)) | (state << ss);
	hex->textures[quadi] = tx;
}

// returns 0 on success and 1 on fail
int QuadChunk::get_tri(int32_t hexi, int32_t trii, float *outFloats) const {
    Hex const * hex = hexes() + hexi;
    int32_t quadi = trii >> 1;
    int32_t state = (hex->states >> (quadi * 2)) & 3;
    if (state == QUAD_STATE_DISABLED) {
        return 1;
    }
    auto v3u5 = Vector3u5(hexi);

    auto v0 = Vector3i(v3u5.x, v3u5.y, v3u5.z);
    auto v1 = Vector3i(quadi == 1, quadi == 2, quadi == 0);
    auto v2 = Vector3i(quadi == 2, quadi == 0, quadi == 1);
    v1 += float(~trii & 1) * v2;
    v2 += float(trii & 1) * v1;
    v1 += v0;
    v2 += v0;

	// 0 1 2 > 0 2 1 > 1 4 2
    int32_t ax1 = 1 << (quadi + (quadi & 1) - (quadi >> 1));
	// 0 1 2 > 1 0 2 > 2 1 4
    int32_t ax2 = 1 << (quadi ^ ((quadi ^ 2) >> 1));
    int32_t i1 = ax1 + ((~trii & 1) * ax2);
    int32_t i2 = ax2 + ((trii & 1) * ax1);

    if (state == QUAD_STATE_OUTSIDE) {
        Vector3 tmp = v1;
        v1 = v2;
        v2 = tmp;
        i1 ^= i2;
        i2 ^= i1;
        i1 ^= i2;
    }

    v0 = v0 * 24 + Vector3i(hex->pos[00].x, hex->pos[00].y, hex->pos[00].z);
    v1 = v1 * 24 + Vector3i(hex->pos[i1].x, hex->pos[i1].y, hex->pos[i1].z);
    v2 = v2 * 24 + Vector3i(hex->pos[i2].x, hex->pos[i2].y, hex->pos[i2].z);

	if (Vector3(v2 - v0).cross(Vector3(v1 - v0)).is_zero_approx()) {
		return 1;
	}

    outFloats[0] = float(v0.x) / 24.0f;
    outFloats[1] = float(v0.y) / 24.0f;
    outFloats[2] = float(v0.z) / 24.0f;
    outFloats[3] = float(v1.x) / 24.0f;
    outFloats[4] = float(v1.y) / 24.0f;
    outFloats[5] = float(v1.z) / 24.0f;
    outFloats[6] = float(v2.x) / 24.0f;
    outFloats[7] = float(v2.y) / 24.0f;
    outFloats[8] = float(v2.z) / 24.0f;
	return 0;
}

Vector<int32_t> QuadChunk::get_block(int32_t hexi, int32_t trii) const {
    Hex const * hex0 = hexes() + hexi;
    int32_t quadi = trii >> 1;
    int32_t state = (hex0->states >> (quadi * 2)) & 3;
    auto v3u5 = Vector3u5(hexi);

    auto v0 = Vector3i(v3u5.x, v3u5.y, v3u5.z);
    auto v1 = Vector3i(quadi == 1, quadi == 2, quadi == 0);
    auto v2 = Vector3i(quadi == 2, quadi == 0, quadi == 1);
    auto v3 = Vector3i(quadi == 2, quadi == 0, quadi == 1);
    v3 += v1;
    v1 += v0;
    v2 += v0;
    v3 += v0;

    int32_t ax1 = 1 << (quadi + (quadi & 1) - (quadi >> 1));
    int32_t ax2 = 1 << (quadi ^ ((quadi ^ 2) >> 1));
    int32_t i1 = ax1;
    int32_t i2 = ax2;
    int32_t i3 = ax1 + ax2;

    if (state == QUAD_STATE_OUTSIDE) {
        Vector3i tmp = v1;
        v1 = v2;
        v2 = tmp;
        i1 ^= i2;
        i2 ^= i1;
        i1 ^= i2;
    }

    auto vt0 = v0 * 24 + Vector3i(hex0->pos[00].x, hex0->pos[00].y, hex0->pos[00].z);
    auto vt1 = v1 * 24 + Vector3i(hex0->pos[i1].x, hex0->pos[i1].y, hex0->pos[i1].z);
    auto vt2 = v2 * 24 + Vector3i(hex0->pos[i2].x, hex0->pos[i2].y, hex0->pos[i2].z);
    auto vt3 = v3 * 24 + Vector3i(hex0->pos[i3].x, hex0->pos[i3].y, hex0->pos[i3].z);

    auto diff = (state & QUAD_STATE_OUTSIDE) - 1;
	v3u5.try_add(quadi, diff);
	LocalVector<int32_t> result;
    result.resize(26);
	result[0] = state;
	result[1] = hex0->textures[quadi];
	result[2] = vt0.x;
	result[3] = vt0.y;
	result[4] = vt0.z;
	result[5] = vt1.x;
	result[6] = vt1.y;
	result[7] = vt1.z;
	result[8] = vt2.x;
	result[9] = vt2.y;
	result[10] = vt2.z;
	result[11] = vt3.x;
	result[12] = vt3.y;
	result[13] = vt3.z;
	result[14] = std::numeric_limits<int32_t>::max();

	if(v3u5.invalid) {
        return Vector<int32_t>(result);
	}

    auto hex1 = hexes() + v3u5.packed;
    v0[quadi] += diff;
    v1[quadi] += diff;
    v2[quadi] += diff;
    v3[quadi] += diff;
    Vector3i tmp = v1;
    v1 = v2;
    v2 = tmp;
    i1 ^= i2;
    i2 ^= i1;
    i1 ^= i2;

    auto vt4 = v0 * 24 + Vector3i(hex1->pos[00].x, hex1->pos[00].y, hex1->pos[00].z);
    auto vt5 = v1 * 24 + Vector3i(hex1->pos[i1].x, hex1->pos[i1].y, hex1->pos[i1].z);
    auto vt6 = v2 * 24 + Vector3i(hex1->pos[i2].x, hex1->pos[i2].y, hex1->pos[i2].z);
    auto vt7 = v3 * 24 + Vector3i(hex1->pos[i3].x, hex1->pos[i3].y, hex1->pos[i3].z);

	result[14] = vt4.x;
	result[15] = vt4.y;
	result[16] = vt4.z;
	result[17] = vt5.x;
	result[18] = vt5.y;
	result[19] = vt5.z;
	result[20] = vt6.x;
	result[21] = vt6.y;
	result[22] = vt6.z;
	result[23] = vt7.x;
	result[24] = vt7.y;
	result[25] = vt7.z;
	return  Vector<int32_t>(result);
}
} //namespace SSK
