#ifndef SSK_FACES_H
#define SSK_FACES_H

#include <core/math/vector3.h>
#include <core/object/ref_counted.h>
namespace SSK {
constexpr int32_t kSize = 32;
constexpr int32_t kSize2 = kSize * kSize;
constexpr int32_t kSize3 = kSize2 * kSize;
struct Face {
	// 3 bits for x y z for each vertex
	int8_t pos[4] = { 0 };
	// final bit of each pos 1 * 4
	int8_t pos_ends = 0;
	// 6 bits for each theta
	// 2 bits for each phi begin
	int8_t theta_phi[4] = { 0 };
	// 3 bits for enabled 3 bits for flip
	int8_t tx = 0;
	int8_t enabled_flip_tx = 0;
	// final 2 bits of each phi 2 * 4
	int8_t phi_ends = 0;
};
class Faces : public RefCounted {
	GDCLASS(Faces, RefCounted);

protected:
	static void _bind_methods();

public:
	int32_t count_;
	Face data_[kSize3 * 3];
	void set_face_vert(int32_t flat, int32_t i, Vector3 unit_pos);
	void set_face_norm(int32_t flat, int32_t i, Vector3 n);
};
} //namespace SSK
#endif
