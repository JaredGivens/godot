#include "faces.h"
#include "core/math/vector3i.h"

namespace SSK {
void Faces::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_face_vert", "flat", "i", "unit_pos"), &Faces::set_face_vert);
	ClassDB::bind_method(D_METHOD("set_face_norm", "flat", "i", "normal"), &Faces::set_face_norm);
}

void Faces::set_face_vert(int32_t flat, int32_t i, Vector3 unit_pos) {
	Vector3i q = Vector3i((unit_pos * 8).round()).clampi(1, 7);
	auto &face = data_[flat];
	face.pos[i] = (q.x & 7) | ((q.y & 7) << 3) | ((q.z & 3) << 6);
	face.pos_ends = (face.pos_ends & ~(1 << i)) | ((q.z & 4) << i);
}

void Faces::set_face_norm(int32_t flat, int32_t i, Vector3 n) {
	int32_t ax = flat % 3;
	real_t phir = Math::acos(abs(n[ax])); // z = cos(phi), so phi = acos(z)
	real_t thetar = Math::atan2(n[(ax + 1) % 3], n[(ax + 2) % 3]) + Math_PI; // theta = atan2(y, x)
	int8_t theta = Math::round(thetar / Math_TAU * 64);
	int8_t phi = Math::round(phir / Math_PI * 32);

	auto &face = data_[flat];
	face.theta_phi[i] = (theta & 63) | ((phi & 3) << 6);
	int32_t shift = i * 2;
	face.phi_ends = (face.phi_ends & ~(3 << shift)) | ((phi & 12) << shift);
}
} //namespace SSK
