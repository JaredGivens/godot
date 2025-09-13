#ifndef SSK_GEOMETRY_H
#define SSK_GEOMETRY_H

#include "collider.h"
#include "core/math/vector3"
#include "core/object/ref_counted.h"
#include "core/templated/rid.h"
#include "dual_contour.h"
#include "navigation.h"
#include "rendering.h"
#include "shardscape.h"

namespace SSK {

class Faces {
	struct Face {
		int8_t pos[4];
		// 6 bits for each theta
		int8_t theta_phi[4];
		// 1 bit for flip 1 bit for each upper theta
		int8_t flip;
		int8_t texture;
		// final 2 bits of each phi
		int8_t phi_ends;
		// final bits of each pos
		int8_t pos_ends;
	};
	Face faces[sizeof(Face) * Chunk::kSize * 3];

public:
	void set_face_vert(int32_t flat, int32_t i, Vector3 unit_pos);
	void set_face_norm(int32_t flat, int32_t i, Vector3 n);
};

class Geometry : public RefCounted {
	GDCLASS(Geometry, RefCounted);
	static constexpr int32_t kMapDimLen = 8;
	static constexpr int32_t kMapDimLen2 = kMapDimLen * kMapDimLen;
	static constexpr int32_t kMapDimLen3 = kMapDimLen * kMapDimLen2;
	static DisplayOptions Options;
	static Vector3i BlockInd(Vector3i block) {
		return Glob::div_floor(block, Chunk::kDimLen);
	}
	Faces *_faces;
	Rendering *_rendering;
	Navigation *_navigation;
	Collider *_collider;
	//ConcavePolygonShape3D _hullShape = new();
	RID _body;
	DualContour *_dual_contour;
	Transform3D _tsf;
	Geometry(RID scenario, RID space, RID navMap);
	~Geometry();
	void add_vertices(Chunk cache);
	void disable();

protected:
	static void _bind_methods();

public:
	void rebuild(Chunk chunk);
	void update(Chunk chunk);
};
} //namespace SSK
#endif
